"""
    For panda (two-finger) gripper: pushing, pushing-left, pushing-up, pulling, pulling-left, pulling-up
        50% all parts closed, 50% middle (for each part, 50% prob. closed, 50% prob. middle)
        Simulate until static before starting

    REPLAY
"""

import os
import sys
import shutil
import numpy as np
from utils import get_global_position_from_camera
import json
import h5py

from sapien.core import Pose
from env import Env, ContactError
from camera import Camera
from robots.panda_robot import Robot
from robots.shadowhand_robot import ShadowHandRobot

from PIL import Image
from subprocess import call

json_fn = sys.argv[1]
robot_type = None
no_gui = False
# optional args: [robot_type] [no_gui]
if len(sys.argv) > 2 and sys.argv[2].strip() != "":
    robot_type = sys.argv[2]
if len(sys.argv) > 3 and sys.argv[3].strip().lower() in ["no_gui", "headless", "nogui"]:
    no_gui = True
out_dir = '/'.join(json_fn.split('/')[:-1])
with open(json_fn, 'r') as fin:
    replay_data = json.load(fin)

shape_id, _, _, primact_type, _ = json_fn.split('/')[-2].split('_')

# setup env
env = Env(show_gui=(not no_gui))

# setup camera
cam_theta = replay_data['camera_metadata']['theta']
cam_phi = replay_data['camera_metadata']['phi']
cam = Camera(env, theta=cam_theta, phi=cam_phi)
if env.show_gui:
    env.set_controller_camera_pose(cam.pos[0], cam.pos[1], cam.pos[2], np.pi+cam_theta, -cam_phi)

# load shape
object_urdf_fn = '../data/where2act_original_sapien_dataset/%s/mobility_vhacd.urdf' % shape_id
object_material = env.get_material(4, 4, 0.01)
state = replay_data['object_state']
print( 'Object State: %s' % state)
env.load_object(object_urdf_fn, object_material, state=state)
env.set_object_joint_angles(replay_data['joint_angles'])
cur_qpos = env.get_object_qpos()

# simulate some steps for the object to stay rest
still_timesteps = 0
wait_timesteps = 0
while still_timesteps < 5000 and wait_timesteps < 20000:
    env.step()
    env.render()
    cur_new_qpos = env.get_object_qpos()
    invalid_contact = False
    for c in env.scene.get_contacts():
        for p in c.points:
            if abs(p.impulse @ p.impulse) > 1e-4:
                invalid_contact = True
                break
        if invalid_contact:
            break
    if np.max(np.abs(cur_new_qpos - cur_qpos)) < 1e-6 and (not invalid_contact):
        still_timesteps += 1
    else:
        still_timesteps = 0
    cur_qpos = cur_new_qpos
    wait_timesteps += 1

if still_timesteps < 5000:
    print('Object Not Still!')
    env.close()
    exit(1)

### use the GT vision
rgb, depth = cam.get_observation()
object_link_ids = env.movable_link_ids
gt_movable_link_mask = cam.get_movable_link_mask(object_link_ids)

# load the pixel to interact
x, y = replay_data['pixel_locs'][0], replay_data['pixel_locs'][1]
env.set_target_object_part_actor_id(object_link_ids[gt_movable_link_mask[x, y]-1])

# load the random direction in the hemisphere
gripper_direction_cam = np.array(replay_data['gripper_direction_camera'], dtype=np.float32)
gripper_direction_cam /= np.linalg.norm(gripper_direction_cam)
gripper_forward_direction_cam = np.array(replay_data['gripper_forward_direction_camera'], dtype=np.float32)
gripper_left_direction_cam = np.cross(gripper_direction_cam, gripper_forward_direction_cam)
gripper_left_direction_cam /= np.linalg.norm(gripper_left_direction_cam)
gripper_forward_direction_cam = np.cross(gripper_left_direction_cam, gripper_direction_cam)
gripper_forward_direction_cam /= np.linalg.norm(gripper_forward_direction_cam)

# convert to world space
mat44 = np.array(replay_data['camera_metadata']['mat44'], dtype=np.float32).reshape(4, 4)
gripper_direction_world = mat44[:3, :3] @ gripper_direction_cam
gripper_forward_direction_world = mat44[:3, :3] @ gripper_forward_direction_cam

# get pixel 3D position (cam/world)
with h5py.File(json_fn.replace('result.json', 'cam_XYZA.h5'), 'r') as fin:
    cam_XYZA_id1 = fin['id1'][:].astype(np.int64)
    cam_XYZA_id2 = fin['id2'][:].astype(np.int64)
    cam_XYZA_pts = fin['pc'][:].astype(np.float32)
cam_XYZA = cam.compute_XYZA_matrix(cam_XYZA_id1, cam_XYZA_id2, cam_XYZA_pts, depth.shape[0], depth.shape[1])
position_cam = cam_XYZA[x, y, :3]
position_cam_xyz1 = np.ones((4), dtype=np.float32)
position_cam_xyz1[:3] = position_cam
position_world_xyz1 = mat44 @ position_cam_xyz1
position_world = position_world_xyz1[:3]

# compute final pose
up = np.array(gripper_direction_world, dtype=np.float32)
up /= np.linalg.norm(up)
forward = np.array(gripper_forward_direction_world, dtype=np.float32)
left = np.cross(up, forward)
left /= np.linalg.norm(left)
forward = np.cross(left, up)
forward /= np.linalg.norm(forward)
rotmat = np.eye(4).astype(np.float32)
rotmat[:3, 0] = forward
rotmat[:3, 1] = left
rotmat[:3, 2] = up

final_dist = 0.1
if primact_type == 'pushing-left' or primact_type == 'pushing-up':
    final_dist = 0.11

final_rotmat = np.array(rotmat, dtype=np.float32)
final_rotmat[:3, 3] = position_world - up * final_dist
final_pose = Pose().from_transformation_matrix(final_rotmat)

start_rotmat = np.array(rotmat, dtype=np.float32)
start_rotmat[:3, 3] = position_world - up * 0.15
start_pose = Pose().from_transformation_matrix(start_rotmat)

action_direction = None
if 'left' in primact_type:
    action_direction = forward
elif 'up' in primact_type:
    action_direction = left

if action_direction is not None:
    end_rotmat = np.array(rotmat, dtype=np.float32)
    end_rotmat[:3, 3] = position_world - up * final_dist + action_direction * 0.05

# setup robot
robot_material = env.get_material(4, 4, 0.01)
recorded_robot_type = replay_data.get('robot_type', 'panda')
recorded_urdf = replay_data.get('robot_urdf', './robots/shadowhand/shadowhand_ign_shadow_hand_fixed.urdf')

# priority: CLI robot_type > recorded robot_type
robot_type = robot_type or recorded_robot_type
if robot_type == 'shadowhand':
    robot_urdf_fn = recorded_urdf
    robot = ShadowHandRobot(env, robot_urdf_fn, robot_material, open_gripper=('pulling' in primact_type))
else:
    robot_urdf_fn = './robots/panda_gripper.urdf'
    robot = Robot(env, robot_urdf_fn, robot_material, open_gripper=('pulling' in primact_type))
print(f"[Replay] Using robot_type={robot_type}, urdf={robot_urdf_fn}")

# start pose
robot.robot.set_root_pose(start_pose)
env.render()
# save start view
rgb0, _ = cam.get_observation()
Image.fromarray((rgb0*255).astype(np.uint8)).save(os.path.join(out_dir, 'replay_start.png'))

# activate contact checking (relax for shadowhand to avoid early abort)
strict_contact = ('pushing' in primact_type)
if robot_type == 'shadowhand':
    strict_contact = False
env.start_checking_contact(robot.hand_actor_id, robot.gripper_actor_ids, strict_contact)

### wait to start (only if GUI)
if env.show_gui:
    env.wait_to_start()

### main steps
print('Start qpos: ', env.get_target_part_qpos())

target_link_mat44 = env.get_target_part_pose().to_transformation_matrix()
position_local_xyz1 = np.linalg.inv(target_link_mat44) @ position_world_xyz1
print(position_local_xyz1)

if 'pushing' in primact_type:
    robot.close_gripper()
elif 'pulling' in primact_type:
    robot.open_gripper()

try:
    # approach
    robot.move_to_target_pose(final_rotmat, 2000)
    robot.wait_n_steps(2000)
    # save after approach
    rgb1, _ = cam.get_observation()
    Image.fromarray((rgb1*255).astype(np.uint8)).save(os.path.join(out_dir, 'replay_after_approach.png'))

    if 'pulling' in primact_type:
        robot.close_gripper()
        robot.wait_n_steps(2000)

    if 'left' in primact_type or 'up' in primact_type:
        robot.move_to_target_pose(end_rotmat, 2000)
        robot.wait_n_steps(2000)

    if primact_type == 'pulling':
        robot.move_to_target_pose(start_rotmat, 2000)
        robot.wait_n_steps(2000)
except ContactError:
    print('[Replay] ContactError caught, saving current visualization and continuing...')

# final snapshot
rgb2, _ = cam.get_observation()
Image.fromarray((rgb2*255).astype(np.uint8)).save(os.path.join(out_dir, 'replay_end.png'))

target_link_mat44 = env.get_target_part_pose().to_transformation_matrix()
position_world_xyz1_end = target_link_mat44 @ position_local_xyz1
print(position_world_xyz1[:3])
print(position_world_xyz1_end[:3])

print('Final qpos: ', env.get_target_part_qpos())

### if GUI, keep rendering; otherwise exit
if env.show_gui:
    robot.wait_n_steps(100000000000)
else:
    env.close()

