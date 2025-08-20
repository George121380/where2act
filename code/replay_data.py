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
from argparse import ArgumentParser

from sapien.core import Pose
from env import Env, ContactError
from camera import Camera
from robots.panda_robot import Robot
from robots.shadowhand_robot import ShadowHandRobot

from PIL import Image
from subprocess import call

parser = ArgumentParser()
parser.add_argument('json_fn', type=str)
parser.add_argument('--robot_type', type=str, choices=['panda', 'shadowhand'], default=None)
parser.add_argument('--no_gui', action='store_true', default=False)
args = parser.parse_args()

json_fn = args.json_fn
robot_type = args.robot_type
no_gui = args.no_gui

out_dir = '/'.join(json_fn.split('/')[:-1])
with open(json_fn, 'r') as fin:
    replay_data = json.load(fin)

shape_id, _, _, primact_type, _ = json_fn.split('/')[-2].split('_')

# setup env
env = Env(show_gui=(not no_gui))

# setup camera
cam_theta = replay_data['camera_metadata']['theta']
cam_phi = replay_data['camera_metadata']['phi']
cam_dist = replay_data['camera_metadata'].get('dist', 5.0)
cam_fov = replay_data['camera_metadata'].get('fov', 35)
cam = Camera(env, theta=cam_theta, phi=cam_phi, dist=cam_dist, fov=np.rad2deg(cam_fov))
if env.show_gui:
    env.set_controller_camera_pose(cam.pos[0], cam.pos[1], cam.pos[2], np.pi+cam_theta, -cam_phi)

# load shape (respect object_mode recorded during collection)
object_material = env.get_material(4, 4, 0.01)
state = replay_data.get('object_state', 'closed')
print('Object State: %s' % state)
object_mode = replay_data.get('object_mode', 'dataset')
object_urdf_fn = None
if object_mode == 'dataset':
    object_urdf_fn = '../data/where2act_original_sapien_dataset/%s/mobility_vhacd.urdf' % shape_id
elif object_mode == 'asset':
    object_urdf_fn = replay_data.get('asset_urdf', None)
elif object_mode == 'simple':
    object_urdf_fn = replay_data.get('object_urdf_override', replay_data.get('object_urdf_builtin', None))
if object_urdf_fn is None:
    # fallback to dataset path
    object_urdf_fn = '../data/where2act_original_sapien_dataset/%s/mobility_vhacd.urdf' % shape_id
print(f"[Replay] object_mode={object_mode}, urdf={object_urdf_fn}")
env.load_object(object_urdf_fn, object_material, state=state)

# Immediately check if object was loaded correctly
print(f"[DEBUG] After load_object: env.object = {env.object}")
if env.object is not None:
    print(f"[DEBUG] Object loaded successfully, root pose: {env.object.get_root_pose()}")
    # Check object links instead of scene actors
    object_links = env.object.get_links()
    print(f"[DEBUG] Object has {len(object_links)} links")
    for i, link in enumerate(object_links):
        try:
            pose = link.get_pose()
            name = link.get_name() if hasattr(link, 'get_name') else f"link_{i}"
            print(f"[DEBUG] Link {i} ({name}): pos={pose.p}, id={link.get_id()}")
        except Exception as e:
            print(f"[DEBUG] Link {i}: <error getting info: {e}>")
    
    # Force a render step to ensure object is visible
    env.step()
    env.render()
    print(f"[DEBUG] After render step")
else:
    print(f"[DEBUG] ERROR: Object loading failed!")

# For simple objects, we need to position them correctly based on the interaction point
# The interaction point should be on the object surface, so we can use it to position the object
if object_mode == 'simple':
    # For simple objects, place them in the center of camera view for better visibility
    # Calculate camera forward direction and place object in front of camera
    cam_forward = -np.array(cam.pos) / np.linalg.norm(cam.pos)  # Camera looks towards origin
    obj_distance = 0.5  # Place object 0.5m in front of camera
    obj_position = cam.pos + cam_forward * obj_distance
    obj_pose = Pose(p=obj_position.tolist(), q=[1, 0, 0, 0])
    env.object.set_root_pose(obj_pose)
    print(f"[DEBUG] Repositioned simple object to camera center: {obj_position}")
    print(f"[DEBUG] Camera forward: {cam_forward}")

# quick sanity snapshot of object right after loading
rgb_loaded, _ = cam.get_observation()
print(f"[DEBUG] Object loaded, rgb shape: {rgb_loaded.shape}, min/max: {rgb_loaded.min():.3f}/{rgb_loaded.max():.3f}")
print(f"[DEBUG] Camera position: {cam.pos}")
print(f"[DEBUG] Camera mat44: {cam.mat44}")

# Additional debugging for object visibility
if object_mode == 'simple':
    obj_pos = env.object.get_root_pose().p
    cam_to_obj = np.array(obj_pos) - np.array(cam.pos)
    cam_to_obj_dist = np.linalg.norm(cam_to_obj)
    print(f"[DEBUG] Object position: {obj_pos}")
    print(f"[DEBUG] Camera to object distance: {cam_to_obj_dist:.3f}")
    print(f"[DEBUG] Camera to object vector: {cam_to_obj}")
    
    # Check if object is in camera frustum
    cam_forward = -np.array(cam.pos) / np.linalg.norm(cam.pos)
    obj_dot_cam = np.dot(cam_to_obj, cam_forward)
    print(f"[DEBUG] Object dot camera forward: {obj_dot_cam:.3f}")
    
    # Check scene objects
    scene_actors = env.scene.get_all_actors()
    print(f"[DEBUG] Scene has {len(scene_actors)} actors")
    for i, actor in enumerate(scene_actors[:5]):  # Show first 5
        try:
            pose = actor.get_pose()
            print(f"[DEBUG] Actor {i}: {actor.get_name()}, pos={pose.p}")
        except:
            print(f"[DEBUG] Actor {i}: <error getting info>")

Image.fromarray((rgb_loaded*255).astype(np.uint8)).save(os.path.join(out_dir, 'replay_object_loaded.png'))
try:
    from PIL import Image as _Image
    import numpy as _np
    obj_mask = cam.get_object_mask().astype(_np.uint8) * 255
    print(f"[DEBUG] Object mask shape: {obj_mask.shape}, min/max: {obj_mask.min()}/{obj_mask.max()}")
    _Image.fromarray(obj_mask).save(os.path.join(out_dir, 'replay_object_mask.png'))
    # also save movable link mask using segmentation
    link_ids = env.movable_link_ids
    seg_mask = cam.get_movable_link_mask(link_ids).astype(_np.uint8) * 20
    print(f"[DEBUG] Seg mask shape: {seg_mask.shape}, min/max: {seg_mask.min()}/{seg_mask.max()}")
    _Image.fromarray(seg_mask).save(os.path.join(out_dir, 'replay_seg_mask.png'))
    # dump links for debugging
    try:
        links_info = [{'id': int(l.get_id()), 'name': str(l.get_name())} for l in env.object.get_links()]
        with open(os.path.join(out_dir, 'replay_links.json'), 'w') as f:
            json.dump({'links': links_info, 'movable_link_ids': [int(x) for x in link_ids]}, f)
        print(f"[DEBUG] Object links: {links_info}")
        print(f"[DEBUG] Movable link IDs: {[int(x) for x in link_ids]}")
        # check object pose
        obj_pose = env.object.get_root_pose()
        print(f"[DEBUG] Object root pose: p={obj_pose.p}, q={obj_pose.q}")
    except Exception:
        pass
except Exception:
    pass
if len(replay_data.get('joint_angles', [])) > 0:
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
    # handle rigid objects with empty qpos
    if len(cur_new_qpos) == 0 and len(cur_qpos) == 0:
        if not invalid_contact:
            still_timesteps += 1
        else:
            still_timesteps = 0
    else:
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

# activate contact checking (skip for shadowhand to avoid early abort on first timestep)
enable_contact_check = True
strict_contact = ('pushing' in primact_type)
if robot_type == 'shadowhand':
    enable_contact_check = False
if enable_contact_check:
    env.start_checking_contact(robot.hand_actor_id, robot.gripper_actor_ids, strict_contact)

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

