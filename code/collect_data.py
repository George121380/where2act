"""
    For panda (two-finger) gripper: pushing, pushing-left, pushing-up, pulling, pulling-left, pulling-up
        50% all parts closed, 50% middle (for each part, 50% prob. closed, 50% prob. middle)
        Simulate until static before starting
"""

import os
import sys
import shutil
import numpy as np
from PIL import Image
from utils import get_global_position_from_camera, save_h5
import cv2
import json
from argparse import ArgumentParser
import sapien

from sapien.core import Pose
from env import Env, ContactError
from camera import Camera
from robots.panda_robot import Robot
from robots.shadowhand_robot import ShadowHandRobot

parser = ArgumentParser()
parser.add_argument('shape_id', type=str)
parser.add_argument('category', type=str)
parser.add_argument('cnt_id', type=int)
parser.add_argument('primact_type', type=str)
parser.add_argument('--out_dir', type=str, default='/media/george/Projects/Research/2026-CVPR-BiDexHand/third_party/where2act/code/results')
parser.add_argument('--trial_id', type=int, default=0, help='trial id')
parser.add_argument('--random_seed', type=int, default=None)
parser.add_argument('--no_gui', action='store_true', default=True, help='no_gui [default: False]')
parser.add_argument('--robot_type', type=str, default='panda', choices=['panda', 'shadowhand'], help='robot type to use')
parser.add_argument('--shadowhand_urdf', type=str, default='./robots/shadowhand/shadowhand_ign_shadow_hand_fixed.urdf', help='URDF path for Shadow Hand')
parser.add_argument('--debug_dump_hand_qpos', action='store_true', default=False, help='dump hand qpos and snapshots for debugging')
parser.add_argument('--object_mode', type=str, default='dataset', choices=['dataset', 'asset', 'simple'], help='where to load the object from')
parser.add_argument('--sapien_asset_id', type=int, default=None, help='SAPIEN PartNet-Mobility asset id (when object_mode=asset)')
parser.add_argument('--sapien_asset_token', type=str, default=None, help='SAPIEN asset access token (fallback to env SAPIEN_ASSET_TOKEN)')
parser.add_argument('--object_urdf_override', type=str, default=None, help='override URDF path when object_mode=simple')
parser.add_argument('--camera_dist', type=float, default=None, help='override camera distance (meters)')
parser.add_argument('--camera_fov_deg', type=float, default=None, help='override camera vertical fov (degrees)')
args = parser.parse_args()

shape_id = args.shape_id
trial_id = args.trial_id
primact_type = args.primact_type
if args.no_gui:
    out_dir = os.path.join(args.out_dir, '%s_%s_%d_%s_%d' % (shape_id, args.category, args.cnt_id, primact_type, trial_id))
else:
    out_dir = os.path.join('results', '%s_%s_%d_%s_%d' % (shape_id, args.category, args.cnt_id, primact_type, trial_id))
if os.path.exists(out_dir):
    shutil.rmtree(out_dir)
os.mkdir(out_dir)
flog = open(os.path.join(out_dir, 'log.txt'), 'w')
out_info = dict()

# set random seed
if args.random_seed is not None:
    np.random.seed(args.random_seed)
    out_info['random_seed'] = args.random_seed

# setup env
env = Env(flog=flog, show_gui=(not args.no_gui))

# For simple objects, position them in a more visible location
if getattr(args, 'object_mode', 'dataset') == 'simple':
    # Set object position offset to place object in camera view
    env.object_position_offset = 0.0  # Keep at origin, but we'll adjust camera

# setup camera
# For simple objects, default to a closer camera to make small boxes visible
default_camera_dist = 5.0
if getattr(args, 'object_mode', 'dataset') == 'simple':
    default_camera_dist = 1.5
cam_dist = args.camera_dist if args.camera_dist is not None else default_camera_dist
cam_fov_deg = args.camera_fov_deg if args.camera_fov_deg is not None else 35
cam = Camera(env, random_position=True, dist=cam_dist, fov=cam_fov_deg)
out_info['camera_metadata'] = cam.get_metadata_json()
if not args.no_gui:
    env.set_controller_camera_pose(cam.pos[0], cam.pos[1], cam.pos[2], np.pi+cam.theta, -cam.phi)

# load shape
object_material = env.get_material(4, 4, 0.01)

# decide URDF source
object_mode = getattr(args, 'object_mode', 'dataset')
out_info['object_mode'] = object_mode
if object_mode == 'dataset':
    object_urdf_fn = '../data/where2act_original_sapien_dataset/%s/mobility_vhacd.urdf' % shape_id
elif object_mode == 'asset':
    asset_id = args.sapien_asset_id
    token = args.sapien_asset_token or os.environ.get('SAPIEN_ASSET_TOKEN')
    if asset_id is None or token is None:
        flog.write('ERROR: asset mode requires --sapien_asset_id and a valid token (CLI or env SAPIEN_ASSET_TOKEN)\n')
        flog.close(); env.close(); sys.exit(2)
    try:
        object_urdf_fn = sapien.asset.download_partnet_mobility(asset_id, token)
        out_info['sapien_asset_id'] = int(asset_id)
        out_info['asset_urdf'] = object_urdf_fn
    except Exception as e:
        flog.write(f'ERROR: failed to download/load SAPIEN asset {asset_id}: {e}\n')
        flog.close(); env.close(); sys.exit(2)
else:
    # simple mode: use override URDF if provided; otherwise use built-in rectangular box
    if args.object_urdf_override is not None:
        object_urdf_fn = args.object_urdf_override
        out_info['object_urdf_override'] = object_urdf_fn
    else:
        object_urdf_fn = './objects/simple/box_rect_060408.urdf'
        out_info['object_urdf_builtin'] = object_urdf_fn

flog.write('object_urdf_fn: %s\n' % object_urdf_fn)

# decide initial state (for articulated assets)
state = 'random-closed-middle'
if np.random.random() < 0.5:
    state = 'closed'
flog.write('Object State: %s\n' % state)
out_info['object_state'] = state

# load into scene
joint_angles = env.load_object(object_urdf_fn, object_material, state=state)
out_info['joint_angles'] = joint_angles
out_info['joint_angles_lower'] = env.joint_angles_lower
out_info['joint_angles_upper'] = env.joint_angles_upper
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
    # handle articulated vs rigid objects (no joints => empty qpos)
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
    flog.write('Object Not Still!\n')
    flog.close()
    env.close()
    exit(1)

### use the GT vision
rgb, depth = cam.get_observation()
Image.fromarray((rgb*255).astype(np.uint8)).save(os.path.join(out_dir, 'rgb.png'))

cam_XYZA_id1, cam_XYZA_id2, cam_XYZA_pts = cam.compute_camera_XYZA(depth)
cam_XYZA = cam.compute_XYZA_matrix(cam_XYZA_id1, cam_XYZA_id2, cam_XYZA_pts, depth.shape[0], depth.shape[1])
save_h5(os.path.join(out_dir, 'cam_XYZA.h5'), \
        [(cam_XYZA_id1.astype(np.uint64), 'id1', 'uint64'), \
         (cam_XYZA_id2.astype(np.uint64), 'id2', 'uint64'), \
         (cam_XYZA_pts.astype(np.float32), 'pc', 'float32'), \
        ])

gt_nor = cam.get_normal_map()
Image.fromarray(((gt_nor+1)/2*255).astype(np.uint8)).save(os.path.join(out_dir, 'gt_nor.png'))

object_link_ids = env.movable_link_ids
gt_movable_link_mask = cam.get_movable_link_mask(object_link_ids)
Image.fromarray((gt_movable_link_mask>0).astype(np.uint8)*255).save(os.path.join(out_dir, 'interaction_mask.png'))

# sample a pixel to interact
xs, ys = np.where(gt_movable_link_mask>0)
if len(xs) == 0:
    flog.write('No Movable Pixel! Quit!\n')
    flog.close()
    env.close()
    exit(1)
idx = np.random.randint(len(xs))
x, y = xs[idx], ys[idx]
out_info['pixel_locs'] = [int(x), int(y)]
env.set_target_object_part_actor_id(object_link_ids[gt_movable_link_mask[x, y]-1])
out_info['target_object_part_actor_id'] = env.target_object_part_actor_id
out_info['target_object_part_joint_id'] = env.target_object_part_joint_id

# get pixel 3D pulling direction (cam/world)
direction_cam = gt_nor[x, y, :3]
direction_cam /= np.linalg.norm(direction_cam)
out_info['direction_camera'] = direction_cam.tolist()
flog.write('Direction Camera: %f %f %f\n' % (direction_cam[0], direction_cam[1], direction_cam[2]))
direction_world = cam.get_metadata()['mat44'][:3, :3] @ direction_cam
out_info['direction_world'] = direction_world.tolist()
flog.write('Direction World: %f %f %f\n' % (direction_world[0], direction_world[1], direction_world[2]))
flog.write('mat44: %s\n' % str(cam.get_metadata()['mat44']))

# sample a random direction in the hemisphere (cam/world)
action_direction_cam = np.random.randn(3).astype(np.float32)
action_direction_cam /= np.linalg.norm(action_direction_cam)
if action_direction_cam @ direction_cam > 0:
    action_direction_cam = -action_direction_cam
out_info['gripper_direction_camera'] = action_direction_cam.tolist()
action_direction_world = cam.get_metadata()['mat44'][:3, :3] @ action_direction_cam
out_info['gripper_direction_world'] = action_direction_world.tolist()

# get pixel 3D position (cam/world)
position_cam = cam_XYZA[x, y, :3]
out_info['position_cam'] = position_cam.tolist()
position_cam_xyz1 = np.ones((4), dtype=np.float32)
position_cam_xyz1[:3] = position_cam
position_world_xyz1 = cam.get_metadata()['mat44'] @ position_cam_xyz1
position_world = position_world_xyz1[:3]
out_info['position_world'] = position_world.tolist()

# compute final pose
up = np.array(action_direction_world, dtype=np.float32)
forward = np.random.randn(3).astype(np.float32)
while abs(up @ forward) > 0.99:
    forward = np.random.randn(3).astype(np.float32)
left = np.cross(up, forward)
left /= np.linalg.norm(left)
forward = np.cross(left, up)
forward /= np.linalg.norm(forward)
out_info['gripper_forward_direction_world'] = forward.tolist()
forward_cam = np.linalg.inv(cam.get_metadata()['mat44'][:3, :3]) @ forward
out_info['gripper_forward_direction_camera'] = forward_cam.tolist()
rotmat = np.eye(4).astype(np.float32)
rotmat[:3, 0] = forward
rotmat[:3, 1] = left
rotmat[:3, 2] = up

final_dist = 0.1
if primact_type == 'pushing-left' or primact_type == 'pushing-up':
    final_dist = 0.11

final_rotmat = np.array(rotmat, dtype=np.float32)
final_rotmat[:3, 3] = position_world - action_direction_world * final_dist
final_pose = Pose().from_transformation_matrix(final_rotmat)
out_info['target_rotmat_world'] = final_rotmat.tolist()

start_rotmat = np.array(rotmat, dtype=np.float32)
start_rotmat[:3, 3] = position_world - action_direction_world * 0.15
start_pose = Pose().from_transformation_matrix(start_rotmat)
out_info['start_rotmat_world'] = start_rotmat.tolist()

action_direction = None
if 'left' in primact_type:
    action_direction = forward
elif 'up' in primact_type:
    action_direction = left

if action_direction is not None:
    end_rotmat = np.array(rotmat, dtype=np.float32)
    end_rotmat[:3, 3] = position_world - action_direction_world * final_dist + action_direction * 0.05
    out_info['end_rotmat_world'] = end_rotmat.tolist()


### viz the EE gripper position
# setup robot
robot_material = env.get_material(4, 4, 0.01)
if args.robot_type == 'panda':
    robot_urdf_fn = './robots/panda_gripper.urdf'
    robot = Robot(env, robot_urdf_fn, robot_material, open_gripper=('pulling' in primact_type))
else:
    robot_urdf_fn = args.shadowhand_urdf
    robot = ShadowHandRobot(env, robot_urdf_fn, robot_material, open_gripper=('pulling' in primact_type))

# record robot metadata
out_info['robot_type'] = args.robot_type
out_info['robot_urdf'] = robot_urdf_fn

# helper for debug dumping
def _debug_dump(stage_name: str):
    if not args.debug_dump_hand_qpos:
        return
    try:
        # save articulation qpos
        qpos = robot.robot.get_qpos()
        with open(os.path.join(out_dir, f'hand_qpos_{stage_name}.json'), 'w') as f:
            json.dump({'stage': stage_name, 'qpos': [float(x) for x in qpos]}, f)
        # save snapshot
        rgb_dbg, _ = cam.get_observation()
        Image.fromarray((rgb_dbg*255).astype(np.uint8)).save(os.path.join(out_dir, f'hand_{stage_name}.png'))
    except Exception as e:
        flog.write(f'DEBUG_DUMP_ERROR@{stage_name}: {e}\n')

# dump joint meta for shadowhand
if args.robot_type == 'shadowhand':
    try:
        joints_meta = []
        for j in robot.robot.get_joints():
            meta = {
                'name': j.get_name(),
                'dof': int(j.get_dof()),
            }
            try:
                lim = j.get_limits()
                if lim is not None and len(lim) > 0:
                    lo, hi = float(lim[0][0]), float(lim[0][1])
                    meta['limit'] = [lo, hi]
            except Exception:
                pass
            joints_meta.append(meta)
        with open(os.path.join(out_dir, 'hand_joint_info.json'), 'w') as f:
            json.dump({'joints': joints_meta, 'initial_qpos': [float(x) for x in robot.robot.get_qpos()]}, f)
    except Exception as e:
        flog.write(f'WARN: joint meta dump failed: {e}\n')

# pre-op open/close away from object (before enabling contact checking)
robot.robot.set_root_pose(start_pose)
env.render()
if args.robot_type == 'shadowhand':
    # ensure joints respond without contact interruption
    robot.open_gripper(); robot.wait_n_steps(1000); _debug_dump('preop_after_open')
    robot.close_gripper(); robot.wait_n_steps(1000); _debug_dump('preop_after_close')

# visualize final pose for approach
robot.robot.set_root_pose(final_pose)
env.render()
rgb_final_pose, _ = cam.get_observation()
Image.fromarray((rgb_final_pose*255).astype(np.uint8)).save(os.path.join(out_dir, 'viz_target_pose.png'))
_debug_dump('at_final_pose')

# move back to start for normal pipeline
robot.robot.set_root_pose(start_pose)
env.render()

# activate contact checking
env.start_checking_contact(robot.hand_actor_id, robot.gripper_actor_ids, 'pushing' in primact_type)

### main steps
out_info['start_target_part_qpos'] = env.get_target_part_qpos()

target_link_mat44 = env.get_target_part_pose().to_transformation_matrix()
position_local_xyz1 = np.linalg.inv(target_link_mat44) @ position_world_xyz1

success = True
try:
    if 'pushing' in primact_type:
        robot.close_gripper(); robot.wait_n_steps(500); _debug_dump('after_close_for_push')
    elif 'pulling' in primact_type:
        robot.open_gripper(); robot.wait_n_steps(500); _debug_dump('after_open_for_pull')

    # approach
    robot.move_to_target_pose(final_rotmat, 2000)
    robot.wait_n_steps(2000)

    if 'pulling' in primact_type:
        robot.close_gripper(); robot.wait_n_steps(500); _debug_dump('after_close_for_pull')
        robot.wait_n_steps(2000)
    
    if 'left' in primact_type or 'up' in primact_type:
        robot.move_to_target_pose(end_rotmat, 2000)
        robot.wait_n_steps(2000)
    
    if primact_type == 'pulling':
        robot.move_to_target_pose(start_rotmat, 2000)
        robot.wait_n_steps(2000)

except ContactError:
    success = False

target_link_mat44 = env.get_target_part_pose().to_transformation_matrix()
position_world_xyz1_end = target_link_mat44 @ position_local_xyz1
flog.write('touch_position_world_xyz_start: %s\n' % str(position_world_xyz1))
flog.write('touch_position_world_xyz_end: %s\n' % str(position_world_xyz1_end))
out_info['touch_position_world_xyz_start'] = position_world_xyz1[:3].tolist()
out_info['touch_position_world_xyz_end'] = position_world_xyz1_end[:3].tolist()

if success:
    out_info['result'] = 'VALID'
    out_info['final_target_part_qpos'] = env.get_target_part_qpos()
else:
    out_info['result'] = 'CONTACT_ERROR'

# save results
with open(os.path.join(out_dir, 'result.json'), 'w') as fout:
    json.dump(out_info, fout)

#close the file
flog.close()

if args.no_gui:
    # close env
    env.close()
else:
    if success:
        print('[Successful Interaction] Done. Ctrl-C to quit.')
        ### wait forever
        robot.wait_n_steps(100000000000)
    else:
        print('[Unsuccessful Interaction] invalid gripper-object contact.')
        # close env
        env.close()

