"""
    Shadow Hand Robot
        Support shadow_hand_right.urdf, shadow_hand_left.urdf
        Provides control interface for Shadow Hand dexterous manipulation
"""

from __future__ import division
import sapien.core as sapien
from sapien.core import Pose, PxrMaterial, SceneConfig
from transforms3d.quaternions import axangle2quat, qmult
import numpy as np
from utils import pose2exp_coordinate, adjoint_matrix


class ShadowHandRobot(object):
    def __init__(self, env, urdf, material, hand_type='right'):
        """
        Initialize Shadow Hand robot
        
        Args:
            env: SAPIEN environment
            urdf: Path to Shadow Hand URDF file
            material: SAPIEN material for the robot
            hand_type: 'right' or 'left' hand configuration
        """
        self.env = env
        self.timestep = env.scene.get_timestep()
        self.hand_type = hand_type

        # Load robot
        loader = env.scene.create_urdf_loader()
        loader.fix_root_link = True
        self.robot = loader.load(urdf, {"material": material})
        self.robot.name = "shadow_hand"

        # Get all joints and links
        self.all_joints = self.robot.get_joints()
        self.all_links = self.robot.get_links()
        
        # Shadow Hand joint names mapping
        self.joint_names = self._get_joint_names()
        self.finger_names = ['thumb', 'first_finger', 'middle_finger', 'ring_finger', 'little_finger']
        
        # Get finger joints organized by finger
        self.finger_joints = self._organize_finger_joints()
        
        # Get fingertip links for contact detection
        self.fingertip_links = self._get_fingertip_links()
        self.fingertip_actor_ids = [link.get_id() for link in self.fingertip_links]
        
        # Get palm link
        self.palm_link = self._get_palm_link()
        self.palm_actor_id = self.palm_link.get_id() if self.palm_link else None
        
        # Set joint drive properties
        self._set_joint_properties()
        
        # Initialize to neutral pose
        self._set_neutral_pose()

    def _get_joint_names(self):
        """Get Shadow Hand joint names"""
        if self.hand_type == 'right':
            return {
                # Wrist joints
                'WRJ2': 'wrist_flexion',
                'WRJ1': 'wrist_abduction',
                
                # Thumb joints
                'THJ5': 'thumb_flexion',
                'THJ4': 'thumb_abduction', 
                'THJ3': 'thumb_middle',
                'THJ2': 'thumb_proximal',
                'THJ1': 'thumb_distal',
                
                # First finger (index) joints
                'FFJ4': 'first_finger_abduction',
                'FFJ3': 'first_finger_flexion',
                'FFJ2': 'first_finger_middle', 
                'FFJ1': 'first_finger_distal',
                
                # Middle finger joints
                'MFJ4': 'middle_finger_abduction',
                'MFJ3': 'middle_finger_flexion',
                'MFJ2': 'middle_finger_middle',
                'MFJ1': 'middle_finger_distal',
                
                # Ring finger joints
                'RFJ4': 'ring_finger_abduction',
                'RFJ3': 'ring_finger_flexion',
                'RFJ2': 'ring_finger_middle',
                'RFJ1': 'ring_finger_distal',
                
                # Little finger joints
                'LFJ5': 'little_finger_abduction',
                'LFJ4': 'little_finger_flexion_2',
                'LFJ3': 'little_finger_flexion',
                'LFJ2': 'little_finger_middle',
                'LFJ1': 'little_finger_distal'
            }
        else:
            # Left hand has similar joint names
            return self._get_joint_names()  # For now, use same mapping

    def _organize_finger_joints(self):
        """Organize joints by finger for easier control"""
        finger_joints = {finger: [] for finger in self.finger_names}
        
        for joint in self.all_joints:
            joint_name = joint.get_name()
            
            if joint_name.startswith('TH'):  # Thumb
                finger_joints['thumb'].append(joint)
            elif joint_name.startswith('FF'):  # First finger (index)
                finger_joints['first_finger'].append(joint)
            elif joint_name.startswith('MF'):  # Middle finger
                finger_joints['middle_finger'].append(joint)
            elif joint_name.startswith('RF'):  # Ring finger
                finger_joints['ring_finger'].append(joint)
            elif joint_name.startswith('LF'):  # Little finger
                finger_joints['little_finger'].append(joint)
                
        return finger_joints

    def _get_fingertip_links(self):
        """Get fingertip links for contact detection"""
        fingertip_links = []
        fingertip_suffixes = ['thdistal', 'ffdistal', 'mfdistal', 'rfdistal', 'lfdistal']
        
        for link in self.all_links:
            link_name = link.get_name().lower()
            if any(suffix in link_name for suffix in fingertip_suffixes):
                fingertip_links.append(link)
                
        return fingertip_links

    def _get_palm_link(self):
        """Get palm link"""
        for link in self.all_links:
            if 'palm' in link.get_name().lower():
                return link
        return None

    def _set_joint_properties(self):
        """Set joint drive properties for Shadow Hand"""
        for joint in self.all_joints:
            if joint.get_dof() == 1:
                # Set appropriate stiffness and damping for different joint types
                joint_name = joint.get_name()
                
                if joint_name.startswith('WR'):  # Wrist joints
                    joint.set_drive_property(stiffness=1000, damping=100)
                elif joint_name.endswith('J1'):  # Distal joints
                    joint.set_drive_property(stiffness=200, damping=20)
                elif joint_name.endswith('J2'):  # Middle joints
                    joint.set_drive_property(stiffness=300, damping=30)
                elif joint_name.endswith('J3'):  # Proximal joints
                    joint.set_drive_property(stiffness=400, damping=40)
                elif joint_name.endswith('J4') or joint_name.endswith('J5'):  # Abduction joints
                    joint.set_drive_property(stiffness=300, damping=30)
                else:
                    joint.set_drive_property(stiffness=200, damping=20)

    def _set_neutral_pose(self):
        """Set Shadow Hand to neutral pose"""
        joint_angles = []
        for joint in self.all_joints:
            if joint.get_dof() == 1:
                # Set to middle of joint range
                limits = joint.get_limits()
                if len(limits) > 0:
                    lower, upper = limits[0]
                    middle = (lower + upper) / 2.0
                    joint_angles.append(middle)
                else:
                    joint_angles.append(0.0)
        
        if joint_angles:
            self.robot.set_qpos(joint_angles)

    def get_joint_positions(self):
        """Get current joint positions"""
        return self.robot.get_qpos()

    def set_joint_positions(self, positions):
        """Set joint positions"""
        self.robot.set_qpos(positions)

    def get_joint_velocities(self):
        """Get current joint velocities"""
        return self.robot.get_qvel()

    def set_joint_velocities(self, velocities):
        """Set joint velocities"""
        self.robot.set_qvel(velocities)

    def set_joint_targets(self, targets):
        """Set joint position targets for PD control"""
        active_joints = [j for j in self.all_joints if j.get_dof() == 1]
        for i, joint in enumerate(active_joints):
            if i < len(targets):
                joint.set_drive_target(targets[i])

    def close_hand(self, strength=1.0):
        """Close the hand with specified strength (0.0 to 1.0)"""
        targets = []
        for joint in self.all_joints:
            if joint.get_dof() == 1:
                limits = joint.get_limits()
                if len(limits) > 0:
                    lower, upper = limits[0]
                    # For closing, move towards upper limit (flexion)
                    target = lower + strength * (upper - lower)
                    targets.append(target)
                else:
                    targets.append(0.0)
        
        self.set_joint_targets(targets)

    def open_hand(self):
        """Open the hand to neutral position"""
        targets = []
        for joint in self.all_joints:
            if joint.get_dof() == 1:
                limits = joint.get_limits()
                if len(limits) > 0:
                    lower, upper = limits[0]
                    # For opening, move towards lower limit (extension)
                    target = lower
                    targets.append(target)
                else:
                    targets.append(0.0)
        
        self.set_joint_targets(targets)

    def set_finger_position(self, finger_name, positions):
        """Set position for a specific finger
        
        Args:
            finger_name: Name of finger ('thumb', 'first_finger', etc.)
            positions: List of joint positions for the finger
        """
        if finger_name not in self.finger_joints:
            return
            
        finger_joints = self.finger_joints[finger_name]
        for i, joint in enumerate(finger_joints):
            if i < len(positions) and joint.get_dof() == 1:
                joint.set_drive_target(positions[i])

    def grasp_object(self, grasp_strength=0.7):
        """Perform a simple grasping motion"""
        # Close fingers gradually
        self.close_hand(grasp_strength)

    def release_object(self):
        """Release grasped object by opening hand"""
        self.open_hand()

    def get_fingertip_poses(self):
        """Get poses of all fingertips"""
        poses = {}
        fingertip_names = ['thumb', 'first_finger', 'middle_finger', 'ring_finger', 'little_finger']
        
        for i, link in enumerate(self.fingertip_links):
            if i < len(fingertip_names):
                poses[fingertip_names[i]] = link.get_pose()
                
        return poses

    def get_palm_pose(self):
        """Get palm pose"""
        if self.palm_link:
            return self.palm_link.get_pose()
        return None

    def wait_n_steps(self, n: int):
        """Wait for n simulation steps"""
        for i in range(n):
            passive_force = self.robot.compute_passive_force()
            self.robot.set_qf(passive_force)
            self.env.step()
            self.env.render()
        self.robot.set_qf([0] * self.robot.dof)

    def move_to_pose(self, target_pose: np.ndarray, num_steps: int):
        """Move the hand to a target pose (simplified version)
        
        Args:
            target_pose: 4x4 transformation matrix for target pose
            num_steps: Number of steps to reach target
        """
        # For Shadow Hand, this would typically involve inverse kinematics
        # For now, we implement a simple version that moves the base
        if hasattr(self.robot, 'set_root_pose'):
            pose = Pose.from_transformation_matrix(target_pose)
            self.robot.set_root_pose(pose)
        
        # Use fewer steps to avoid collision issues during approach
        self.wait_n_steps(min(num_steps, 500))

    def get_contact_forces(self):
        """Get contact forces on fingertips"""
        contacts = self.env.scene.get_contacts()
        fingertip_forces = {name: np.zeros(3) for name in self.finger_names}
        
        for contact in contacts:
            actor1_id = contact.actor1.get_id()
            actor2_id = contact.actor2.get_id()
            
            # Check if contact involves fingertips
            for i, fingertip_id in enumerate(self.fingertip_actor_ids):
                if actor1_id == fingertip_id or actor2_id == fingertip_id:
                    # Sum up contact forces
                    for point in contact.points:
                        if i < len(self.finger_names):
                            fingertip_forces[self.finger_names[i]] += point.impulse
        
        return fingertip_forces

    def check_object_contact(self, object_actor_ids):
        """Check if hand is in contact with specified objects
        
        Args:
            object_actor_ids: List of object actor IDs to check contact with
            
        Returns:
            bool: True if any part of hand is in contact with objects
        """
        contacts = self.env.scene.get_contacts()
        hand_actor_ids = self.fingertip_actor_ids + ([self.palm_actor_id] if self.palm_actor_id else [])
        
        for contact in contacts:
            actor1_id = contact.actor1.get_id()
            actor2_id = contact.actor2.get_id()
            
            # Check if contact is between hand and target objects
            if ((actor1_id in hand_actor_ids and actor2_id in object_actor_ids) or
                (actor2_id in hand_actor_ids and actor1_id in object_actor_ids)):
                
                # Check if contact has significant force
                for point in contact.points:
                    if np.linalg.norm(point.impulse) > 1e-4:
                        return True
        
        return False

    def get_hand_state(self):
        """Get complete hand state including joint positions and velocities"""
        # Convert numpy arrays to lists for JSON serialization
        joint_positions = self.get_joint_positions()
        joint_velocities = self.get_joint_velocities()
        fingertip_poses = self.get_fingertip_poses()
        palm_pose = self.get_palm_pose()
        
        # Convert poses to serializable format
        fingertip_poses_serializable = {}
        for finger, pose in fingertip_poses.items():
            if pose is not None:
                try:
                    fingertip_poses_serializable[finger] = {
                        'position': pose.p.tolist(),
                        'quaternion': [pose.q.w, pose.q.x, pose.q.y, pose.q.z]
                    }
                except AttributeError:
                    # Handle case where pose might be in different format
                    fingertip_poses_serializable[finger] = str(pose)
        
        palm_pose_serializable = None
        if palm_pose is not None:
            try:
                palm_pose_serializable = {
                    'position': palm_pose.p.tolist(),
                    'quaternion': [palm_pose.q.w, palm_pose.q.x, palm_pose.q.y, palm_pose.q.z]
                }
            except AttributeError:
                # Handle case where pose might be in different format
                palm_pose_serializable = str(palm_pose)
        
        return {
            'joint_positions': joint_positions.tolist() if hasattr(joint_positions, 'tolist') else joint_positions,
            'joint_velocities': joint_velocities.tolist() if hasattr(joint_velocities, 'tolist') else joint_velocities,
            'fingertip_poses': fingertip_poses_serializable,
            'palm_pose': palm_pose_serializable
        }
