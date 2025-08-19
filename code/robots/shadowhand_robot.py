"""
Shadow Hand robot controller for SAPIEN.

This controller provides a minimal interface compatible with the existing
Panda controller usage in collect_data.py:
  - Attributes: hand_actor_id, gripper_actor_ids, robot
  - Methods: move_to_target_pose(mat44, num_steps), open_gripper(), close_gripper(), wait_n_steps(n)

Implementation notes:
  - We treat the Shadow Hand as a kinematic body whose base pose is moved by
    linearly interpolating the root link pose towards the target pose.
  - Gripper open/close is implemented by driving finger joints towards their
    upper/lower joint limits respectively.
  - This is a minimal working version to integrate into the existing pipeline.
    Further improvements (dexterous pre-shapes, coordinated motions) can be layered on top.
"""

from __future__ import annotations

import numpy as np
import sapien.core as sapien
from sapien.core import Pose
from typing import List


def _mat44_to_pose(mat44: np.ndarray) -> Pose:
    """Convert a 4x4 homogeneous transform to SAPIEN Pose."""
    assert mat44.shape == (4, 4)
    # Extract quaternion from rotation matrix using numpy -> sapien Pose can take quaternion directly
    # We use a simple conversion via singular value decomposition to ensure orthonormality
    r = mat44[:3, :3].astype(np.float32)
    u, _, v = np.linalg.svd(r)
    r_ortho = (u @ v).astype(np.float32)
    # Convert rotation matrix to quaternion (w, x, y, z)
    qw = np.sqrt(max(0.0, 1.0 + r_ortho[0, 0] + r_ortho[1, 1] + r_ortho[2, 2])) / 2.0
    qx = np.sign(r_ortho[2, 1] - r_ortho[1, 2]) * np.sqrt(max(0.0, 1.0 + r_ortho[0, 0] - r_ortho[1, 1] - r_ortho[2, 2])) / 2.0
    qy = np.sign(r_ortho[0, 2] - r_ortho[2, 0]) * np.sqrt(max(0.0, 1.0 - r_ortho[0, 0] + r_ortho[1, 1] - r_ortho[2, 2])) / 2.0
    qz = np.sign(r_ortho[1, 0] - r_ortho[0, 1]) * np.sqrt(max(0.0, 1.0 - r_ortho[0, 0] - r_ortho[1, 1] + r_ortho[2, 2])) / 2.0
    quat = np.array([qw, qx, qy, qz], dtype=np.float32)
    quat /= np.linalg.norm(quat) + 1e-9
    pos = mat44[:3, 3].astype(np.float32)
    return Pose(p=pos.tolist(), q=quat.tolist())


def _slerp_quat(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two quaternions.
    q: (w, x, y, z)
    """
    q0 = q0 / (np.linalg.norm(q0) + 1e-9)
    q1 = q1 / (np.linalg.norm(q1) + 1e-9)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        # Linear fallback for very close quaternions
        res = q0 + t * (q1 - q0)
        return res / (np.linalg.norm(res) + 1e-9)
    theta_0 = np.arccos(dot)
    theta = theta_0 * t
    q2 = (q1 - q0 * dot)
    q2 /= (np.linalg.norm(q2) + 1e-9)
    return q0 * np.cos(theta) + q2 * np.sin(theta)


class ShadowHandRobot(object):
    def __init__(self, env, urdf: str, material: sapien.PhysicalMaterial, open_gripper: bool = False):
        """Load the Shadow Hand URDF and initialize joint drives.

        Args:
            env: Env instance providing scene and stepping/rendering helpers
            urdf: Path to Shadow Hand URDF
            material: SAPIEN physical material
            open_gripper: Whether to open the hand initially
        """
        self.env = env
        self.timestep = env.scene.get_timestep()

        loader = env.scene.create_urdf_loader()
        loader.fix_root_link = True
        self.robot = loader.load(urdf, {"material": material})
        self.robot.name = "robot"

        # Identify palm link (as hand base) and finger links (as gripper links)
        links: List[sapien.Link] = self.robot.get_links()
        self.palm_link = None
        for l in links:
            n = (l.get_name() or "").lower()
            if "palm" in n:
                self.palm_link = l
                break
        if self.palm_link is None:
            # Fallback: choose the first link as base if palm not found
            self.palm_link = links[0]

        self.hand_actor_id = self.palm_link.get_id()

        # Define gripper links as all finger segments (exclude forearm/wrist/palm)
        self.gripper_actor_ids: List[int] = []
        for l in links:
            n = (l.get_name() or "").lower()
            if ("forearm" in n) or ("wrist" in n) or ("palm" in n):
                continue
            self.gripper_actor_ids.append(l.get_id())

        # All joints with DOF > 0 are finger/wrist joints for driving
        self.joints = [j for j in self.robot.get_joints() if j.get_dof() > 0]
        for j in self.joints:
            # Modest stiffness/damping for stable motion
            j.set_drive_property(stiffness=50.0, damping=5.0)

        # Cache joint limits for open/close presets
        self.joint_limits = {}
        for j in self.joints:
            lim = j.get_limits()[0]
            self.joint_limits[j.get_name()] = (float(lim[0]), float(lim[1]))

        if open_gripper:
            self.open_gripper()

    def _set_joint_targets(self, targets: dict):
        """Set drive targets for joints given a mapping name->target."""
        for j in self.joints:
            name = j.get_name()
            if name in targets:
                j.set_drive_target(targets[name])

    def open_gripper(self):
        """Open hand by moving joints towards upper limits (where applicable)."""
        targets = {}
        for name, (lo, hi) in self.joint_limits.items():
            # For most finger joints, larger angle corresponds to opening from closed contact
            targets[name] = hi
        self._set_joint_targets(targets)

    def close_gripper(self):
        """Close hand by moving joints towards lower limits."""
        targets = {}
        for name, (lo, hi) in self.joint_limits.items():
            targets[name] = lo
        self._set_joint_targets(targets)

    def move_to_target_pose(self, target_ee_pose: np.ndarray, num_steps: int) -> None:
        """Move the hand base (root link) towards target base pose by interpolation.

        Args:
            target_ee_pose: (4, 4) transform of hand base in scene/world frame
            num_steps: number of simulation steps for the motion
        """
        assert target_ee_pose.shape == (4, 4)
        # Current pose
        current_pose: Pose = self.robot.get_root_pose()
        cp = np.array(current_pose.p, dtype=np.float32)
        cq = np.array(current_pose.q, dtype=np.float32)
        # Target pose
        tp_pose = _mat44_to_pose(target_ee_pose)
        tp = np.array(tp_pose.p, dtype=np.float32)
        tq = np.array(tp_pose.q, dtype=np.float32)

        for i in range(max(1, num_steps)):
            t = float(i + 1) / float(num_steps)
            p = (1.0 - t) * cp + t * tp
            q = _slerp_quat(cq, tq, t)
            self.robot.set_root_pose(Pose(p=p.tolist(), q=q.tolist()))
            self.env.step()
            self.env.render()

    def wait_n_steps(self, n: int):
        """Let physics settle while keeping current drive targets."""
        for _ in range(n):
            passive = self.robot.compute_passive_force()
            self.robot.set_qf(passive)
            self.env.step()
            self.env.render()


