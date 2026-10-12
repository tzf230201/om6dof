"""Geometric Jacobian at a URDF link origin, with linear rows before angular rows."""

import math

import numpy as np


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('joint axis must be finite and nonzero')
    x, y, z = axis / norm
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def origin_transform(origin):
    transform = np.eye(4)
    if origin is not None:
        xyz = origin.xyz if origin.xyz is not None else [0.0, 0.0, 0.0]
        roll, pitch, yaw = origin.rpy if origin.rpy is not None else [0.0, 0.0, 0.0]
        transform[:3, :3] = (rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch)
                             @ rotation([1, 0, 0], roll))
        transform[:3, 3] = xyz
    return transform


class GeometricJacobian:
    def __init__(self, robot, tip_link):
        if tip_link not in robot.link_map:
            raise ValueError('tip_link %s is absent from robot_description' % tip_link)
        self.root = robot.get_root()
        self.tip = tip_link
        parents = {joint.child: joint for joint in robot.joints}
        chain = []
        current = tip_link
        while current != self.root:
            if current not in parents:
                raise ValueError('tip link is disconnected from the URDF root')
            joint = parents[current]
            if joint.type not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                raise ValueError('unsupported joint type in TCP chain: ' + joint.type)
            if joint.mimic is not None:
                raise ValueError('mimic joints in the TCP chain are unsupported: ' + joint.name)
            chain.append(joint)
            current = joint.parent
        self.chain = list(reversed(chain))
        self.joints = [joint for joint in self.chain if joint.type != 'fixed']
        self.names = [joint.name for joint in self.joints]
        if not self.joints:
            raise ValueError('TCP chain contains no movable joints')
        self.origins = {joint.name: origin_transform(joint.origin) for joint in self.chain}

    def evaluate(self, positions, root_transform=None):
        """Return TCP pose and 6xn Jacobian in the root_transform target frame."""
        transform = np.eye(4) if root_transform is None else np.array(root_transform, dtype=float)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError('root_transform must be a finite 4x4 matrix')
        axes, pivots = [], []
        for joint in self.chain:
            transform = transform @ self.origins[joint.name]
            if joint.type == 'fixed':
                continue
            if joint.name not in positions or not math.isfinite(positions[joint.name]):
                raise ValueError('missing finite measured position for ' + joint.name)
            axis = np.array(joint.axis if joint.axis is not None else [1.0, 0.0, 0.0])
            axis = axis / np.linalg.norm(axis)
            if not np.all(np.isfinite(axis)):
                raise ValueError('invalid axis for ' + joint.name)
            axes.append(transform[:3, :3] @ axis)
            pivots.append(transform[:3, 3].copy())
            motion = np.eye(4)
            if joint.type == 'prismatic':
                motion[:3, 3] = axis * positions[joint.name]
            else:
                motion[:3, :3] = rotation(axis, positions[joint.name])
            transform = transform @ motion
        matrix = np.zeros((6, len(self.joints)))
        for index, joint in enumerate(self.joints):
            if joint.type == 'prismatic':
                matrix[:3, index] = axes[index]
            else:
                matrix[:3, index] = np.cross(axes[index], transform[:3, 3] - pivots[index])
                matrix[3:, index] = axes[index]
        return transform, matrix


def ellipsoid_segments(center, linear_jacobian, scale, samples=48):
    """Three great circles of the SVD ellipsoid, including collapsed singular axes."""
    axes, singular_values, _ = np.linalg.svd(linear_jacobian, full_matrices=True)
    radii = np.zeros(3)
    radii[:len(singular_values)] = singular_values * scale
    points = []
    for first, second in ((0, 1), (1, 2), (2, 0)):
        for step in range(samples):
            for angle in (2 * math.pi * step / samples, 2 * math.pi * (step + 1) / samples):
                point = (center + axes[:, first] * radii[first] * math.cos(angle)
                         + axes[:, second] * radii[second] * math.sin(angle))
                points.append(point)
    return points, singular_values
