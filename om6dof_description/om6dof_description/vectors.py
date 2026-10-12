"""Geometry for gravity vectors; lengths are metres and forces are newtons."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LinkMass:
    name: str
    mass: float
    com: tuple


def link_masses(robot):
    """Read every inertial link, including fixed payload and gripper branches."""
    result = []
    for link in robot.links:
        if link.inertial is None:
            continue
        mass = float(link.inertial.mass)
        if not math.isfinite(mass) or mass < 0.0:
            raise ValueError('invalid mass for link %s' % link.name)
        if mass == 0.0:
            continue
        origin = link.inertial.origin
        com = tuple(origin.xyz) if origin and origin.xyz is not None else (0.0, 0.0, 0.0)
        if len(com) != 3 or not all(math.isfinite(value) for value in com):
            raise ValueError('invalid centre of mass for link %s' % link.name)
        # The inertial origin's translation locates the CoM. Its rpy rotates
        # the inertia tensor, not that translation.
        result.append(LinkMass(link.name, mass, com))
    return result


def transform_point(point, translation, quaternion):
    """Rotate a link-local point by an xyzw quaternion, then translate it."""
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12:
        raise ValueError('rotation quaternion has zero norm')
    x, y, z, w = (value / norm for value in quaternion)
    px, py, pz = point
    # q * p * q^-1, using two cross products.
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return (
        px + w * tx + y * tz - z * ty + translation[0],
        py + w * ty + z * tx - x * tz + translation[1],
        pz + w * tz + x * ty - y * tx + translation[2],
    )


def vector_endpoints(link, translation, quaternion, gravity, force_scale, mode):
    """Return an arrow in the reference frame, independent of link orientation."""
    com = transform_point(link.com, translation, quaternion)
    if mode == 'com_offset':
        return tuple(translation), com
    if mode != 'gravity':
        raise ValueError('vector_mode must be gravity or com_offset')
    return com, tuple(com[i] + force_scale * link.mass * gravity[i] for i in range(3))
