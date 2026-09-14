"""Versioned RGB-D input for the Python DD-GNG viewers.

Graph coordinates remain camera-optical metres; no arm FK or world transform
is applied here. The V2 camera default is D435i for the installed robot, while
an explicit D435 is supported without pretending the device identities match.
"""

from dataclasses import dataclass

import numpy as np

from om6dof_perception.camera_model import (
    camera_model_for, color_point_map, select_camera, select_stream_fps,
)
from om6dof_perception.realsense_low_light import (
    configure_color_sensor, configure_depth_sensor,
)


def add_camera_arguments(parser):
    parser.add_argument('--model-version', choices=('v1', 'v2'), default='v1')
    parser.add_argument('--camera-model', choices=('auto', 'D405', 'D435', 'D435i'),
                        default='auto', help='auto: V1 D405; V2 D435i')
    parser.add_argument('--camera-serial', default='',
                        help='optional: required only if multiple matching cameras exist')
    parser.add_argument('--camera-fps', type=int, default=30,
                        help='normal-mode FPS; must be advertised by both streams')


def camera_contract(args):
    model = 'D435i' if args.model_version == 'v2' and args.camera_model == 'auto' else args.camera_model
    return camera_model_for(args.model_version, model)


@dataclass
class CameraFrame:
    color: np.ndarray
    depth: np.ndarray
    depth_scale: float
    intrinsics: object
    points_xyz: object = None


def sample_camera_points(frame, pixel_step, z_min, z_max):
    """Return contiguous float64 XYZ metres for the unchanged DD-GNG C API."""
    if pixel_step <= 0:
        raise ValueError('pixel_step must be positive')
    if frame.points_xyz is not None:
        points = frame.points_xyz[::pixel_step, ::pixel_step].reshape(-1, 3)
    else:
        # Preserve the V1 aligned-depth/pinhole preprocessing.
        height, width = frame.depth.shape
        uu, vv = np.meshgrid(np.arange(0, width, pixel_step),
                             np.arange(0, height, pixel_step))
        zz = frame.depth[::pixel_step, ::pixel_step].astype(np.float64) * frame.depth_scale
        intr = frame.intrinsics
        points = np.stack(((uu - intr.ppx) / intr.fx * zz,
                           (vv - intr.ppy) / intr.fy * zz, zz), axis=-1).reshape(-1, 3)
    valid = np.isfinite(points).all(axis=1) & (points[:, 2] > z_min) & (points[:, 2] < z_max)
    return np.ascontiguousarray(points[valid], dtype=np.float64)


def project_camera_points(points, intrinsics, sdk=None):
    """Project camera XYZ; V2 uses SDK RGB distortion, V1 keeps its pinhole."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    uv = np.full((len(points), 2), -1, dtype=np.int32)
    valid = np.isfinite(points).all(axis=1) & (points[:, 2] > 1e-6)
    if not valid.any():
        return uv
    if sdk is None:
        xyz = points[valid]
        projected = np.column_stack((xyz[:, 0] / xyz[:, 2] * intrinsics.fx + intrinsics.ppx,
                                     xyz[:, 1] / xyz[:, 2] * intrinsics.fy + intrinsics.ppy))
    else:
        projected = np.asarray([sdk.rs2_project_point_to_pixel(intrinsics, point.tolist())
                                for point in points[valid]])
    finite = np.isfinite(projected).all(axis=1)
    # Bound off-screen coordinates before converting to OpenCV integer points.
    indices = np.flatnonzero(valid)[finite]
    uv[indices] = np.clip(projected[finite], -100000, 100000).astype(np.int32)
    return uv


class RealSenseInput:
    """One explicitly selected camera, with native V2 depth-to-RGB registration.

    Constructing this object does not access a device. Only start/read do I/O.
    A failed start closes an opened pipeline; close is safe before start.
    """

    def __init__(self, sdk, args, width=640, height=480, low_light_config=None):
        self.sdk = sdk
        self.contract = camera_contract(args)
        self.serial = args.camera_serial
        self.requested_fps = args.camera_fps
        self.width, self.height = width, height
        self.low_light_config = low_light_config
        self.pipe = None
        self.metadata = {}
        self.configuration_messages = []

    def start(self):
        rs = self.sdk
        context = rs.context()
        devices = list(context.query_devices())
        identities = [{'name': device.get_info(rs.camera_info.name),
                       'serial': device.get_info(rs.camera_info.serial_number)}
                      for device in devices]
        selected = select_camera(identities, self.contract.camera_model, self.serial)
        self.serial = selected['serial']
        device = devices[identities.index(selected)]
        color_rates, depth_rates = set(), set()
        for sensor in device.query_sensors():
            for profile in sensor.get_stream_profiles():
                if not profile.is_video_stream_profile():
                    continue
                video = profile.as_video_stream_profile()
                if (video.width(), video.height()) != (self.width, self.height):
                    continue
                if profile.stream_type() == rs.stream.color and profile.format() == rs.format.bgr8:
                    color_rates.add(profile.fps())
                if profile.stream_type() == rs.stream.depth and profile.format() == rs.format.z16:
                    depth_rates.add(profile.fps())
        low_light = bool(self.low_light_config and self.low_light_config['enabled'])
        fps = select_stream_fps(color_rates, depth_rates, self.requested_fps, low_light)
        config = rs.config()
        config.enable_device(self.serial)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, fps)
        self.pipe = rs.pipeline(context)
        try:
            profile = self.pipe.start(config)
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            color = profile.get_stream(rs.stream.color).as_video_stream_profile()
            depth = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            self.intrinsics = color.get_intrinsics()
            self.extrinsics = depth.get_extrinsics_to(color)
            self.align = rs.align(rs.stream.color) if self.contract.version == 'v1' else None
            self.cloud = rs.pointcloud() if self.contract.version == 'v2' else None
            if self.low_light_config is not None:
                if self.contract.version == 'v2' and not low_light:
                    self.configuration_messages.append('depth emitter: device setting retained')
                else:
                    self.configuration_messages.append(configure_depth_sensor(
                        profile, rs, self.low_light_config))
                self.configuration_messages.append(configure_color_sensor(
                    profile, rs, self.low_light_config))
            intr = self.intrinsics
            self.metadata = {
                'model_version': self.contract.version,
                'camera_model': self.contract.camera_model,
                'camera_name': selected['name'], 'camera_serial': self.serial,
                'frame': self.contract.optical_frame,
                'coordinate_space': 'camera_optical', 'units': 'metres',
                'robot_kinematics_applied': False, 'world_projection': False,
                'fps': fps, 'depth_scale_m': self.depth_scale,
                'point_registration': ('sdk_depth_to_color_pointcloud'
                                       if self.contract.version == 'v2'
                                       else 'legacy_aligned_depth'),
                'color_intrinsics': {
                    'width': intr.width, 'height': intr.height,
                    'fx': intr.fx, 'fy': intr.fy, 'ppx': intr.ppx, 'ppy': intr.ppy,
                    'model': str(intr.model), 'coeffs': list(intr.coeffs),
                },
                'depth_to_color': {
                    'rotation_column_major': list(self.extrinsics.rotation),
                    'translation_m': list(self.extrinsics.translation), 'source': 'device_sdk',
                },
            }
            return self
        except BaseException:
            self.close()
            raise

    def read(self):
        frames = self.pipe.wait_for_frames(timeout_ms=2000)
        if self.align is not None:
            frames = self.align.process(frames)
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth:
            raise RuntimeError('Camera frame is missing color or depth')
        image = np.asanyarray(color.get_data()).copy()
        intr = color.profile.as_video_stream_profile().get_intrinsics()
        if self.contract.version == 'v1':
            return CameraFrame(image, np.asanyarray(depth.get_data()).copy(), self.depth_scale, intr)
        self.cloud.map_to(color)
        points = self.cloud.calculate(depth)
        vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        texture = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
        xyz = color_point_map(vertices, texture, self.extrinsics.rotation,
                              self.extrinsics.translation, *image.shape[:2])
        registered_depth = np.nan_to_num(xyz[:, :, 2], nan=0.0)
        return CameraFrame(image, registered_depth, 1.0, intr, xyz)

    def close(self):
        pipe, self.pipe = self.pipe, None
        if pipe is not None:
            try:
                pipe.stop()
            except RuntimeError:
                pass
