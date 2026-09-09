# Licensing and provenance

Original OM6DOF contributions use Apache License 2.0, consistent with the
OM6DOF package manifests. The full text is in [LICENSE](../LICENSE).
Third-party material retains its own terms; this license does not replace
another copyright holder's terms or supply missing permissions.

## Verified OpenManipulator Friends material

Compared against these complete upstream trees:

- [ROBOTIS OpenManipulator Friends, f029a07a](https://github.com/ROBOTIS-GIT/open_manipulator_friends/tree/f029a07a76597f3f30b52e90a8cf2571b2cf835d).
- [ROS 2 reference, 5e1531c5](https://github.com/tzf230201/open_manipulator_friends_ros2/tree/5e1531c5b4031d49a61f4b11238eb03bb35d0a9e).

Both contain the same Apache-2.0 LICENSE. Neither inspected tree includes a
NOTICE file. The notices added here record OM6DOF's attribution and changes;
they are not presented as upstream-authored notices.

The nine arm/finger STL files and `urdf/materials.xacro` are byte-identical
to upstream. Their paths and Git blob hashes are recorded in
[om6dof_description/NOTICE](../om6dof_description/NOTICE).
The adapted `urdf/om6dof.urdf.xacro` carries a prominent modification notice
covering the updated inertial parameters, joint limits, ROS integration,
frames, pedestal, gripper configuration, and RealSense wrist payload.
Original description-package author credits are retained in NOTICE and
`package.xml`. No upstream copyright year has been invented for files that
did not carry one.

`om6dof_description/CMakeLists.txt` installs the package's LICENSE and NOTICE
into `share/om6dof_description`, so they accompany an installed description
package as well as the source checkout.

## Verified ALOHA wrist bracket

The maintainer identified [tonyzhaozh/aloha](https://github.com/tonyzhaozh/aloha)
as the bracket source. In the complete inspected Git tree
`06369f03cd8e0a47e16d3a90167853fd33af7557`, the upstream file
`aloha2/Aloha cam wrist mount v13.stl` and local
`om6dof_description/meshes/aloha_cam_wrist_mount_v13.stl` have the same
Git blob SHA-1: `a1fedb360dc7380454314499873042521f6093e7`.
The mesh is unchanged; only the filename was normalized.

Upstream uses the [MIT License](https://github.com/tonyzhaozh/aloha/blob/main/LICENSE),
Copyright (c) 2023 Tony Z. Zhao. No separate bracket license or NOTICE was
found in that tree. The complete license and copyright notice are preserved in
[meshes/LICENSE-ALOHA.txt](../om6dof_description/meshes/LICENSE-ALOHA.txt);
the existing mesh-directory installation rule includes this file in installed
packages. Keep it with redistributed copies of the bracket. This asset retains
MIT terms, rather than being relabeled Apache-2.0.

This resolves the standalone bracket's provenance, not the RealSense camera
model or the assembly modifications in `d405_wrist_cam.stl`.

## Redistribution

For Apache-covered upstream files and derivatives, follow
[Apache-2.0 section 4](https://www.apache.org/licenses/LICENSE-2.0#redistribution):

- Supply the complete license text to recipients, including binary releases.
- Keep relevant copyright, patent, trademark, and attribution notices.
- Put prominent change notices in modified files. README attribution alone
  does not replace this requirement.
- Preserve relevant upstream NOTICE content when supplied by an upstream
  distribution. Keep this repository's attribution notices with its assets.

For a standalone package or archive, include the license and applicable
notices in that distribution; a link to GitHub is not a substitute for the
license text. When importing additional files, record the source revision,
original license and attribution, and modifications before redistributing.

## Separate terms and unresolved provenance

The following are concrete gaps in the current source records. Adding an
Apache LICENSE at the root does not resolve them.

| Files/component | Evidence and remaining information |
|---|---|
| `om6dof_description/meshes/chain_link3_v2.stl` (formerly `chain_link3_v2stl`) | Not in either inspected arm tree. Confirm author/source, whether it modifies `chain_link3.stl`, and the changes and applicable license. The corrected filename does not resolve provenance. |
| `om6dof_description/meshes/d405.stl`, `d435.stl`, `stand_rs-d435_s01.stl` | Obtain the model sources and applicable redistribution terms. A hardware vendor's software license does not establish the license of a CAD model. |
| `om6dof_description/meshes/d405_wrist_cam.stl`, `d435_wrist_cam.stl` | Combined camera/bracket meshes. Confirm component sources/licenses and record assembly modifications; the verified standalone ALOHA bracket does not establish permission for the camera model or the combined assemblies. |
| `om6dof_dd_gng/realsense_ddgng/core` and `om6dof_dd_gng/DepthSensor_Buggy` | Files retain credits to Naoyuki Kubota, 首都大学東京, and Azhar Aulia Saputra, including "All rights reserved" notices. A package-level Apache declaration alone is insufficient evidence of permission for those pre-existing contributions; obtain their license or authorization. |
| `om6dof_dd_gng/third_party/drawstuff` | Existing headers identify Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith, and LGPL-2.1-or-later/BSD-style alternatives. The referenced `LICENSE.TXT` and `LICENSE-BSD.TXT` were not found in this copy. Identify the imported version and restore its license texts before distributing it under the applicable option. |

The original licenses and notices of `DynamixelSDK`, `dynamixel_interfaces`,
and `dynamixel_hardware_interface` remain in their respective directories.
This targeted review establishes the OpenManipulator description provenance;
it does not establish licenses for every dependency or historical research
snapshot in the repository. Until the outstanding sources are confirmed,
do not describe the entire collection as verified Apache-only material.
