// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "om6dof_dd_gng/camera_profile.hpp"

using namespace om6dof_dd_gng;

TEST(CameraProfile, LegacyDefaultsAndV2D435i)
{
  const auto v1 = cameraProfile("v1");
  EXPECT_EQ(v1.model, "D405");
  EXPECT_EQ(v1.depth_frame, "d405_depth_optical_frame");
  const auto v2 = cameraProfile("v2");
  EXPECT_EQ(v2.model, "D435i");
  EXPECT_EQ(v2.depth_frame, "d435_depth_optical_frame");
  EXPECT_EQ(cameraProfile("v2", "D435").model, "D435");
}

TEST(CameraProfile, RefusesMixedCameraAndFrames)
{
  EXPECT_THROW(cameraProfile("v3"), std::invalid_argument);
  EXPECT_THROW(cameraProfile("v2", "D405"), std::invalid_argument);
  EXPECT_THROW(cameraProfile("v1", "D435i"), std::invalid_argument);
  EXPECT_THROW(cameraProfile("v2", "D435f"), std::invalid_argument);
  EXPECT_THROW(cameraProfile("v2", "auto", "d405_depth_optical_frame"), std::invalid_argument);
  EXPECT_THROW(cameraProfile("v2", "auto", "d435_color_optical_frame"), std::invalid_argument);
}

TEST(CameraProfile, SelectsSingleMatchingCameraWithoutSerial)
{
  std::vector<CameraIdentity> devices{
    {"Intel RealSense D405", "100"}, {"Intel RealSense D435I", "000200"}};
  EXPECT_EQ(selectCameraSerial(devices, cameraProfile("v2")), "000200");
  EXPECT_THROW(selectCameraSerial(devices, cameraProfile("v2"), "100"), std::runtime_error);
  EXPECT_THROW(selectCameraSerial(devices, cameraProfile("v2", "D435")), std::runtime_error);
  EXPECT_FALSE(cameraNameMatches("Intel RealSense D435if", "D435i"));
}

TEST(CameraProfile, MultipleCamerasRequireSerialAndReconnectCannotSwitchUnit)
{
  const std::vector<CameraIdentity> devices{{"D435I", "1"}, {"D435I", "2"}};
  EXPECT_THROW(selectCameraSerial(devices, cameraProfile("v2")), std::runtime_error);
  EXPECT_EQ(selectCameraSerial(devices, cameraProfile("v2"), "1"), "1");
  EXPECT_THROW(selectCameraSerial({devices[1]}, cameraProfile("v2"), "1"), std::runtime_error);
}

TEST(CameraProfile, RejectsInvalidStaleOrFutureCaptureTime)
{
  EXPECT_TRUE(freshCapture(1000000000, 1050000000, .5));
  EXPECT_FALSE(freshCapture(0, 1050000000, .5));
  EXPECT_FALSE(freshCapture(1000000000, 1500000001, .5));
  EXPECT_FALSE(freshCapture(1000000001, 1000000000, .5));
  EXPECT_FALSE(freshCapture(1000000000, 1000000000, 0));
}
