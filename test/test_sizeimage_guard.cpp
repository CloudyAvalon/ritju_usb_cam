// Copyright 2026 usb_cam Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "usb_cam/utils.hpp"

using usb_cam::utils::fourcc_string;
using usb_cam::utils::effective_publish_rate;
using usb_cam::utils::framerate_matches;
using usb_cam::utils::sizeimage_bound;
using usb_cam::utils::sizeimage_error;
using usb_cam::utils::validate_frame_geometry;

TEST(SizeimageGuard, BoundForMjpeg)
{
  EXPECT_EQ(sizeimage_bound(640, 480, V4L2_PIX_FMT_MJPEG), 3686400u);
  EXPECT_EQ(sizeimage_bound(1280, 1024, V4L2_PIX_FMT_MJPEG), 15728640u);
}

TEST(SizeimageGuard, BoundForYuyv)
{
  EXPECT_EQ(sizeimage_bound(640, 480, V4L2_PIX_FMT_YUYV), 2457600u);
}

TEST(SizeimageGuard, ZeroIsLegal)
{
  EXPECT_TRUE(sizeimage_error(0, 640, 480, V4L2_PIX_FMT_MJPEG).empty());
}

TEST(SizeimageGuard, HealthyFrameIsAccepted)
{
  EXPECT_TRUE(sizeimage_error(921600, 640, 480, V4L2_PIX_FMT_MJPEG).empty());
  EXPECT_TRUE(sizeimage_error(3686400, 640, 480, V4L2_PIX_FMT_MJPEG).empty());
}

TEST(SizeimageGuard, AboveBoundIsRejected)
{
  EXPECT_FALSE(sizeimage_error(3686401, 640, 480, V4L2_PIX_FMT_MJPEG).empty());
  EXPECT_FALSE(sizeimage_error(104857600, 640, 480, V4L2_PIX_FMT_MJPEG).empty());
}

TEST(SizeimageGuard, Uint32GarbageIsRejected)
{
  const std::string reason =
    sizeimage_error(0xFFFFFFFFu, 640, 480, V4L2_PIX_FMT_MJPEG);
  EXPECT_FALSE(reason.empty());
  EXPECT_NE(reason.find("garbage"), std::string::npos);
}

TEST(SizeimageGuard, FourccIsPrintable)
{
  EXPECT_EQ(fourcc_string(V4L2_PIX_FMT_MJPEG), "MJPG");
  EXPECT_EQ(fourcc_string(V4L2_PIX_FMT_YUYV), "YUYV");
}

TEST(SizeimageGuard, InvalidGeometryHasNoBound)
{
  EXPECT_EQ(sizeimage_bound(0, 480, V4L2_PIX_FMT_MJPEG), 0u);
  EXPECT_TRUE(sizeimage_error(999999999, 0, 480, V4L2_PIX_FMT_MJPEG).empty());
}

TEST(FrameGeometry, Valid)
{
  EXPECT_EQ(validate_frame_geometry(640, 480, 3), 921600u);
  EXPECT_EQ(validate_frame_geometry(1280, 1024, 3), 3932160u);
}

TEST(FrameGeometry, RejectsNonPositive)
{
  EXPECT_THROW(validate_frame_geometry(0, 480, 3), std::invalid_argument);
  EXPECT_THROW(validate_frame_geometry(-640, 480, 3), std::invalid_argument);
  EXPECT_THROW(validate_frame_geometry(640, 0, 3), std::invalid_argument);
}

TEST(FrameGeometry, RejectsAbsurdDimensions)
{
  EXPECT_THROW(validate_frame_geometry(20000, 480, 3), std::out_of_range);
}

TEST(FrameGeometry, RejectsOversizedImage)
{
  EXPECT_THROW(validate_frame_geometry(8192, 8192, 4), std::out_of_range);
  EXPECT_THROW(validate_frame_geometry(16384, 16384, 3), std::out_of_range);
}

TEST(FramerateMatch, MatchesWithinTolerance)
{
  EXPECT_TRUE(framerate_matches(15.0, 15));
  EXPECT_TRUE(framerate_matches(15.09, 15));
  EXPECT_TRUE(framerate_matches(14.91, 15));
}

TEST(FramerateMatch, RejectsLargerDifference)
{
  EXPECT_FALSE(framerate_matches(15.1, 15));
  EXPECT_FALSE(framerate_matches(30.0, 15));
  EXPECT_FALSE(framerate_matches(0.0, 15));
  EXPECT_FALSE(framerate_matches(15.0, 0));
}

TEST(PublishRate, FallsBackToFramerate)
{
  EXPECT_DOUBLE_EQ(effective_publish_rate(15.0, 0.0), 15.0);
  EXPECT_DOUBLE_EQ(effective_publish_rate(30.0, 0.0), 30.0);
}

TEST(PublishRate, ExplicitValueWins)
{
  EXPECT_DOUBLE_EQ(effective_publish_rate(30.0, 10.0), 10.0);
  EXPECT_DOUBLE_EQ(effective_publish_rate(15.0, 30.0), 30.0);
}
