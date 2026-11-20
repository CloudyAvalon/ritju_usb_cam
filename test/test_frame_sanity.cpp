// Copyright 2024 usb_cam Authors
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

#include <string>

#include "usb_cam/frame_sanity.hpp"

using usb_cam::frame_sanity::mjpeg_soi_error;
using usb_cam::frame_sanity::payload_error;

TEST(FrameSanity, PayloadEmpty)
{
  EXPECT_FALSE(payload_error(0, 100).empty());
}

TEST(FrameSanity, PayloadOverCapacity)
{
  EXPECT_FALSE(payload_error(101, 100).empty());
}

TEST(FrameSanity, PayloadAtCapacity)
{
  EXPECT_TRUE(payload_error(100, 100).empty());
}

TEST(FrameSanity, PayloadUnknownCapacity)
{
  EXPECT_TRUE(payload_error(123, 0).empty());
}

TEST(FrameSanity, SoiValid)
{
  const unsigned char data[] = {0xFF, 0xD8, 0x00, 0x01};
  EXPECT_TRUE(mjpeg_soi_error(data, sizeof(data)).empty());
}

TEST(FrameSanity, SoiWrongMarker)
{
  const unsigned char data[] = {0x00, 0xD8, 0x00, 0x01};
  EXPECT_FALSE(mjpeg_soi_error(data, sizeof(data)).empty());
}

TEST(FrameSanity, SoiTooShort)
{
  const unsigned char data[] = {0xFF};
  EXPECT_FALSE(mjpeg_soi_error(data, sizeof(data)).empty());
}

TEST(FrameSanity, SoiNull)
{
  EXPECT_FALSE(mjpeg_soi_error(nullptr, 0).empty());
}
