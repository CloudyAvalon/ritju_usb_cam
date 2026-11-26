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

#ifndef USB_CAM__FRAME_SANITY_HPP_
#define USB_CAM__FRAME_SANITY_HPP_

#include <cstddef>
#include <string>

namespace usb_cam
{
namespace frame_sanity
{

/// @brief Validate the V4L2 payload length before it reaches decode/copy.
/// @param used      bytes reported by the driver (v4l2_buffer::bytesused)
/// @param capacity  allocated length of that V4L2 buffer
/// @return empty string when the payload is plausible, otherwise a short
///         reason suitable for logging.
inline std::string payload_error(size_t used, size_t capacity)
{
  if (used == 0) {
    return "empty payload (bytesused=0)";
  }
  if (capacity != 0 && used > capacity) {
    return "payload " + std::to_string(used) + " exceeds buffer capacity " +
           std::to_string(capacity);
  }
  return "";
}

/// @brief Reject an MJPEG payload that does not start with the JPEG SOI marker.
///
/// A live UVC MJPEG frame must start with FF D8.  A driver that hands back a
/// truncated/garbage frame usually loses this marker, and decoding such a
/// payload either fails or produces a meaningless image.
/// @return empty string when the payload starts with SOI, otherwise the reason.
inline std::string mjpeg_soi_error(const unsigned char * data, size_t size)
{
  if (data == nullptr || size < 2) {
    return "mjpeg payload too short for SOI";
  }
  if (data[0] != 0xFF || data[1] != 0xD8) {
    return "mjpeg payload missing SOI marker";
  }
  return "";
}

}  // namespace frame_sanity
}  // namespace usb_cam

#endif  // USB_CAM__FRAME_SANITY_HPP_
