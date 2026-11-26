// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Robert Bosch, LLC nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#define CLEAR(x) memset(&(x), 0, sizeof(x))

extern "C" {
#include <linux/videodev2.h>  // Defines V4L2 format constants
#include <malloc.h>  // for memalign
#include <sys/ioctl.h>  // for ioctl
#include <sys/mman.h>  // for mmap
#include <sys/stat.h>  // for stat
#include <unistd.h>  // for getpagesize()
#include <fcntl.h>  // for O_* constants and open()
}

#include <chrono>
#include <ctime>
#include <iostream>
#include <set>
#include <tuple>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "opencv2/imgproc.hpp"

#include "usb_cam/usb_cam.hpp"
#include "usb_cam/frame_sanity.hpp"
#include "usb_cam/conversions.hpp"
#include "usb_cam/utils.hpp"


namespace usb_cam
{

using utils::io_method_t;


UsbCam::UsbCam()
: m_device_name(), m_io(io_method_t::IO_METHOD_MMAP), m_fd(-1),
  m_number_of_buffers(4), m_buffers(new usb_cam::utils::buffer[m_number_of_buffers]), m_image(),
  m_avframe(NULL), m_avcodec(NULL), m_avoptions(NULL),
  m_avcodec_context(NULL), m_is_capturing(false), m_framerate(0),
  m_epoch_time_shift_us(usb_cam::utils::get_epoch_time_shift_us()), m_supported_formats()
{}

void UsbCam::log(log_level_t level, const std::string & msg) const
{
  if (m_logger) {
    m_logger(level, msg);
    return;
  }
  // Standalone fallback (unit tests, library users without a ROS logger).
  const bool to_stderr =
    level == log_level_t::WARN || level == log_level_t::WARN_THROTTLE ||
    level == log_level_t::ERROR;
  std::ostream & stream = to_stderr ? std::cerr : std::cout;
  stream << msg << std::endl;
}

UsbCam::~UsbCam()
{
  shutdown();
}


/// @brief Fill destination image with source image. If required, convert a given
/// V4L2 Image into another type. Look up possible V4L2 pixe formats in the
/// `linux/videodev2.h` header file.
/// @param src a pointer to a V4L2 source image
/// @param dest a pointer to where the source image should be copied (if required)
/// @param bytes_used number of bytes used by the src buffer
void UsbCam::process_image(const char * src, char * & dest, const int & bytes_used)
{
  // TODO(flynneva): could we skip the copy here somehow?
  // If no conversion required, just copy the image from V4L2 buffer
  if (m_image.pixel_format->requires_conversion() == false) {
    memcpy(dest, src, m_image.size_in_bytes);
  } else {
    m_image.pixel_format->convert(src, dest, bytes_used);
  }
}

bool UsbCam::read_frame()
{
  struct v4l2_buffer buf;
  unsigned int i;
  int len;

  switch (m_io) {
    case io_method_t::IO_METHOD_READ: {
        len = read(m_fd, m_buffers[0].start, m_buffers[0].length);
        if (len == -1) {
          switch (errno) {
            case EAGAIN:
              return false;
            default:
              throw std::runtime_error("Unable to read frame");
          }
        }
        std::string error =
          frame_sanity::payload_error(static_cast<size_t>(len), m_buffers[0].length);
        if (error.empty() && m_frame_sanity_enabled &&
          m_image.pixel_format->v4l2() == V4L2_PIX_FMT_MJPEG)
        {
          error = frame_sanity::mjpeg_soi_error(
            reinterpret_cast<const unsigned char *>(m_buffers[0].start), len);
        }
        if (!error.empty()) {
          m_last_frame_error = error;
          return false;
        }
        process_image(m_buffers[0].start, m_image.data, len);
        m_last_frame_error.clear();
        return true;
      }
    case io_method_t::IO_METHOD_MMAP: {
        m_image.v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        // Get current v4l2 pixel format
        if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_FMT), &m_image.v4l2_fmt)) {
          switch (errno) {
            case EAGAIN:
              return false;
            default:
              throw std::runtime_error("Invalid v4l2 format");
          }
        }

        // Hand a dequeued buffer back to the driver without processing it.
        const auto requeue = [this](unsigned int index) {
            v4l2_buffer buffer;
            CLEAR(buffer);
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = index;
            if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buffer)) {
              throw std::runtime_error("Unable to exchange buffer with the driver");
            }
          };

        /// Drain every completed buffer and keep only the newest frame.
        ///
        /// The driver keeps m_number_of_buffers frames queued in FIFO order, so
        /// dequeuing a single frame per tick always publishes the oldest one: a
        /// fixed latency of about (buffers - 1) / framerate (0.3 s at 10 Hz with
        /// 4 buffers).  Skipped frames are handed straight back to the driver
        /// without decoding, so the extra cost per tick is a few ioctls only.
        int newest_index = -1;
        timeval newest_stamp{};
        unsigned int newest_bytes = 0;

        for (unsigned int j = 0; j < m_number_of_buffers; ++j) {
          v4l2_buffer candidate;
          CLEAR(candidate);
          candidate.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          candidate.memory = V4L2_MEMORY_MMAP;

          if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_DQBUF), &candidate)) {
            if (errno == EAGAIN) {
              break;  // no frame has completed since the previous tick
            }
            throw std::runtime_error("Unable to retrieve frame with mmap");
          }

          if (newest_index < 0 || timercmp(&newest_stamp, &candidate.timestamp, <)) {
            if (newest_index >= 0) {
              // The previous candidate is no longer the newest: give it back
              // untouched, it will not be decoded.
              requeue(static_cast<unsigned int>(newest_index));
            }
            newest_index = static_cast<int>(candidate.index);
            newest_stamp = candidate.timestamp;
            newest_bytes = candidate.bytesused;
          } else {
            requeue(candidate.index);
          }
        }

        if (newest_index < 0) {
          // Nothing new: keep the previous image and its timestamp, the caller
          // will republish it (same behaviour as before this change).
          return false;
        }

        // Payload bounds are a memory-safety invariant: enforce them even when
        // the optional frame sanity checks are disabled.  The SOI check stays
        // behind the switch because it is heuristic.
        std::string error =
          frame_sanity::payload_error(newest_bytes, m_buffers[newest_index].length);
        if (error.empty() && m_frame_sanity_enabled &&
          m_image.pixel_format->v4l2() == V4L2_PIX_FMT_MJPEG)
        {
          error = frame_sanity::mjpeg_soi_error(
            reinterpret_cast<const unsigned char *>(m_buffers[newest_index].start), newest_bytes);
        }
        if (!error.empty()) {
          m_last_frame_error = error;
          requeue(static_cast<unsigned int>(newest_index));
          return false;
        }

        // Decode/copy only the newest frame; a decode failure is a dropped
        // frame, not a crash: the camera container must keep running.
        try {
          process_image(m_buffers[newest_index].start, m_image.data, newest_bytes);
        } catch (const std::exception & e) {
          m_last_frame_error = std::string("decode failed: ") + e.what();
          requeue(static_cast<unsigned int>(newest_index));
          return false;
        } catch (...) {
          m_last_frame_error = "decode failed: unknown exception";
          requeue(static_cast<unsigned int>(newest_index));
          return false;
        }
        m_image.stamp = usb_cam::utils::calc_img_timestamp(newest_stamp, m_epoch_time_shift_us);
        requeue(static_cast<unsigned int>(newest_index));
        m_last_frame_error.clear();
        return true;
      }
    case io_method_t::IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_DQBUF), &buf)) {
        switch (errno) {
          case EAGAIN:
            return false;
          default:
            throw std::runtime_error("Unable to exchange buffer with driver");
        }
      }

      // Get timestamp from V4L2 image buffer
      m_image.stamp = usb_cam::utils::calc_img_timestamp(buf.timestamp, m_epoch_time_shift_us);

      for (i = 0; i < m_number_of_buffers; ++i) {
        if (buf.m.userptr == reinterpret_cast<uint64_t>(m_buffers[i].start) && \
          buf.length == m_buffers[i].length)
        {
          return true;
        }
      }

      assert(i < m_number_of_buffers);
      process_image(reinterpret_cast<const char *>(buf.m.userptr), m_image.data, buf.bytesused);
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
        throw std::runtime_error("Unable to exchange buffer with the driver");
      }
      m_last_frame_error.clear();
      return true;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("IO method unknown");
  }
  return false;
}

void UsbCam::stop_capturing()
{
  if (!m_is_capturing) {return;}

  m_is_capturing = false;
  enum v4l2_buf_type type;

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      /* Nothing to do. */
      return;
    case io_method_t::IO_METHOD_MMAP:
    case io_method_t::IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_STREAMOFF, &type)) {
        // Set capturing variable to true again, since stream was not stopped successfully
        m_is_capturing = true;
        throw std::runtime_error("Unable to stop capturing stream");
      }
      return;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("IO method unknown");
  }
}

void UsbCam::start_capturing()
{
  if (m_is_capturing) {return;}

  unsigned int i;
  enum v4l2_buf_type type;

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      /* Nothing to do. */
      break;
    case io_method_t::IO_METHOD_MMAP:
      // Queue the buffers
      for (i = 0; i < m_number_of_buffers; ++i) {
        struct v4l2_buffer buf;
        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
          throw std::runtime_error("Unable to queue image buffer");
        }
      }

      // Start the stream
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_STREAMON, &type)) {
        throw std::runtime_error("Unable to start stream");
      }
      break;
    case io_method_t::IO_METHOD_USERPTR:
      for (i = 0; i < m_number_of_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = reinterpret_cast<uint64_t>(m_buffers[i].start);
        buf.length = m_buffers[i].length;

        if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
          throw std::runtime_error("Unable to configure stream");
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_STREAMON, &type)) {
        throw std::runtime_error("Unable to start stream");
      }
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("IO method unknown");
  }
  m_is_capturing = true;
}

void UsbCam::uninit_device()
{
  if (m_buffers) {
    if (m_io == io_method_t::IO_METHOD_MMAP) {
      for (unsigned int i = 0; i < m_allocated_buffers; ++i) {
        if (m_buffers[i].start != nullptr && m_buffers[i].length > 0) {
          munmap(m_buffers[i].start, m_buffers[i].length);
        }
      }
    } else if (m_io == io_method_t::IO_METHOD_USERPTR) {
      for (unsigned int i = 0; i < m_allocated_buffers; ++i) {
        free(m_buffers[i].start);
      }
    }
  }
  m_read_buffer.clear();
  m_read_buffer.shrink_to_fit();
  m_allocated_buffers = 0;
  m_buffers.reset();
}

void UsbCam::init_read()
{
  if (!m_buffers) {
    throw std::overflow_error("Out of memory");
  }

  // A READ-mode buffer must hold both the raw payload (sizeimage) and the
  // converted destination; previously start was never allocated at all.
  const size_t sizeimage = m_image.v4l2_fmt.fmt.pix.sizeimage;
  const size_t capacity = std::max(m_image.size_in_bytes, sizeimage);
  if (capacity == 0 || capacity > usb_cam::utils::kMaxImageBytes) {
    throw std::out_of_range(
            "read buffer " + std::to_string(capacity) + " bytes is invalid");
  }
  m_read_buffer.assign(capacity, 0);
  m_buffers[0].start = m_read_buffer.data();
  m_buffers[0].length = m_read_buffer.size();
  m_allocated_buffers = 1;
  log(
    log_level_t::INFO,
    "usb_cam: read buffer " + std::to_string(capacity) + " bytes");
}

void UsbCam::init_mmap()
{
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = m_number_of_buffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  log(
    log_level_t::INFO,
    "usb_cam: requesting " + std::to_string(m_number_of_buffers) +
    " mmap buffers via VIDIOC_REQBUFS");

  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_REQBUFS), &req)) {
    if (EINVAL == errno) {
      throw std::runtime_error("Device does not support memory mapping");
    } else {
      throw std::runtime_error("Unable to initialize memory mapping");
    }
  }

  if (req.count < m_number_of_buffers) {
    throw std::overflow_error("Insufficient buffer memory on device");
  }
  m_allocated_buffers = req.count;
  log(
    log_level_t::INFO,
    "usb_cam: VIDIOC_REQBUFS returned " + std::to_string(req.count) + " buffers");

  if (!m_buffers) {
    throw std::overflow_error("Out of memory");
  }

  for (uint32_t current_buffer = 0; current_buffer < req.count; ++current_buffer) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = current_buffer;

    if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYBUF), &buf)) {
      throw std::runtime_error("Unable to query status of buffer");
    }

    log(
      log_level_t::INFO,
      "usb_cam: QUERYBUF[" + std::to_string(current_buffer) + "] length=" +
      std::to_string(buf.length) + " bytes");
    if (buf.length == 0 || buf.length > usb_cam::utils::kAbsSizeimageCap) {
      throw std::runtime_error(
              "Driver reported buffer length " + std::to_string(buf.length) +
              "; refusing mmap (limit " +
              std::to_string(usb_cam::utils::kAbsSizeimageCap) + " bytes)");
    }
    m_buffers[current_buffer].length = buf.length;
    m_buffers[current_buffer].start =
      reinterpret_cast<char *>(mmap(
        NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */,
        MAP_SHARED /* recommended */, m_fd, buf.m.offset));

    if (MAP_FAILED == m_buffers[current_buffer].start) {
      throw std::runtime_error("Unable to allocate memory for image buffers");
    }
  }
}

void UsbCam::init_userp()
{
  struct v4l2_requestbuffers req;
  unsigned int page_size;

  page_size = getpagesize();
  if (m_image.size_in_bytes >
    std::numeric_limits<size_t>::max() - static_cast<size_t>(page_size - 1))
  {
    throw std::overflow_error("user-pointer buffer size overflow");
  }
  auto buffer_size = (m_image.size_in_bytes + page_size - 1) & ~(page_size - 1);

  CLEAR(req);

  req.count = m_number_of_buffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      throw std::invalid_argument("Device does not support user pointer i/o");
    } else {
      throw std::invalid_argument("Unable to initialize memory mapping");
    }
  }

  if (!m_buffers) {
    throw std::overflow_error("Out of memory");
  }
  m_allocated_buffers = req.count;

  for (uint32_t current_buffer = 0; current_buffer < req.count; ++current_buffer) {
    m_buffers[current_buffer].length = buffer_size;
    m_buffers[current_buffer].start =
      reinterpret_cast<char *>(memalign(/* boundary */ page_size, buffer_size));

    if (!m_buffers[current_buffer].start) {
      throw std::overflow_error("Out of memory");
    }
  }
}

void UsbCam::init_device()
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;

  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYCAP), &cap)) {
    if (EINVAL == errno) {
      throw std::invalid_argument("Device is not a V4L2 device");
    } else {
      throw std::invalid_argument("Unable to query device capabilities");
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    throw std::invalid_argument("Device is not a video capture device");
  }

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        throw std::invalid_argument("Device does not support read i/o");
      }
      break;
    case io_method_t::IO_METHOD_MMAP:
    case io_method_t::IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        throw std::invalid_argument("Device does not support streaming i/o");
      }
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("Unsupported IO method specified");
  }

  /* Select video input, video standard and tune here. */

  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_CROPCAP), &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  } else {
    /* Errors ignored. */
  }

  m_image.v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  m_image.v4l2_fmt.fmt.pix.width = m_image.width;
  m_image.v4l2_fmt.fmt.pix.height = m_image.height;
  m_image.v4l2_fmt.fmt.pix.pixelformat = m_image.pixel_format->v4l2();
  m_image.v4l2_fmt.fmt.pix.field = V4L2_FIELD_ANY;

  // Set v4l2 capture format
  // Note VIDIOC_S_FMT may change width and height
  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_FMT), &m_image.v4l2_fmt)) {
    throw strerror(errno);
  }

  // The UVC descriptor bug can report a garbage sizeimage after a resolution
  // switch.  VIDIOC_REQBUFS makes vb2 allocate `count * sizeimage` bytes, so
  // validating after REQBUFS is too late: the host OOMs before any frame is
  // read.  Verify G_FMT here and refuse to request buffers when the value is
  // implausible; an S_FMT kick heals the transient case.
  const uint32_t target_fourcc = m_image.v4l2_fmt.fmt.pix.pixelformat;
  for (int attempt = 0; attempt < 3; ++attempt) {
    m_image.v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_FMT), &m_image.v4l2_fmt)) {
      throw std::runtime_error("Unable to read the negotiated v4l2 format");
    }
    const auto & pix = m_image.v4l2_fmt.fmt.pix;
    if (pix.width == 0 || pix.height == 0 || pix.width > 16384 || pix.height > 16384) {
      throw std::runtime_error(
              "Driver reported implausible geometry " + std::to_string(pix.width) + "x" +
              std::to_string(pix.height) + "; refusing VIDIOC_REQBUFS");
    }
    std::string sizeimage_error =
      usb_cam::utils::sizeimage_error(pix.sizeimage, pix.width, pix.height, pix.pixelformat);
    if (sizeimage_error.empty() && m_io == io_method_t::IO_METHOD_MMAP) {
      // A zero sizeimage is "legal" for the driver, but for MMAP the kernel
      // then derives the buffer size from the driver's internal format.  When
      // that state is corrupt it allocates an enormous buffer at REQBUFS;
      // refuse instead of letting it OOM.
      if (pix.sizeimage == 0) {
        sizeimage_error = "sizeimage is 0 for mmap; refusing VIDIOC_REQBUFS";
      } else if (pix.bytesperline > 0) {
        const size_t implied = static_cast<size_t>(pix.bytesperline) * pix.height;
        const size_t bound =
          usb_cam::utils::sizeimage_bound(pix.width, pix.height, pix.pixelformat);
        if (bound > 0 && implied > bound) {
          sizeimage_error =
            "bytesperline " + std::to_string(pix.bytesperline) + " x height " +
            std::to_string(pix.height) + " implies " + std::to_string(implied) +
            " bytes, above bound " + std::to_string(bound) +
            "; refusing VIDIOC_REQBUFS";
        }
      }
    }
    if (sizeimage_error.empty()) {
      break;
    }
    if (attempt == 2) {
      throw std::runtime_error(sizeimage_error + "; refusing VIDIOC_REQBUFS");
    }
    log(log_level_t::WARN, "usb_cam: " + sizeimage_error + "; retrying VIDIOC_S_FMT");
    m_image.v4l2_fmt.fmt.pix.width = m_image.width;
    m_image.v4l2_fmt.fmt.pix.height = m_image.height;
    m_image.v4l2_fmt.fmt.pix.pixelformat = target_fourcc;
    m_image.v4l2_fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_FMT), &m_image.v4l2_fmt)) {
      throw strerror(errno);
    }
  }
  log(
    log_level_t::INFO,
    "usb_cam: negotiated " + std::to_string(m_image.v4l2_fmt.fmt.pix.width) + "x" +
    std::to_string(m_image.v4l2_fmt.fmt.pix.height) + " fourcc=" +
    usb_cam::utils::fourcc_string(m_image.v4l2_fmt.fmt.pix.pixelformat) +
    " sizeimage=" + std::to_string(m_image.v4l2_fmt.fmt.pix.sizeimage) +
    " bytesperline=" + std::to_string(m_image.v4l2_fmt.fmt.pix.bytesperline));
  if (m_image.v4l2_fmt.fmt.pix.width != m_image.width ||
    m_image.v4l2_fmt.fmt.pix.height != m_image.height)
  {
    throw std::runtime_error(
            "Driver changed geometry from " + std::to_string(m_image.width) + "x" +
            std::to_string(m_image.height) + " to " +
            std::to_string(m_image.v4l2_fmt.fmt.pix.width) + "x" +
            std::to_string(m_image.v4l2_fmt.fmt.pix.height) +
            "; refusing VIDIOC_REQBUFS");
  }

  if (m_set_framerate) {
    struct v4l2_streamparm stream_params;
    memset(&stream_params, 0, sizeof(stream_params));
    stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_PARM), &stream_params) < 0) {
      throw strerror(errno);
    }

    const auto & tpf = stream_params.parm.capture.timeperframe;
    const double current_fps =
      (tpf.numerator > 0) ? static_cast<double>(tpf.denominator) / tpf.numerator : 0.0;
    log(
      log_level_t::INFO,
      "usb_cam: current framerate " + std::to_string(current_fps) + " fps, requested " +
      std::to_string(m_framerate));

    if (!(stream_params.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
      // Do not call S_PARM on a device that cannot commit a frame interval;
      // the failed/partial commit is exactly what corrupts the UVC state.
      log(
        log_level_t::WARN,
        "usb_cam: V4L2_CAP_TIMEPERFRAME not supported; skipping VIDIOC_S_PARM");
    } else if (usb_cam::utils::framerate_matches(current_fps, m_framerate)) {
      // Already close enough: avoid another UVC probe/commit.
      log(
        log_level_t::INFO,
        "usb_cam: framerate already matches within 0.1 fps; skipping VIDIOC_S_PARM");
    } else {
      // TODO(lucasw) need to get list of valid numerator/denominator pairs
      // and match closest to what user put in.
      stream_params.parm.capture.timeperframe.numerator = 1;
      stream_params.parm.capture.timeperframe.denominator = m_framerate;
      log(log_level_t::INFO, "usb_cam: setting framerate via VIDIOC_S_PARM");
      if (usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_PARM), &stream_params) < 0) {
        throw std::invalid_argument("Couldn't set camera framerate");
      }
      log(log_level_t::INFO, "usb_cam: VIDIOC_S_PARM returned");
    }
  } else {
    log(log_level_t::INFO, "usb_cam: set_framerate=false; skipping VIDIOC_S_PARM");
  }

  // Re-read the format after the optional S_PARM: setting the frame interval
  // commits the UVC streaming control, and a corrupted commit can change the
  // sizeimage the kernel will use for REQBUFS.
  m_image.v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_FMT), &m_image.v4l2_fmt)) {
    throw std::runtime_error("Unable to read the v4l2 format after VIDIOC_S_PARM");
  }
  log(
    log_level_t::INFO,
    "usb_cam: after framerate setup " + std::to_string(m_image.v4l2_fmt.fmt.pix.width) + "x" +
    std::to_string(m_image.v4l2_fmt.fmt.pix.height) +
    " sizeimage=" + std::to_string(m_image.v4l2_fmt.fmt.pix.sizeimage) +
    " bytesperline=" + std::to_string(m_image.v4l2_fmt.fmt.pix.bytesperline));
  {
    const std::string post_error =
      usb_cam::utils::sizeimage_error(
      m_image.v4l2_fmt.fmt.pix.sizeimage, m_image.v4l2_fmt.fmt.pix.width,
      m_image.v4l2_fmt.fmt.pix.height, m_image.v4l2_fmt.fmt.pix.pixelformat);
    if (!post_error.empty()) {
      throw std::runtime_error(post_error + "; refusing VIDIOC_REQBUFS after S_PARM");
    }
  }

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      init_read();
      break;
    case io_method_t::IO_METHOD_MMAP:
      init_mmap();
      break;
    case io_method_t::IO_METHOD_USERPTR:
      init_userp();
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      // TODO(flynneva): log something
      break;
  }
}

void UsbCam::close_device()
{
  // Device is already closed
  if (m_fd == -1) {return;}

  if (-1 == close(m_fd)) {
    throw strerror(errno);
  }

  m_fd = -1;
}

void UsbCam::open_device()
{
  struct stat st;

  if (-1 == stat(m_device_name.c_str(), &st)) {
    throw strerror(errno);
  }

  if (!S_ISCHR(st.st_mode)) {
    throw strerror(errno);
  }

  m_fd = open(m_device_name.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == m_fd) {
    throw strerror(errno);
  }
}

void UsbCam::configure(
  parameters_t & parameters, const io_method_t & io_method)
{
  m_device_name = parameters.device_name;
  m_io = io_method;

  // Validate user input before opening the device or constructing a pixel
  // format: the MJPEG decoder allocates AVFrames in its constructor from these
  // dimensions, and image_t::set_size_in_bytes feeds the node's resize().
  usb_cam::utils::validate_frame_geometry(
    parameters.image_width, parameters.image_height, 4);
  if (parameters.framerate <= 0) {
    throw std::invalid_argument("framerate must be positive");
  }

  // Open device file descriptor before anything else
  open_device();

  m_set_framerate = parameters.set_framerate;
  m_image.width = static_cast<int>(parameters.image_width);
  m_image.height = static_cast<int>(parameters.image_height);
  m_image.set_number_of_pixels();

  // Do this before calling set_bytes_per_line and set_size_in_bytes
  m_image.pixel_format = set_pixel_format(parameters);
  m_image.set_bytes_per_line();
  m_image.set_size_in_bytes();
  m_framerate = parameters.framerate;

  init_device();
}

void UsbCam::start()
{
  start_capturing();
}

void UsbCam::shutdown()
{
  stop_capturing();
  uninit_device();
  close_device();
}

/// @brief Grab new image from V4L2 device, return pointer to image
/// @return pointer to image data
char * UsbCam::get_image()
{
  if ((m_image.width == 0) || (m_image.height == 0)) {
    return nullptr;
  }
  // grab the image
  if (!grab_image()) {
    return nullptr;
  }
  return m_image.data;
}

/// @brief Overload get_image so users can pass in an image pointer to fill
/// @param destination destination to fill in with image
bool UsbCam::get_image(char * destination)
{
  if ((m_image.width == 0) || (m_image.height == 0)) {
    return false;
  }
  // Set the destination pointer to be filled
  m_image.data = destination;
  // grab the image
  return grab_image();
}

std::vector<capture_format_t> UsbCam::get_supported_formats()
{
  // A broken UVC stack can keep answering ENUM_FMT / ENUM_FRAMESIZES /
  // ENUM_FRAMEINTERVALS with success for ever-increasing indices.  Without
  // caps this push_back loop grows until the host OOMs right after the
  // "This device supports the following formats:" line.
  constexpr uint32_t kMaxFormats = 32;
  constexpr uint32_t kMaxSizes = 64;
  constexpr uint32_t kMaxIntervals = 32;
  constexpr size_t kMaxEntries = 256;

  std::set<std::tuple<uint32_t, uint32_t, uint32_t, uint32_t, uint32_t>> seen;

  m_supported_formats.clear();
  log(log_level_t::INFO, "usb_cam: enumerating supported formats");

  v4l2_fmtdesc current_format{};
  current_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  for (uint32_t format_index = 0; format_index < kMaxFormats; ++format_index) {
    current_format.index = format_index;
    current_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (usb_cam::utils::xioctl(
        m_fd, static_cast<int>(VIDIOC_ENUM_FMT), &current_format) != 0)
    {
      break;
    }

    v4l2_frmsizeenum current_size{};
    current_size.pixel_format = current_format.pixelformat;
    for (uint32_t size_index = 0; size_index < kMaxSizes; ++size_index) {
      current_size.index = size_index;
      current_size.pixel_format = current_format.pixelformat;
      if (usb_cam::utils::xioctl(
          m_fd, static_cast<int>(VIDIOC_ENUM_FRAMESIZES), &current_size) != 0)
      {
        break;
      }
      if (current_size.type != V4L2_FRMSIZE_TYPE_DISCRETE) {
        continue;
      }

      v4l2_frmivalenum current_interval{};
      current_interval.pixel_format = current_size.pixel_format;
      current_interval.width = current_size.discrete.width;
      current_interval.height = current_size.discrete.height;
      for (uint32_t interval_index = 0; interval_index < kMaxIntervals; ++interval_index) {
        current_interval.index = interval_index;
        current_interval.pixel_format = current_size.pixel_format;
        current_interval.width = current_size.discrete.width;
        current_interval.height = current_size.discrete.height;
        if (usb_cam::utils::xioctl(
            m_fd, static_cast<int>(VIDIOC_ENUM_FRAMEINTERVALS), &current_interval) != 0)
        {
          break;
        }
        if (current_interval.type != V4L2_FRMIVAL_TYPE_DISCRETE) {
          continue;
        }
        const auto key = std::make_tuple(
          current_format.pixelformat,
          static_cast<uint32_t>(current_size.discrete.width),
          static_cast<uint32_t>(current_size.discrete.height),
          static_cast<uint32_t>(current_interval.discrete.numerator),
          static_cast<uint32_t>(current_interval.discrete.denominator));
        if (!seen.insert(key).second) {
          log(
            log_level_t::WARN,
            "usb_cam: duplicate mode during enumeration "
            "(driver ignores the index); stopping");
          return m_supported_formats;
        }
        if (m_supported_formats.size() >= kMaxEntries) {
          log(
            log_level_t::WARN,
            "usb_cam: format enumeration hit the " +
            std::to_string(kMaxEntries) + "-entry cap; ignoring the rest");
          return m_supported_formats;
        }
        capture_format_t capture_format{};
        capture_format.format = current_format;
        capture_format.v4l2_fmt = current_interval;
        m_supported_formats.push_back(capture_format);
      }  // interval loop
    }  // size loop
  }  // fmt loop

  log(
    log_level_t::INFO,
    "usb_cam: enumerated " + std::to_string(m_supported_formats.size()) +
    " capture modes");
  return m_supported_formats;
}

bool UsbCam::grab_image()
{
  fd_set fds;
  struct timeval tv;
  int r;

  FD_ZERO(&fds);
  FD_SET(m_fd, &fds);

  /* Timeout. */
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  r = select(m_fd + 1, &fds, NULL, NULL, &tv);

  if (-1 == r) {
    if (EINTR == errno) {
      // interrupted (e.g. maybe Ctrl + c) so don't throw anything
      return false;
    }

    log(
      log_level_t::ERROR,
      "Something went wrong, exiting... errno=" + std::to_string(errno));
    throw errno;
  }

  if (0 == r) {
    log(log_level_t::ERROR, "Select timeout, exiting...");
    throw "select timeout";
  }

  return read_frame();
}

// enables/disables auto focus
bool UsbCam::set_auto_focus(int value)
{
  struct v4l2_queryctrl queryctrl;
  struct v4l2_ext_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;

  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYCTRL), &queryctrl)) {
    if (errno != EINVAL) {
      log(log_level_t::ERROR, "VIDIOC_QUERYCTRL");
      return false;
    } else {
      log(log_level_t::WARN, "V4L2_CID_FOCUS_AUTO is not supported");
      return false;
    }
  } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    log(log_level_t::WARN, "V4L2_CID_FOCUS_AUTO is not supported");
    return false;
  } else {
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_FOCUS_AUTO;
    control.value = value;

    if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_CTRL), &control)) {
      log(log_level_t::ERROR, "VIDIOC_S_CTRL");
      return false;
    }
  }
  return true;
}

/**
* Set video device parameter via ioctl V4L2 controls.
*
* @param param The name of the parameter to set
* @param value The value to assign
* @return true on success, false on failure
*/
bool UsbCam::set_v4l_parameter(const std::string & param, int value)
{
  // Parameter name to V4L2 control ID mapping
  static const std::unordered_map<std::string, int> param_map = {
    {"brightness", V4L2_CID_BRIGHTNESS},
    {"contrast", V4L2_CID_CONTRAST},
    {"saturation", V4L2_CID_SATURATION},
    {"sharpness", V4L2_CID_SHARPNESS},
    {"gain", V4L2_CID_GAIN},
    {"white_balance_temperature_auto", V4L2_CID_DO_WHITE_BALANCE},
    {"white_balance_temperature", V4L2_CID_WHITE_BALANCE_TEMPERATURE},
    {"exposure_auto", V4L2_CID_EXPOSURE_AUTO},
    {"exposure_absolute", V4L2_CID_EXPOSURE_ABSOLUTE},
    {"focus_auto", V4L2_CID_FOCUS_AUTO},
    {"focus_absolute", V4L2_CID_FOCUS_ABSOLUTE},
  };

  // Look up control ID
  auto it = param_map.find(param);
  if (it == param_map.end()) {
    log(log_level_t::WARN, "usb_cam: unknown parameter '" + param + "'");
    return false;
  }
  int control_id = it->second;

  // Query if the control exists on this device
  struct v4l2_queryctrl query;
  CLEAR(query);
  query.id = control_id;
  if (ioctl(m_fd, VIDIOC_QUERYCTRL, &query) < 0) {
    log(
      log_level_t::WARN,
      "usb_cam: parameter '" + param + "' not supported by device");
    return false;
  }

  // Clamp value to device range
  int clamped = value;
  if (clamped < query.minimum) {clamped = query.minimum;}
  if (clamped > query.maximum) {clamped = query.maximum;}

  // Set the control value
  struct v4l2_control ctrl;
  CLEAR(ctrl);
  ctrl.id = control_id;
  ctrl.value = clamped;
  if (ioctl(m_fd, VIDIOC_S_CTRL, &ctrl) < 0) {
    log(
      log_level_t::WARN,
      "usb_cam: failed to set '" + param + "' to " + std::to_string(clamped));
    return false;
  }

  return true;
}

/**
* Set video device parameter via string value (converted to int).
*
* @param param The name of the parameter to set
* @param value The value to assign as string
* @return true on success, false on failure
*/
bool UsbCam::set_v4l_parameter(const std::string & param, const std::string & value)
{
  try {
    int int_value = std::stoi(value);
    return set_v4l_parameter(param, int_value);
  } catch (const std::exception & e) {
    log(
      log_level_t::ERROR,
      "usb_cam: invalid value '" + value + "' for parameter '" + param + "'");
    return false;
  }
}

}  // namespace usb_cam
