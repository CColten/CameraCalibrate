#pragma once
#include "asio.hpp"
#include "byte_queue.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#define RECV_LENGTH 4096
#define JSON_STR_SIZE 1600000 // json data size

using char8_t = char;
using uchar8_t = unsigned char;
using float32_t = float;
using float64_t = double;

#pragma pack(1)
// BYD 传输数据
struct RealtimeData {
  uint8_t magic[4];       // 4 Byte 固定头部
  uint8_t img_format;     // 1 Byte 图像格式
  uint32_t img_len;       // 4 Byte 图像数据长度
  uint16_t img_height;    // 2 Byte 图像高度
  uint16_t img_width;     // 2 Byte 图像宽度
  uint64_t ts_sec;        // 8 Byte 时间戳秒
  uint64_t ts_usec;       // 8 Byte 时间戳微秒
  uint32_t result_len;    // 4 Byte 结果长度
  uint8_t img_meta;       // 1 Byte 图片压缩信息
  uint8_t version;        // 1 Byte 版本信息，0x28表示SDKv2.8.0
  float32_t resize_scale; // 4 Byte 缩放比例
  uint8_t reserved[9];    // 9 Byte 保留9个字节
  uint8_t data[0];        // 图像数据指针
};
#pragma pack()

typedef enum PixelFormat {
  M_PIX_FMT_GRAY8 = 1,             /**< gray8  */
  M_PIX_FMT_RGBA8888 = 2,          /**< rgba8888 */
  M_PIX_FMT_RGB888 = 3,            /**< rgb888 */
  M_PIX_FMT_RGB888_PLANAR = 4,     /**< rgb888 RRRRRR:GGGGGG:BBBBBB */
  M_PIX_FMT_BGRA8888 = 5,          /**< bgra8888 */
  M_PIX_FMT_BGR888 = 6,            /**< bgr888 */
  M_PIX_FMT_BGR888_PLANAR = 7,     /**< bgr888 BBBBBB:GGGGGG:RRRRRR */
  M_PIX_FMT_YUV420P = 8,           /**< yuv420p */
  M_PIX_FMT_NV12 = 9,              /**< YUV  4:2:0 YYYY:UV */
  M_PIX_FMT_NV21 = 10,             /**< YUV  4:2:0 YYYY:VU */
  M_PIX_FMT_GRAY32 = 11,           /**< gray32*/
  M_PIX_FMT_RGB323232 = 12,        /**< rgb323232 fp32*/
  M_PIX_FMT_RGB323232_PLANAR = 13, /**< rgb323232 fp32  RRRRRR:GGGGGG:BBBBBB*/
  M_PIX_FMT_BGR323232 = 14,        /**< bgr323232 fp32*/
  M_PIX_FMT_BGR323232_PLANAR = 15, /**< bgr323232 fp32 BBBBBB:GGGGGG:RRRRRR*/
  M_PIX_FMT_GRAY16 = 16,           /**< gray16*/
  M_PIX_FMT_RGB161616 = 17,        /**< rgb161616 fp16*/
  M_PIX_FMT_RGB161616_PLANAR = 18, /**< rgb161616 fp16  RRRRRR:GGGGGG:BBBBBB*/
  M_PIX_FMT_BGR161616 = 19,        /**< bgr161616 fp16*/
  M_PIX_FMT_BGR161616_PLANAR = 20, /**< bgr161616 fp16 BBBBBB:GGGGGG:RRRRRR*/
  M_PIX_FMT_FLOAT32C4 = 21,        /**< fp32 channel ==4 */
  M_PIX_FMT_NV12_DETACH = 22,      /**< Y/UV not Contiguous memory*/
  M_PIX_FMT_NV21_DETACH = 23,      /**< Y/UV not Contiguous memory*/
  M_PIX_FMT_YUYV = 24,             /**< YUV422 PACKED*/
  M_PIX_FMT_UYVY = 25,             /**< YUV422 PACKED*/
  M_PIX_FMT_YV12 = 26,             /**< YUV  4:2:0 YYYYYYYY:VVUU */
  M_PIX_FMT_YU12 = 27,             /**< YUV  4:2:0 YYYYYYYY:UUVV */
  M_PIX_FMT_MAX = 28               /**< pixel format is invalid */
} PixelFormat;

/** A enum to represent pixel format */
typedef PixelFormat MPixelFormat;

using asio::ip::tcp;

struct car_cam_recv_t {
    car_cam_recv_t() {
        realtime_data =
            ( RealtimeData* )new uint8_t[sizeof(RealtimeData) + 1920 * 1080 * 3 + JSON_STR_SIZE];
    }
  ~car_cam_recv_t() {
        m_is_opened = false;
        if (nullptr != realtime_data) {
            delete[] realtime_data;
        }
    }
  bool open(std::string ip, std::string port);

  bool close() {
    m_is_opened = false;
    if (socket->is_open()) {
        socket->close();
    }

    return m_is_opened;
  }

  bool is_opened() { return m_is_opened; }

  bool getOneFramePacket(RealtimeData *realtime_data);

  // <data,seconds,microseconds>
  void recv_img();
  bool get_one_pic(cv::Mat &cvimg);

  std::shared_ptr<asio::ip::tcp::socket> socket;
  std::thread io_thread;
  asio::io_service ios;
  asio::io_context io_context;
  DataByteQueue bq;
  bool m_is_opened = false;
  RealtimeData* realtime_data;

  // test
  int frame_count_{0};
};
