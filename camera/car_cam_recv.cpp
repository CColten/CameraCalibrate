/*
 * @Author: git config luochenguang@sensetime.com
 * @Date: 2023-08-08 14:40:58
 * @LastEditors: luochenguang
 * @LastEditTime: 2023-09-08 19:19:09
 * @FilePath: /CameraCalibration/camera/car_cam_recv.cpp
 * @Description:
 *
 * Copyright (c) 2023 by SenseAuto, All Rights Reserved.
 */
#include "car_cam_recv.hpp"

#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <QThread>
#include <QDebug>

bool img_compress_proc(cv::Mat &cvimg, RealtimeData *realtime_data) {
  cv::Mat rawData =
      cv::Mat(1, realtime_data->img_len, CV_8UC1, (void *)realtime_data->data);
  if (M_PIX_FMT_GRAY8 == realtime_data->img_format) {
    // std::cout << "recv M_PIX_FMT_GRAY8 compress" << std::endl;
    cvimg = cv::imdecode(rawData, 0);
    cv::cvtColor(cvimg, cvimg, cv::COLOR_GRAY2BGR);
  } else if (M_PIX_FMT_BGR888 == realtime_data->img_format) {
    // std::cout << "recv M_PIX_FMT_BGR888 compress" << std::endl;
    cvimg = cv::imdecode(rawData, 3);
    cv::cvtColor(cvimg, cvimg, cv::COLOR_RGB2BGR);
  } else if (M_PIX_FMT_RGB888 == realtime_data->img_format) {
    std::cout << "recv M_PIX_FMT_RGB888 compress" << std::endl;
    cvimg = cv::imdecode(rawData, 3);
    // cv::cvtColor(cvimg, cvimg, cv::COLOR_RGB2BGR);
  } else if (M_PIX_FMT_NV21 == realtime_data->img_format) {
    std::cout << "recv M_PIX_FMT_NV21 compress" << std::endl;
    cvimg = cv::imdecode(rawData, 3);
    // cv::cvtColor(cvimg, cvimg, cv::COLOR_RGB2BGR);
  } else {
    std::cout << "unsupported img format! compress" << std::endl;
    return false;
  }

  return true;
}

bool img_noncompress_proc(cv::Mat &cvimg, RealtimeData *realtime_data) {
  if (M_PIX_FMT_GRAY8 == realtime_data->img_format) {
    // std::cout << "recv M_PIX_FMT_GRAY8" << std::endl;
    cvimg = cv::Mat(realtime_data->img_height, realtime_data->img_width,
                    CV_8UC1, (void *)realtime_data->data);
    cv::cvtColor(cvimg, cvimg, cv::COLOR_GRAY2BGR);
  } else if (M_PIX_FMT_BGR888 == realtime_data->img_format) {
    // std::cout << "recv M_PIX_FMT_BGR888" << std::endl;
    cvimg = cv::Mat(realtime_data->img_height, realtime_data->img_width,
                    CV_8UC3, (void *)realtime_data->data);
  } else if (M_PIX_FMT_RGB888 == realtime_data->img_format) {
    // std::cout << "recv M_PIX_FMT_RGB888" << std::endl;
    cvimg = cv::Mat(realtime_data->img_height, realtime_data->img_width,
                    CV_8UC3, (void *)realtime_data->data);
    cv::cvtColor(cvimg, cvimg, cv::COLOR_RGB2BGR);
  } else if (M_PIX_FMT_NV21 == realtime_data->img_format) {
    // std::cout << "recv M_PIX_FMT_NV21" << std::endl;
    cvimg = cv::Mat(realtime_data->img_height * 3 / 2, realtime_data->img_width,
                    CV_8UC1, (void *)realtime_data->data);
    cv::cvtColor(cvimg, cvimg, cv::COLOR_YUV2BGR_I420);
    if (cvimg.data == NULL) {
      std::cout << "Create rgb image failed";
    }
  } else {
    std::cout << "unsupported img format!" << std::endl;
    return false;
  }

  return true;
}

bool car_cam_recv_t::open(std::string ip, std::string port) {
  try {
    socket = std::make_shared<asio::ip::tcp::socket>(ios);
    tcp::resolver resolver(io_context);
    asio::error_code error;
    qDebug() << "connect to " <<ip.c_str() << ":" << port.c_str();
    asio::connect(*socket, resolver.resolve(ip, port), error);
    if (error) {
      return false;
    }
    m_is_opened = true;
    qDebug() << "Connect to svr succ";
  } catch (...) {
    m_is_opened = false;
    return false;
  }

  return true;
}

bool car_cam_recv_t::getOneFramePacket(RealtimeData *realtime_data) {
  std::unique_lock<std::mutex> lock(bq.getMutex());
  if (bq.getCurrentSize() < sizeof(RealtimeData)) {
    return false;
  }

  RealtimeData *data = (RealtimeData *)bq.getData();
  if (data->magic[0] != 0xAA || data->magic[1] != 0xBB ||
      data->magic[2] != 0xCC || data->magic[3] != 0xDD) {
    bq.clearBuffer();
    return false;
  }

  uint32_t total_len = sizeof(RealtimeData) + data->img_len + data->result_len;
  //std::cout << "total_len: " << total_len << " getCurrentSize: " << bq.getCurrentSize();
  if (bq.getCurrentSize() < total_len) {
    return false;
  }

  memcpy((void *)realtime_data, (void *)data, total_len);
  bq.readOver(total_len);
  return true;
}

void car_cam_recv_t::recv_img()
{
    try {
        while (m_is_opened) {
            if (socket->is_open()) {
                uint8_t buffer[RECV_LENGTH] = {0};
                size_t recv_length = asio::read(*socket, asio::buffer(buffer, RECV_LENGTH));
                bq.addData(buffer, recv_length);
                static uint8_t send_buf[8] = {0xAA, 0xAA, 0x55, 0x55, 0x00, 0x00, 0x00, 0x00};
                static int keep_alive = 100;
                if (--keep_alive < 0) {
                keep_alive = 100;
                asio::write(*socket, asio::buffer(send_buf, 8)); // inform server that i am still alive
                }
            } else {
                qDebug() << "Disconnected  ---";
                m_is_opened= false;
            }
        }

        if (socket->is_open()) {
            socket->close();
        }
    } catch (std::exception err) {
        m_is_opened= false;
        if (socket->is_open()) {
            socket->close();
        }
        qDebug() << "Exception: " << err.what();
    }
}

bool car_cam_recv_t::get_one_pic(cv::Mat &cvimg) {
    if (getOneFramePacket(realtime_data)) {
        //qDebug() << "img_height: " << realtime_data->img_height << "  img_width: " << realtime_data->img_width;
        static cv::Mat cvimg_tmp;
        if (M_PIX_FMT_BGR888 == realtime_data->img_format) {
            //qDebug() << "recv M_PIX_FMT_BGR888";
            cvimg_tmp = cv::Mat(realtime_data->img_height, realtime_data->img_width, CV_8UC3, (void *)realtime_data->data);
        } else if (M_PIX_FMT_NV21 == realtime_data->img_format) {
            //qDebug() << "recv M_PIX_FMT_NV21";
            cvimg_tmp = cv::Mat(realtime_data->img_height * 3 / 2, realtime_data->img_width, CV_8UC1, (void *)realtime_data->data);
            cv::cvtColor(cvimg_tmp, cvimg_tmp, cv::COLOR_YUV2BGR_I420);
        }

        cvimg = cvimg_tmp;
        return true;
    }
    return false;
}

