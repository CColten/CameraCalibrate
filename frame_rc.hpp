#pragma once
#include "def.hpp"
#include "memory.hpp"

#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
namespace detail {
struct FrameMeta {
    Rc< std::lock_guard< std::mutex > > make_lock_guard() {
        return std::make_shared< std::lock_guard< std::mutex > >(m_mtx);
    }

    cv::Mat& get_data_ref() { return m_data; }

    size_t& get_cnt_ref() { return m_cnt; }

    int& get_camera_no() { return m_camera_no; }

    void set_open() { m_is_opened = true; }

    void set_close() { m_is_opened = false; }

    bool is_opened() { return m_is_opened; }

private:
    cv::Mat m_data;
    std::mutex m_mtx;
    size_t m_cnt = -1;
    int m_camera_no = -1;
    bool m_is_opened = false;
};
} // namespace detail

template < typename T > using Rc = std::shared_ptr< T >;
using FrameRc = Rc< detail::FrameMeta >;
