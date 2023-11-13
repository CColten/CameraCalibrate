/*
 * @Author: git config luochenguang@sensetime.com
 * @Date: 2023-08-07 11:51:50
 * @LastEditors: luochenguang
 * @LastEditTime: 2023-09-08 17:24:20
 * @FilePath: /CameraCalibration/include/mainwindow.h
 * @Description:
 *
 * Copyright (c) 2023 by SenseAuto, All Rights Reserved.
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QFont>
#include <QIntValidator>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QStandardItem>
#include <QtSerialPort>

#include <QButtonGroup>
#include <QCamera>
#include <QCameraInfo>
#include <QMediaRecorder>

#include "cameracalibration.h"
#include "lidarcalibration.h"
#include "yamlhelp.h"

// #include "camera/cam_car.hpp"
#include "camera/car_cam_recv.hpp"
#include "frame_rc.hpp"

#include "define_type.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
  void closeEvent(QCloseEvent *event);
  //
//  QButtonGroup *ratio_group_;

  struct Settings {
    QString name;
    qint32 baudRate;
    QString stringBaudRate;
    QSerialPort::DataBits dataBits;
    QString stringDataBits;
    QSerialPort::Parity parity;
    QString stringParity;
    QSerialPort::StopBits stopBits;
    QString stringStopBits;
    QSerialPort::FlowControl flowControl;
    QString stringFlowControl;
    bool localEchoEnabled;
    bool sendNewLineEnabled;
    QString stringStatus;
    bool isDtr;
    bool isRts;
    bool isHexDisplay;
    bool isHexSend;
    bool isTimerSend;
    qint32 timerLength;
    QString sendCache;
    QString sendStringCache;
    qint32 sendNum;
    qint32 receiveNum;
  } DEF_SETTINGS = {"",
                    QSerialPort::BaudRate::Baud115200,
                    QString::number(QSerialPort::Baud115200),
                    QSerialPort::DataBits::Data8,
                    QString::number(QSerialPort::DataBits::Data8),
                    QSerialPort::Parity::NoParity,
                    QString::number(QSerialPort::Parity::NoParity),
                    QSerialPort::StopBits::OneStop,
                    QString::number(QSerialPort::StopBits::OneStop),
                    QSerialPort::FlowControl::NoFlowControl,
                    QString::number(QSerialPort::FlowControl::NoFlowControl),
                    false,
                    true,
                    "",
                    false,
                    false,
                    false,
                    false,
                    false,
                    1000,
                    "",
                    "",
                    0,
                    0};
  Settings settings() const;

signals:
  void lidar_run_signal(double lidar_val);

  void grap_cam_image_signal(int index);
  void save_image_signal(int index, const cv::Mat &img);

  void open_car_cam_signal();

private slots:
  /**
   * @description: lidar write and send data
   */
  void writeData();
  /**
   * @description: lidar read data when get done signal
   */
  void readData();
  /**
   * @description: lidar data update
   */
  void currentIndexChanged();
  /**
   * @description:
   * @param {SerialPortError} error
   * @return {*}
   */
  void handleError(QSerialPort::SerialPortError error);

  void change_camera_index(int index);
  /**
   * @description: 打开/关闭串口
   */
  void on_bt_open_serial_clicked();
  /**
   * @description: 串口更新
   * @param {int} index
   */
  void on_serialPortInfoListBox_currentIndexChanged(int index);
  /**
   * @description:
   * @param {int} index
   * @return {*}
   */
  void on_radio_button_clicked(int index);
  /**
   * @description: 标定板ui更新
   * @param {QString&} qstr
   * @return {*}
   */
  void update_chess_board_config(const QString &qstr);
  /**
   * @description: anchor标定
   * @param {double} lidar_val
   * @return {*}
   */
  void run_lidar(double lidar_val);
  /**
   * @description: 视频图像更新到ui
   * @return {*}
   */
  void update_camera_video();
  /**
   * @description: 拍照按钮相应函数，配合三种模式进行切换
   *               1.内参标定模式
   *               2.瞄准模式
   *               3.anchor标定模式
   */
  void on_pushButton_grap_clicked();

  void select_file_from_folder_for_camera_params();
  void select_file_from_folder_for_lidar_params();
  void select_file_from_folder_for_anchors();

  void update_camera_params(const QString &qstr);
  void update_lidar_params(const QString &qstr);
  void update_anchors(const QString &qstr);

  void grap_image(int index);

  void save_image(int index, const cv::Mat &img);
  /**
   * @description: lidar发送信号
   * @return {*}
   */
  void LidarTest();

  void save_anchors_to_json_file();

  void save_anchors_to_txt_file();
  /**
   * @description: 打开车载摄像头
   * @return {*}
   */
  void open_car_camera();

  // 车机摄像头
  void handle_connect();
  void update_window();
  void recv_data(car_cam_recv_t *car_recv);
private:
  Ui::MainWindow *ui;

  QStandardItemModel *m_model;

private:
  YamlHelp yaml_help_;
  void init(const std::string &config_yaml);

private:
  ConfigInfo config_info_;
  CameraCalibration camera_calibrate_;
  LidarCalibration lidar_calibrate_;

  // 标定模式
  MODETYPE action_type_{MODETYPE::MODE_INTERNAL};

  cv::VideoCapture cam_;
  cv::Mat curr_img_;
  cv::Mat curr_processing_img_;

  // 内参标定所需的图片数量
  int num_calibrate_{20};
  // 当前内参标定所获取的图片数量，达到num_calibrate_后，需重置为0
  int curr_calibrate_num_{0};
  std::vector<cv::Mat> img_calibrate_;
  // 拍照标定图片保存路径
  std::string path_calibrate_images_;
  // json 结果保存文件
  std::string path_anchors_result_json_{"json_result.json"};
  // txt 结果保存文件
  std::string path_anchors_result_txt_{"txt_result.txt"};

  /*  lidar 信号处理  */
  Settings currentSettings;
  QSerialPort *serial;
  QString singal_str = "80 06 02 78";
  QString cur_lidar_info_;
  bool serial_opened_{false};

  void fillPortsInfo();
  bool openSerialPort();
  void closeSerialPort();
  void updateSettings();
  bool setParameter(QSerialPort *serial, Settings settings);
  Settings doSettings(bool isWrite, Settings inSettings);
  /*  lidar 信号处理  */

  const bool DEBUG = true;
  bool update_car_cam_data{false};

private:
  // camera
  //   QList<QCameraDevice> list_cameras;
  QScopedPointer<QCamera> my_camera;
  QScopedPointer<QMediaRecorder> my_mediaRecorder;
  //   QMediaCaptureSession my_captureSession;
  //   QImageCapture *imageCapture;

  bool camera_state;

  // 获取摄像头名称
  void getCamera();
  std::unordered_map<int, QString> cam_map_;

  // 定时刷新视频
  QTimer *timer_cam_;
  int time_cam_interval_{50};

  // 定时启动lidar测距
  QTimer *timer_lidar_;
  int time_lidar_interval_{1000};

  // 本地视频测试
  bool is_video_test_{false};
  std::string test_video_path_;

  // for test
  bool is_internal_params_test_{false};
  std::string internal_calibrate_image_test_path_;

  // 车机摄像头
  car_cam_recv_t m_car_cam_recv;
  std::string m_ip, m_port;
  int m_time_diff = 0;
  std::chrono::system_clock::time_point m_prev_rec_time;
  QTimer *timer_car_cam_;
  int time_car_cam_interval_{50};
  FrameRc m_origin_frame;
  bool is_car_cam_opened_{false};

  std::mutex mutex_;

  // test
  int frame_count_{0};

  void TestCalAnchorLocal();
};
#endif // MAINWINDOW_H
