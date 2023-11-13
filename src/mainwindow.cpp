#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "json/json11.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/video.hpp"
#include "opencv2/videoio.hpp"

#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
// #include <qDebug>
#include <QDir>
#include <fstream>
#include <thread>

#ifdef _WIN32
#include <io.h>
#elif __linux__
#include <unistd.h>
#endif

#include <future>
#include <thread>

template <typename R, typename Fn>
std::future<R> start_detached_future(Fn func) {
  std::promise<R> pro;
  std::future<R> fut = pro.get_future();
  std::thread([func](std::promise<R> p) { p.set_value(func()); },
              std::move(pro))
      .detach();
  return fut;
}

#define let const auto
#define mut auto

using MJson = json11::Json;

/** 将hex(61 62 63 64 65 66 67) to String(abcdefg) */
static QString hexToQString(bool isDebug, QString hexStr) {
  if (isDebug) {
  }
  // qDebug() << __func__ << ": " << hexStr;
  QString ret;
  QStringList list = hexStr.split(" ", QString::SkipEmptyParts);
  for (QString qs : list) {
    bool bStatus = false;
    int a = qs.toInt(&bStatus, 16);
    if (bStatus && (qs.length() == 2)) {
      QString sA = QString(QChar(a));
      if (isDebug) {
      }
      // qDebug() << "a:" << a << "sA:" << sA << "qs:" + qs;
      ret.append(sA);
    } else { /* 转换失败，使用默认字体串 */
      if (isDebug) {
      }
      // qDebug() << "error!!!";
      //            ret = "abcdefg";
      break;
    }
  }
  return ret;
}

/** 将String(abcdefg) to Hex(61 62 63 64 65 66 67) */
static QString stringToHex(bool isDebug, QString str) {
  if (isDebug) {
  }
  // qDebug() << __func__ << ": " << str;
  QString ret;
  /* 将String(abcdefg) to Hex */
  for (int i = 0; i < str.length(); i++) {
    /* 将字符串中字符转换成QChar */
    QChar random = str.at(i).toLatin1();
    QString hex;
    QString str1;
    /* 将QChar转换成unicode */
    hex.setNum(random.unicode(), 16);
    /**
     * 由于unicode位数随机，所以需要根据情况进行格式化
     * 只保留末两位，如果不够两位补0
     */
    if (hex.length() >= 2) {
      if (isDebug) {
      }
      // qDebug() << "hex.length() >= 2 hex:" << hex;
      str1 = hex.mid(hex.length() - 2, hex.length());
    } else if (hex.length() == 1) {
      if (isDebug) {
      }
      // qDebug() << "hex.length() == 1 hex:" << hex;
      str1 = hex.prepend("0");
    } else {
      if (isDebug) {
      }
      // qDebug() << "else";
      str1 = "";
    }
    ret.append(str1.toUpper() + " ");
  }
  return ret;
}

float GetLidarValue(QString str) {
  QString result_str;
  QStringList list = str.split(" "); // QString字符串分割函数
  if (list.size() < 12) {

  } else {
    for (qsizetype i = 0; i < 12; ++i) {
      if (i > 2 && i < 11 && i != 6) {
        result_str += list[i].right(1);
      } else {
        if (i == 6) {
          result_str += ".";
        }
      }
    }
  }

  return result_str.toFloat();
}

inline void GetFolders(std::string path, std::vector<std::string> &files) {
    QDir dirinfo = QDir(path.c_str());
    if (!dirinfo.exists()) {
        qDebug() << path.c_str() << " not exist";
    } else {
        QStringList fileList = dirinfo.entryList(QDir::Files);
        fileList.removeOne(".");
        fileList.removeOne("..");
        for (const auto &item : fileList) {
            files.push_back(item.toStdString());
        }
    }
}

void MainWindow::init(const std::string &config_yaml) {
  std::map<std::string, std::string> key_vals;

  yaml_help_.GetYamlFromCV(config_yaml, config_info_);

    ui->lineEdit_2->setText(QString::number(config_info_.chess_board_size));

    ui->lineEdit_3->setText(QString::number(config_info_.chess_board_rows));
    ui->lineEdit_4->setText(QString::number(config_info_.chess_board_cols));

    camera_calibrate_.SetBoardSize(config_info_.chess_board_size,
                                 config_info_.chess_board_rows,
                                 config_info_.chess_board_cols);

    camera_calibrate_.SetNumReqFrames(config_info_.internal_calibrate_image_num);
    camera_calibrate_.SetWindowSize(config_info_.window_size);

    if (config_info_.camera_internal_params_file != "")
        ui->lineEdit_5->setText(QString::fromStdString(config_info_.camera_internal_params_file));
    if (config_info_.lidar_params_file != "")
        ui->lineEdit_6->setText(QString::fromStdString(config_info_.lidar_params_file));
    if (config_info_.anchors_file != "")
        ui->lineEdit_7->setText(QString::fromStdString(config_info_.anchors_file));

    time_lidar_interval_ = config_info_.lidar_interval_time;
    path_calibrate_images_ = config_info_.internal_calibrate_image_save_path;

    camera_calibrate_.SetIsSaveImagePointsImg(config_info_.is_save_image_points);

    QDir dir;
    //没有photograph文件夹则创建
    if (dir.exists(QString::fromStdString(path_calibrate_images_)) == false) {
        dir.mkpath(QString::fromStdString(path_calibrate_images_));
    }

    if (dir.exists(QString::fromStdString("pic")) == false) {
        dir.mkpath(QString::fromStdString("pic"));
    }


    is_internal_params_test_ =config_info_.is_internal_params_test;

    path_anchors_result_json_ = config_info_.anchors_result_json_save_path;

    path_anchors_result_txt_ = config_info_.anchors_result_txt_save_path;

    is_video_test_ = config_info_.is_video_test;

    if (is_video_test_) {
        test_video_path_ = config_info_.test_video_path;
     }

    internal_calibrate_image_test_path_=config_info_.internal_calibrate_image_test_path;


    // // qDebug() << "is_video_test: " << is_video_test_;
    // // qDebug() << "test_video_path: " << test_video_path_;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  ui->label->setStyleSheet("border: 1px solid red;");

  ui->tableView->installEventFilter(this);

  m_model = new QStandardItemModel();
  ui->tableView->setModel(m_model);

  ui->lineEdit_car_ip->setText("192.168.46.1");
  ui->lineEdit_car_port->setText("65510");

  ui->radioButton->setChecked(true);
  ui->radioButton_2->setChecked(false);
  ui->radioButton_3->setChecked(false);

//  ratio_group_ = new QButtonGroup(this);
//  ratio_group_->addButton(ui->radioButton, 1);
//  ratio_group_->addButton(ui->radioButton_2, 2);
//  ratio_group_->addButton(ui->radioButton_3, 3);

//  connect(ratio_group_, SIGNAL(idClicked(int)), this,
//          SLOT(on_radio_button_clicked(int)));

  QIntValidator *validator = new QIntValidator(this);
  ui->lineEdit_2->setText("15");
  ui->lineEdit_3->setText("7");
  ui->lineEdit_4->setText("10");

  ui->lineEdit_2->setValidator(validator);
  ui->lineEdit_3->setValidator(validator);
  ui->lineEdit_4->setValidator(validator);

  ui->lineEdit_5->setText("cam_config.yaml");
  ui->lineEdit_6->setText("lidar_config.yaml");
  ui->lineEdit_7->setText("anchors.json");

  ui->lineEdit_5->setReadOnly(true);
  ui->lineEdit_6->setReadOnly(true);
  ui->lineEdit_7->setReadOnly(true);

  ui->lineEdit_11->setText("0");

  m_model->setColumnCount(2);
  m_model->setHeaderData(0, Qt::Horizontal, "序号");
  m_model->setHeaderData(1, Qt::Horizontal, "anchors点位坐标");

  ui->tableView->setColumnWidth(0, 100);
  ui->tableView->setColumnWidth(1, ui->tableView->width() - 100);

  for (int i = 0; i < 48; i++) {
    m_model->setItem(
        i, 0, new QStandardItem(QString::fromStdString(std::to_string(i))));
    m_model->setItem(i, 1, new QStandardItem("11"));
  }

  ui->lineEdit_car_ip->setText("192.168.46.1");
  ui->lineEdit_car_port->setText("65510");

  //! [1]
  serial = new QSerialPort(this);
  connect(serial, SIGNAL(errorOccurred(QSerialPort::SerialPortError)), this,
          SLOT(handleError(QSerialPort::SerialPortError)));

  //! [2]
  connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));

  connect(this, SIGNAL(lidar_run_signal(double)), this,
          SLOT(run_lidar(double)));

  connect(ui->lineEdit_2, SIGNAL(textChanged(QString)), this,
          SLOT(update_chess_board_config(QString)));
  connect(ui->lineEdit_3, SIGNAL(textChanged(QString)), this,
          SLOT(update_chess_board_config(QString)));
  connect(ui->lineEdit_4, SIGNAL(textChanged(QString)), this,
          SLOT(update_chess_board_config(QString)));
  connect(ui->serialPortInfoListBox, SIGNAL(currentIndexChanged(int)), this,
          SLOT(on_serialPortInfoListBox_currentIndexChanged(int)));
  connect(ui->cmb_camera_list, SIGNAL(currentIndexChanged(int)), this,
          SLOT(change_camera_index(int)));

  timer_cam_ = new QTimer();
  timer_lidar_ = new QTimer();
  timer_car_cam_ = new QTimer();

  connect(timer_cam_, SIGNAL(timeout()), this, SLOT(update_camera_video()));
  connect(timer_lidar_, SIGNAL(timeout()), this, SLOT(LidarTest()));
  //connect(timer_car_cam_, SIGNAL(timeout()), this, SLOT(update_window()));

  connect(ui->pushButtonC, SIGNAL(clicked()), this,
          SLOT(select_file_from_folder_for_camera_params()));
  connect(ui->pushButtonC_2, SIGNAL(clicked()), this,
          SLOT(select_file_from_folder_for_lidar_params()));
  connect(ui->pushButtonC_3, SIGNAL(clicked()), this,
          SLOT(select_file_from_folder_for_anchors()));

  connect(ui->lineEdit_5, SIGNAL(textChanged(QString)), this,
          SLOT(update_camera_params(QString)));
  connect(ui->lineEdit_6, SIGNAL(textChanged(QString)), this,
          SLOT(update_lidar_params(QString)));
  connect(ui->lineEdit_7, SIGNAL(textChanged(QString)), this,
          SLOT(update_anchors(QString)));

  connect(this, SIGNAL(grap_cam_image_signal(int)), this,
          SLOT(grap_image(int)));
  connect(this, SIGNAL(save_image_signal(int, cv::Mat)), this,
          SLOT(save_image(int, cv::Mat)));

  connect(ui->pushButtonSaveAnchors, SIGNAL(clicked()), this,
          SLOT(save_anchors_to_json_file()));
  connect(ui->pushButtonSaveToTxt, SIGNAL(clicked()), this,
          SLOT(save_anchors_to_txt_file()));

  connect(ui->bt_open_car_camera, SIGNAL(clicked()), this,
          SLOT(open_car_camera()));
  connect(this, SIGNAL(open_car_cam_signal()), this, SLOT(handle_connect()));

  init("config.yaml");

  m_origin_frame = std::make_shared<detail::FrameMeta>();

  fillPortsInfo();
  updateSettings();
  getCamera();

  if (is_video_test_) {
    if (_access(test_video_path_.c_str(), 0) == -1) {
      // qDebug() << test_video_path_ << " not exist";
      QMessageBox::information(this, tr("Tip"), tr("test video not exist"));
    } else {
      this->cam_.open(test_video_path_);
    }
  } else {
    auto cam_index = ui->cmb_camera_list->currentIndex();

    this->cam_.open(0);
    if (!this->cam_.isOpened()) {
      QMessageBox::information(this, tr("Warn"), tr("open camera failed"));
    }
//    cv::Mat img;
//    this->cam_.read(img);
//    cv::imwrite("hello_init.jpg", img);
  }

  timer_cam_->start(time_cam_interval_);
}

MainWindow::~MainWindow() {
    if (nullptr != m_model) {
        delete m_model;
    }
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    qDebug() << "closr win";
    update_car_cam_data = false;
    m_car_cam_recv.close();
}

void MainWindow::on_radio_button_clicked(int index) {
  // 处理单选框被选中时的事件
  // // qDebug() << "Radio button " << index << " clicked";
  switch (index) {
  case 1:
  case 2:
    if (timer_lidar_->isActive()) {
      // qDebug() << "timer_lidar_  is active";
      timer_lidar_->stop();
    }
    // if (!timer_cam_->isActive()){
    //     // qDebug() << "timer_cam_  is close";
    //     timer_cam_->start(time_cam_interval_);
    // }
    break;
  default:
    break;
  }
}

void MainWindow::TestCalAnchorLocal(){

    cv::Mat img = cv::imread("run_lidar_image.jpg");

    curr_processing_img_ = img.clone();
    // lidar 测试
    float lidar = 1.063f;
    emit lidar_run_signal(static_cast<double>(lidar * 1000));
}

/*
 * brief:拍照：同步触发拍照和lidar测距
*/
void MainWindow::on_pushButton_grap_clicked() {
  if (ui->radioButton->isChecked()) {
    timer_lidar_->stop();
    // if (!timer_cam_->isActive()){
    //     timer_cam_->start(time_cam_interval_);
    // }
    // 内参标定模式，拍照，不进行lidar测距
    int grap_num = camera_calibrate_.num_req_frames_;
    std::vector<cv::Mat> imgs;
    cv::Mat img;

    if (!this->is_car_cam_opened_) {
        if (!this->cam_.isOpened()){
            auto camera_index = ui->cmb_camera_list->currentIndex();
            this->cam_.open(camera_index);
        }
        this->cam_.read(img);
    }else{
        std::unique_lock<std::mutex> lock(mutex_);
        img = curr_img_.clone();
    }

    if (!is_internal_params_test_) {
        ui->pushButton_grap->setEnabled(false);
        if (curr_calibrate_num_ < num_calibrate_) {

            if (img.empty()) {
                QMessageBox::warning(NULL, "warning", "get photo failed, please retry",
                                   QMessageBox::Yes | QMessageBox::No,
                                   QMessageBox::Yes);
                ui->pushButton_grap->setEnabled(true);
                return;
            }

        auto err = camera_calibrate_.ProccessSingleImage(img, MODETYPE::MODE_INTERNAL, curr_calibrate_num_);
        ui->pushButton_grap->setEnabled(true);
        if (err.ret != ERRTYPE::M_OK){
            QMessageBox::information(this, tr("Warn"), QString::fromStdString(err.err_str));
            return;
        }
        emit save_image_signal(curr_calibrate_num_, img);
        curr_calibrate_num_++;
      } else {
        camera_calibrate_.CalInternalParams(MODETYPE::MODE_INTERNAL);
        camera_calibrate_.SaveTolog();
        curr_calibrate_num_ = 0;
        img_calibrate_.clear();

        QMessageBox::information(this, tr("Tip"), tr("内参标定完成"));
      }
      ui->pushButton_grap->setEnabled(true);
    } else {
      std::string path = internal_calibrate_image_test_path_;
      std::vector<std::string> files;

      GetFolders(path, files);

      cv::Mat img;
      for (auto &file : files) {
        img = cv::imread(path + "/" + file);

        imgs.push_back(img);
      }

      camera_calibrate_.ProcessInternal(imgs, MODETYPE::MODE_INTERNAL);
      curr_calibrate_num_ = 0;
      QMessageBox::information(this, tr("Tip"), tr("internal params cal done and image saved"));
    }

  } else if (ui->radioButton_2->isChecked()) {

    timer_lidar_->stop();

    // anchors标定, 同时触发拍照和lidar测距
    cv::Mat img;
    if (!is_car_cam_opened_) {
      if (!this->cam_.isOpened()) {

        if (!this->cam_.isOpened()){
            auto camera_index = ui->cmb_camera_list->currentIndex();
            this->cam_.open(camera_index);
        }
      }

      this->cam_.read(curr_processing_img_);
      if (curr_processing_img_.empty()) {
        return;
      }
    } else {
      std::unique_lock<std::mutex> lock(mutex_);
      curr_processing_img_ = curr_img_.clone();
    }

    // lidar 测试
    if (serial_opened_) {
      LidarTest();

      float lidar = GetLidarValue(cur_lidar_info_);

      ui->te_lidar_value->setText(QString("%1").arg(lidar * 1000));
      emit lidar_run_signal(static_cast<double>(lidar * 1000));

    } else {
      QMessageBox::information(this, tr("Tip"),
                               tr("lidar port open failed"));
    }

  } else {
    // 瞄准模式
    // timer_cam_->stop();
    timer_lidar_->start(time_lidar_interval_);
  }
}

/**
 * bytesToHex
 * @brief bytesToHex
 * @param array
 * @return
 */
static QString bytesToHex(QByteArray array) {
  QString hex = array.toHex().toUpper();
  return hex.replace(QRegularExpression("(.{2})"), "\\1 ");
}

/** 发送数据 */
void MainWindow::writeData() {
  if (!serial->isOpen()) {
    return;
  }

  QString text = currentSettings.sendStringCache;
  //    if(DEBUG) // qDebug() << __func__ << ":" << text;
  if (text.length() != 0 && currentSettings.sendNewLineEnabled)
    text += "\r\n";

  QByteArray data = text.toLatin1();
  // // qDebug() << "writeData: " << bytesToHex(data);
  qint32 len = serial->write(data);
  // 更新显示长度
  if (len >= 0) {
    currentSettings.sendNum += len;
    //        updateUi(currentSettings);
  }
  //    if(DEBUG) // qDebug() <<"currentSettings.sendNum:" <<
  //    currentSettings.sendNum;
}

qint64 lastTimestamp = 0;
//! [6]
//! [7]
void MainWindow::readData() {
  // 读取数据
  QByteArray data = serial->readAll();
  QString str = QString::fromLatin1(data.data());
  QString hex = bytesToHex(data);
  QString newStr = currentSettings.isHexDisplay ? hex : str;
  //    ui->receive_textBrowser->moveCursor (QTextCursor::End);

  cur_lidar_info_ = newStr;

  // 更新显示长度
  qint32 len = data.length();
  // if(DEBUG) // qDebug() <<"len:" << len;
  if (len >= 0) {
    currentSettings.receiveNum += len;
    currentIndexChanged();
  }

  // 显示数据
  ui->lineEdit->setText(QString("%1").arg(cur_lidar_info_));

  float lidar = GetLidarValue(cur_lidar_info_);

  ui->te_lidar_value->setText(QString("%1").arg(lidar * 1000));
}

void MainWindow::LidarTest() {
  currentIndexChanged();
  writeData();
}

/**
 * @brief MainWindow::currentIndexChanged
 * @param idx
 * handle StopBitsBox dataBitsBox stopBitsBox parityBox flowControlBox
 */
void MainWindow::currentIndexChanged() {
  // // qDebug() << __func__;
  Settings old = currentSettings;
  updateSettings();
  Settings now = currentSettings;
  if (serial->isOpen())
    setParameter(serial, currentSettings);

  // 更新显示方式
  if (old.isHexDisplay != now.isHexDisplay) {
    // qDebug() << "is need hexDisplay:" << now.isHexDisplay;
  }

  // 输入框文字发生改变，更新缓存 解决第一次启动时缓存为空问题
  if (currentSettings.sendStringCache == "" ||
      QString::compare(old.sendCache, now.sendCache, Qt::CaseInsensitive) !=
          0) {
    currentSettings.sendStringCache = now.sendCache;
    /* 如果是16进制状态，需要将16进制数转换成字符串 */
    if (now.isHexSend) {
      currentSettings.sendStringCache = hexToQString(false, now.sendCache);
    }
  }

  // 更新发送方式
  if (old.isHexSend != now.isHexSend) {
    /* 需要将输入框内容有字体串改为16进制显示 */
    QString inputStr = singal_str;
    if (now.isHexSend) {
      /* 将原数据保存 */
      currentSettings.sendCache = stringToHex(false, inputStr);
      currentSettings.sendStringCache = inputStr;
    } else {
      /* 将数据保存 */
      QString tmp = hexToQString(false, inputStr);
      currentSettings.sendCache = tmp;
      currentSettings.sendStringCache = tmp;
    }
  }

  // if(DEBUG) // qDebug() << "sendStringCache:" << stringToHex(false,
  // currentSettings.sendStringCache);
}

void MainWindow::updateSettings() {
  if (serial->isOpen())
    currentSettings.stringStatus = "已打开";
  else
    currentSettings.stringStatus = "已关闭";

  currentSettings.name = ui->serialPortInfoListBox->currentText();

  // Baud Rat
  // standard baud rate
  currentSettings.baudRate = static_cast<QSerialPort::BaudRate>(9600);
  currentSettings.stringBaudRate = QString::number(currentSettings.baudRate);

  // Data bits
  currentSettings.dataBits = static_cast<QSerialPort::DataBits>(8);
  currentSettings.stringDataBits = QString::number(currentSettings.dataBits);

  // Parity
  currentSettings.parity = static_cast<QSerialPort::Parity>(0);
  currentSettings.stringParity = QString::number(currentSettings.parity);

  // Stop bits
  currentSettings.stopBits = static_cast<QSerialPort::StopBits>(1);
  currentSettings.stringStopBits = QString::number(currentSettings.stopBits);

  // Flow control
  currentSettings.flowControl = static_cast<QSerialPort::FlowControl>(
      QSerialPort::FlowControl::NoFlowControl);
  currentSettings.stringFlowControl =
      QString::number(currentSettings.flowControl);

  // Additional options
  currentSettings.localEchoEnabled =
      false; // ui->localEchoCheckBox->isChecked();

  // new line
  currentSettings.sendNewLineEnabled = true;

  currentSettings.isDtr = false;
  currentSettings.isRts = false;
  currentSettings.isHexDisplay = true;
  currentSettings.isHexSend = true;
  currentSettings.sendNewLineEnabled = true;
  currentSettings.isTimerSend = false;
  currentSettings.timerLength = 1000;
  currentSettings.sendCache = singal_str;
}

bool MainWindow::setParameter(QSerialPort *serial, Settings settings) {
  bool ret;
  Settings p = settings;
  if (serial->setBaudRate(p.baudRate) && serial->setDataBits(p.dataBits) &&
      serial->setParity(p.parity) && serial->setStopBits(p.stopBits) &&
      serial->setFlowControl(p.flowControl)) {
    ret = true;
  } else
    ret = false;
  return ret;
}

MainWindow::Settings MainWindow::doSettings(bool isWrite, Settings inSettings) {
  Settings in = inSettings;
  Settings out;
  QSettings settings("Yzs_think", "Application");
  if (isWrite) {
    settings.setValue("name", in.name);
    settings.setValue("baudRate", in.baudRate);
    settings.setValue("stringBaudRate", in.stringBaudRate);
    settings.setValue("dataBits", in.dataBits);
    settings.setValue("stringDataBits", in.stringDataBits);
    settings.setValue("parity", in.parity);
    settings.setValue("stringParity", in.stringParity);
    settings.setValue("stopBits", in.stopBits);
    settings.setValue("stringStopBits", in.stringStopBits);
    settings.setValue("flowControl", in.flowControl);
    settings.setValue("stringFlowControl", in.stringFlowControl);
    settings.setValue("sendNewLineEnabled", in.sendNewLineEnabled);
    settings.setValue("stringStatus", in.stringStatus);
    settings.setValue("isDtr", in.isDtr);
    settings.setValue("isRts", in.isRts);
    settings.setValue("isHexDisplay", in.isHexDisplay);
    settings.setValue("isHexSend", in.isHexSend);
    // 和Windows版本同步，不保存定时发送开关
    settings.setValue("isTimerSend", DEF_SETTINGS.isTimerSend);
    settings.setValue("timeTimerSend", in.timerLength);
    settings.setValue("sendCache", in.sendCache);
  } else {
    out.name = settings.value("name", DEF_SETTINGS.name).toString();
    out.baudRate =
        (QSerialPort::BaudRate)settings.value("baudRate", DEF_SETTINGS.baudRate)
            .toInt();
    out.stringBaudRate =
        settings.value("stringBaudRate", DEF_SETTINGS.stringBaudRate)
            .toString();
    out.dataBits =
        (QSerialPort::DataBits)settings.value("dataBits", DEF_SETTINGS.dataBits)
            .toInt();
    out.stringDataBits =
        settings.value("stringDataBits", DEF_SETTINGS.stringDataBits)
            .toString();
    out.parity =
        (QSerialPort::Parity)settings.value("parity", DEF_SETTINGS.parity)
            .toInt();
    out.stringParity =
        settings.value("stringParity", DEF_SETTINGS.stringParity).toString();
    out.stopBits =
        (QSerialPort::StopBits)settings.value("stopBits", DEF_SETTINGS.stopBits)
            .toInt();
    out.stringStopBits =
        settings.value("stringStopBits", DEF_SETTINGS.stringStopBits)
            .toString();
    out.flowControl = (QSerialPort::FlowControl)settings
                          .value("flowControl", DEF_SETTINGS.flowControl)
                          .toInt();
    out.stringFlowControl =
        settings.value("stringFlowControl", DEF_SETTINGS.stringFlowControl)
            .toString();
    out.sendNewLineEnabled =
        settings.value("sendNewLineEnabled", DEF_SETTINGS.sendNewLineEnabled)
            .toBool();
    out.stringStatus =
        settings.value("stringStatus", DEF_SETTINGS.stringStatus).toString();
    out.isDtr = settings.value("isDtr", DEF_SETTINGS.isDtr).toBool();
    out.isRts = settings.value("isRts", DEF_SETTINGS.isRts).toBool();
    out.isHexDisplay =
        settings.value("isHexDisplay", DEF_SETTINGS.isHexDisplay).toBool();
    out.isHexSend =
        settings.value("isHexSend", DEF_SETTINGS.isHexSend).toBool();
    out.isTimerSend =
        settings.value("isTimerSend", DEF_SETTINGS.isTimerSend).toBool();
    out.timerLength =
        settings.value("timeTimerSend", DEF_SETTINGS.timerLength).toInt();
    out.sendCache =
        settings.value("sendCache", DEF_SETTINGS.sendCache).toString();
    out.sendNum = 0;
    out.receiveNum = 0;
  }

  return out;
}

//! [8]
//! 添加除错功能，解决强制拔出时程序崩溃。
void MainWindow::handleError(QSerialPort::SerialPortError error) {
  QString errStr = "NO ERROR";
  switch (error) {
  case QSerialPort::DeviceNotFoundError:
    errStr = "DeviceNotFoundError";
    // qDebug() << errStr;
    closeSerialPort();
    break;
  case QSerialPort::PermissionError:
    errStr = "PermissionError";
    // qDebug() << errStr;
    closeSerialPort();
    break;
  case QSerialPort::OpenError:
    errStr = "OpenError";
    // qDebug() << errStr;
    closeSerialPort();
    break;
  //    case QSerialPort::ParityError:
  //        errStr = "ParityError";
  //        // qDebug() << errStr;
  //        closeSerialPort();
  //        break;
  //    case QSerialPort::FramingError:
  //        errStr = "FramingError";
  //        // qDebug() << errStr;
  //        closeSerialPort();
  //        break;
  //    case QSerialPort::BreakConditionError:
  //        errStr = "BreakConditionError";
  //        // qDebug() << errStr;
  //        closeSerialPort();
  //        break;
  //    case QSerialPort::WriteError:
  //        errStr = "WriteError";
  //        // qDebug() << errStr;
  //        closeSerialPort();
  //        break;
  case QSerialPort::ReadError:
    errStr = "ReadError";
    closeSerialPort();
    break;
  case QSerialPort::ResourceError:
    errStr = "ResourceError";
    // qDebug() << errStr;
    // on_openserial_pushButton_pressed();
    break;
  case QSerialPort::UnsupportedOperationError:
    errStr = "UnsupportedOperationError";
    // qDebug() << errStr;
    closeSerialPort();
    break;
  case QSerialPort::UnknownError:
    // do nothing!!!
    errStr = "UnknownError";
    // qDebug() << errStr;
    //        on_openserial_pushButton_pressed();
    break;
  case QSerialPort::TimeoutError:
    errStr = "TimeoutError";
    // qDebug() << errStr;
    closeSerialPort();
    break;
  case QSerialPort::NotOpenError:
    // do nothing.
    errStr = "NotOpenError";
    // qDebug() << errStr;
    //        closeSerialPort();
    break;
  default:
    // qDebug() << errStr;
    break;
  }
  if (error == QSerialPort::ResourceError) {
    QMessageBox::critical(this, tr("Critical Error"), serial->errorString());
    // closeSerialPort();
  }
}

bool MainWindow::openSerialPort() {
  bool ret = false;
  Settings p = currentSettings;
  serial->setPortName(p.name);
  if (serial->open(QIODevice::ReadWrite)) {
    if (setParameter(serial, p)) {
      ui->bt_open_serial->setText("关闭串口");
      ret = true;
      serial_opened_ = !serial_opened_;
    } else {
      serial->close();
      serial_opened_ = false;
      QMessageBox::critical(this, tr("Error"), serial->errorString());
    }
  } else {
    QMessageBox::critical(this, tr("Error"), serial->errorString());
  }

  return ret;
}

void MainWindow::closeSerialPort() {
  ui->bt_open_serial->setText("打开串口");
  serial->close();
}

void MainWindow::on_bt_open_serial_clicked() {
  if (serial->isOpen())
    closeSerialPort();
  else
    openSerialPort();
  currentIndexChanged();
}

void MainWindow::fillPortsInfo() {
  ui->serialPortInfoListBox->clear();
  static const QString blankString = QObject::tr("N/A");
  QString description;
  QString manufacturer;
  foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
    QStringList list;
    description = info.description();
    manufacturer = info.manufacturer();
    list << info.portName()
         << (!description.isEmpty() ? description : blankString)
         << (!manufacturer.isEmpty() ? manufacturer : blankString)
         << info.systemLocation()
         << (info.vendorIdentifier()
                 ? QString::number(info.vendorIdentifier(), 16)
                 : blankString)
         << (info.productIdentifier()
                 ? QString::number(info.productIdentifier(), 16)
                 : blankString);

    ui->serialPortInfoListBox->addItem(list.first(), list);
  }
}

void MainWindow::on_serialPortInfoListBox_currentIndexChanged(int index) {
  auto cur_index = ui->serialPortInfoListBox->currentIndex();
  auto cur_text = ui->serialPortInfoListBox->currentText();

  if (index == cur_index) {
    // if (DEBUG)
    //   // qDebug() << "cur_index:" << cur_index;
    //   if (DEBUG)
    // qDebug() << "serial text" << cur_text;
  }

  if (serial->isOpen())
    closeSerialPort();
  updateSettings();
}

bool SortName(const QCameraInfo &name1, const QCameraInfo &name2) {
  return QString::compare(name1.deviceName(), name2.deviceName(),
                          Qt::CaseInsensitive);
}

//获取摄像头信息
void MainWindow::getCamera() {
  std::vector<std::string> camera_names;
  std::vector<std::string> device_name;
  std::vector<QCamera::Position> device_pos;
  int count = 0;
  auto camera_infos = QCameraInfo::availableCameras();

  qSort(camera_infos.begin(), camera_infos.end(), SortName);

  foreach (const QCameraInfo &cameraInfo, camera_infos) {
    camera_names.push_back(cameraInfo.description().toStdString());
    device_name.push_back(cameraInfo.deviceName().toStdString());
    device_pos.push_back(cameraInfo.position());
    cam_map_[count++] = cameraInfo.description();
  }

  auto device_name_tmp = device_name;
  std::sort(device_name_tmp.begin(), device_name_tmp.end());

  for (const auto &camera_name : camera_names) {
    ui->cmb_camera_list->addItem(QString::fromStdString(camera_name));
  }
}

void MainWindow::update_chess_board_config(const QString &qstr) {
  auto chess_board_size = ui->lineEdit_2->selectedText();
  auto chess_board_rows = ui->lineEdit_3->selectedText();
  auto chess_board_cols = ui->lineEdit_4->selectedText();

  float board_size = chess_board_size.toDouble();
  int board_rows = chess_board_rows.toInt();
  int board_cols = chess_board_cols.toInt();

  camera_calibrate_.SetBoardSize(board_size, board_rows, board_cols);
}

void MainWindow::run_lidar(double lidar_val) {

  if (!curr_processing_img_.empty()) {

    // 设置标定参数
//    QString camera_params_file = ui->lineEdit_5->text();
//    auto err = camera_calibrate_.GetCameraInternalParams(
//        camera_params_file.toStdString());

    cv::Point3f pt;
    auto ret = camera_calibrate_.CalAnchors(
        curr_processing_img_, MODETYPE::MODE_ANCHORS, lidar_val, pt);

    if (ret.ret == ERRTYPE::M_OK) {
      ui->lineEdit_8->setText(QString::number(pt.x, 10, 5));
      ui->lineEdit_9->setText(QString::number(pt.y, 10, 5));
      ui->lineEdit_10->setText(QString::number(pt.z, 10, 5));

      // 设置tables数据
      auto q_index = ui->lineEdit_11->text();
      auto index = q_index.toInt();

      QString q_str = QString::number(pt.x, 10, 5) + "," +
                      QString::number(pt.y, 10, 5) + "," +
                      QString::number(pt.z, 10, 5);
      m_model->setItem(index, 1, new QStandardItem(q_str));

      QString pic_save_path;
      pic_save_path.sprintf("pic/%d_%lf_%f-%f-%f.jpg", index, lidar_val, pt.x, pt.y, pt.z);
      cv::imwrite(pic_save_path.toStdString(), curr_processing_img_);

      ui->lineEdit_11->setText(QString::number(index + 1));
    } else {
      QMessageBox::warning(this, tr("warn"), QString::fromStdString(ret.err_str));
    }

  } else {
    QMessageBox::warning(this, tr("warn"), tr("get camera image failed！"));
  }
}

void MainWindow::update_camera_video() {

  if (cam_.isOpened()) {
    cv::Mat img;
    cam_.read(img);

    if (img.empty()) {
      return;
    }

//    cv::imwrite(std::to_string(frame_count_++) + ".jpg", img);

    auto h = ui->label->size().height();
    auto w = ui->label->size().width();
//    qDebug() << "img_info: " << h << ", " << w << "," << img.channels();
    cv::resize(img, img, cv::Size(w, h));
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    QImage qimg = QImage((uchar *)img.data, img.cols, img.rows, img.step,
                         QImage::Format::Format_RGB888);

    ui->label->setPixmap(QPixmap::fromImage(qimg));
  }
}

void MainWindow::grap_image(int index) {
  if (cam_.isOpened()) {
    cv::Mat img;
    cam_.read(img);

    if (img.empty()) {
      QMessageBox::warning(NULL, "warning", "please retry",
                           QMessageBox::Yes | QMessageBox::No,
                           QMessageBox::Yes);
      ui->pushButton_grap->setEnabled(true);
      return;
    }

//    img_calibrate_.push_back(img);

    emit save_image_signal(index, img);

  } else if (is_car_cam_opened_) {

    if (curr_img_.empty()) {
      QMessageBox::warning(NULL, "warning", "get car camera image failed, please retry",
                           QMessageBox::Yes | QMessageBox::No,
                           QMessageBox::Yes);
      ui->pushButton_grap->setEnabled(true);
      return;
    }

//    img_calibrate_.push_back(curr_img_);

    emit save_image_signal(index, curr_img_);
  }
}

void MainWindow::save_image(int index, const cv::Mat &img) {

  QDateTime current_date_time = QDateTime::currentDateTime();
  QString current_date = current_date_time.toString("yyyyMMdd_hh_mm_ss");

  std::string str_name = current_date.toStdString();

  std::string str_index = std::to_string(index);

  str_index = str_index.length() < 2 ? "0" + str_index : str_index;

  str_name = path_calibrate_images_ + "/" + "img_" + str_name + "_" +
             str_index + ".jpg";

  bool ret = false;
  if (!is_car_cam_opened_) {
    ret = cv::imwrite(str_name, img);
  } else {
    std::unique_lock<std::mutex> lock(mutex_);
    ret = cv::imwrite(str_name, curr_img_);
  }

  if (ret) {
    curr_calibrate_num_++;
    std::string str = "image " + std::to_string(index) + " save success";
    QMessageBox::information(NULL, tr("info"), QString::fromStdString(str));
  } else {
    QString qstr = QString::number(index);
    std::string str = "image " + std::to_string(index) + " save failed";
    QMessageBox::information(NULL, tr("info"), QString::fromStdString(str));

  }
  ui->pushButton_grap->setEnabled(true);
}

void MainWindow::change_camera_index(int index) {
  this->timer_cam_->stop();
  if (this->cam_.isOpened()) {
    this->cam_.release();
  }
  this->cam_.open(index);

  this->timer_cam_->start(time_cam_interval_);
}

void Stringsplit(const std::string &str, const std::string &splits,
                 std::vector<std::string> &res) {
  if (str == "")
    return;
  std::string strs = str + splits;
  size_t pos = strs.find(splits);
  int step = splits.size();

  while (pos != strs.npos) {
    std::string temp = strs.substr(0, pos);
    res.push_back(temp);
    strs = strs.substr(pos + step, strs.size());
    pos = strs.find(splits);
  }
  return;
}

void MainWindow::save_anchors_to_json_file() {
  std::ofstream fout;
  fout.open(path_anchors_result_json_);

  MJson json;
  // 获取 tableview
  auto rows = m_model->rowCount();

  MJson::object obj;

  for (int i = 0; i < rows; i++) {
    QString data = m_model->data(m_model->index(i, 1)).toString();

    std::string str = data.toStdString();

    std::vector<std::string> str_vecs;

    Stringsplit(str, ",", str_vecs);

    if (str_vecs.size() == 3) {
      MJson::object temp;
      temp["x"] = MJson(stof(str_vecs[0]));
      temp["y"] = MJson(stof(str_vecs[1]));
      temp["z"] = MJson(stof(str_vecs[2]));

      obj[std::to_string(i)] = temp;
    }
  }

  std::string json_str = MJson(obj).dump();

  fout << json_str << std::endl;

  fout.close();

  std::string file_save = "result saved in ：" + path_anchors_result_json_;
  QMessageBox::information(this, tr("Done"), QString::fromStdString(file_save));
}

void MainWindow::save_anchors_to_txt_file() {
  std::ofstream fout;
  fout.open(path_anchors_result_txt_);

  fout << "//x y z" << std::endl;

  // 获取 tableview
  auto rows = m_model->rowCount();

  for (int i = 0; i < rows; i++) {
    QString data = m_model->data(m_model->index(i, 1)).toString();

    std::string str = data.toStdString();
    const char x = ',';
    const char y = ' ';

    std::replace(str.begin(), str.end(), x, y);
    fout << str << std::endl;
  }
  fout.close();

  std::string file_save = "result saved in ：" + path_anchors_result_txt_;
  QMessageBox::information(this, tr("Done"), QString::fromStdString(file_save));
}

void MainWindow::open_car_camera() {

  if (this->cam_.isOpened()) {
    this->cam_.release();
  }

  QString qip = ui->lineEdit_car_ip->text();
  QString qport = ui->lineEdit_car_port->text();

  m_ip = qip.toStdString();
  m_port = qport.toStdString();
  qDebug() << "ip: " << m_ip.c_str();
  qDebug() << "port: " << m_port.c_str();

  emit open_car_cam_signal();
}

void MainWindow::handle_connect() {
  if (!m_car_cam_recv.is_opened() && m_origin_frame->is_opened()) {
    // stop();
  }

  auto future_result = start_detached_future<bool>(
      [this]() { return m_car_cam_recv.open(m_ip, m_port); });
  if (std::future_status::timeout ==
      future_result.wait_for(std::chrono::seconds(1))) {
    QMessageBox messageBox;
    messageBox.warning(0, "Error", "Unable to find dest IP:PORT");
    messageBox.setFixedSize(500, 200);
    return;
  }

  if (!future_result.get()) {
    QMessageBox messageBox;
    messageBox.warning(0, "Error", "Camera Not Open");
    messageBox.setFixedSize(500, 200);
    m_origin_frame->set_close();
  } else {
    std::cout << "ip cam connect success" << std::endl;
    m_origin_frame->set_open();
    timer_cam_->stop();
    timer_car_cam_->start(50);
    is_car_cam_opened_ = true;

    std::thread recv_th(&MainWindow::recv_data, this, &m_car_cam_recv);
    recv_th.detach();
    std::thread data_th(&MainWindow::update_window, this);
    data_th.detach();
  }
}

void MainWindow::recv_data(car_cam_recv_t *car_recv) {

    if (nullptr == car_recv) {
        qDebug() << "Recv data is invalid";
    } else {
        car_recv->recv_img();
    }
}

void MainWindow::update_window() {
    auto height = ui->label->size().height();
    auto width = ui->label->size().width();
    update_car_cam_data = true;
    while (update_car_cam_data) {
        cv::Mat cv_img; curr_img_;
        if (!m_car_cam_recv.get_one_pic(cv_img)) {
            QThread::msleep(33);
            continue;
        }
        if (!cv_img.empty()) {
            cv::cvtColor(cv_img, cv_img, cv::COLOR_BGR2RGB);
            curr_img_ = cv_img.clone();
            cv::resize(cv_img, cv_img, cv::Size(width, height));
            QImage qimg = QImage((uchar *)cv_img.data, cv_img.cols, cv_img.rows, cv_img.step,
                                 QImage::Format::Format_RGB888);
            ui->label->setPixmap(QPixmap::fromImage(qimg));
        }
    }
}

void MainWindow::select_file_from_folder_for_camera_params() {
  QString fileName = QFileDialog::getOpenFileName(
      this, tr("open a file."), ".", tr("yaml(*.yaml);All files(*.*)"));

  if (fileName.isEmpty()) {
    QMessageBox::warning(this, "Warning!", "Failed to open file!");
  } else {
    ui->lineEdit_5->setText(fileName);
  }
}

void MainWindow::select_file_from_folder_for_lidar_params() {
  QString fileName = QFileDialog::getOpenFileName(
      this, tr("open a file."), ".", tr("yaml(*.yaml);All files(*.*)"));

  if (fileName.isEmpty()) {
    QMessageBox::warning(this, "Warning!", "Failed to open the file!");
  } else {
    ui->lineEdit_6->setText(fileName);
  }
}

void MainWindow::select_file_from_folder_for_anchors() {
  QString fileName = QFileDialog::getOpenFileName(
      this, tr("open a file."), ".", tr("json(*.yaml); All files(*.*)"));

  if (fileName.isEmpty()) {
    QMessageBox::warning(this, "Warning!", "Failed to open the file!");
  } else {
    ui->lineEdit_7->setText(fileName);
  }
}

void MainWindow::update_camera_params(const QString &qstr) {
  std::string str = qstr.toStdString();
  camera_calibrate_.GetCameraInternalParams(str);
}

void MainWindow::update_lidar_params(const QString &qstr) {
  std::string str = qstr.toStdString();
  camera_calibrate_.GetLidarParamsFromFile(str);
}

void MainWindow::update_anchors(const QString &qstr) {
  std::string file = qstr.toStdString();

  std::ifstream is(file);

  const std::istreambuf_iterator<char> eos;
  const std::string json_str(std::istreambuf_iterator<char>(is), eos);

  std::string error;
  MJson root = MJson::parse(json_str, error);

  std::map<int, std::string> anchors_results_map;

  auto objects = root.object_items();

  for (auto &item : objects) {
    if (item.first == "frame_idx")
      continue;
    // NUL, NUMBER, BOOL, STRING, ARRAY, OBJECT
    {
      double x{0.0}, y{0.0}, z{0.0};
      if (item.second["x"].type() == MJson::NUMBER) {
        x = item.second["x"].number_value();
        y = item.second["y"].number_value();
        z = item.second["z"].number_value();
      } else if (item.second["x"].type() == MJson::STRING) {
        x = std::stod(item.second["x"].string_value());
        y = std::stod(item.second["y"].string_value());
        z = std::stod(item.second["z"].string_value());
      }
      std::string str_val;
      str_val += (std::to_string(x) + ",");
      str_val += (std::to_string(x) + ",");
      str_val += (std::to_string(x));

      anchors_results_map[std::stoi(item.first)] = str_val;
    }
  }

  for (int i = 0; i < 48; i++) {
    m_model->setItem(i, 1, new QStandardItem(
                               QString::fromStdString(anchors_results_map[i])));
  }
}
