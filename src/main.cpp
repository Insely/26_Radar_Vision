#include "camera/camera.hpp"
#include "detector/detector.hpp"
#include "serial/serial.hpp"
#include <iostream>

#define DEBUG_MODE

int main() {
  // 串口初始化
  SerialPort serial("/dev/ttyUSB0", 115200);
  serial.init();

  // 改用工业相机
  // 读取setting.yaml
  cv::FileStorage sysfs;
  bool use_demo = false, enable_ui = true;
  int camera_fps = 120;
  if (sysfs.open("../configs/setting.yaml", cv::FileStorage::READ)) {
    if (!sysfs["UseDemoVideo"].empty()) sysfs["UseDemoVideo"] >> use_demo;
    if (!sysfs["EnableUI"].empty()) sysfs["EnableUI"] >> enable_ui;
    if (!sysfs["CameraFPS"].empty()) sysfs["CameraFPS"] >> camera_fps;
  } else {
    std::cerr << "Warning: setting.yaml not found, using defaults" << std::endl;
  }

  Camera camera;
  bool use_camera = !use_demo && camera.init("../configs/settings.yaml");
  cv::VideoCapture video_cap;
  if (!use_camera) {
    std::cerr << "Camera Init Failed! Falling back to demo.mp4" << std::endl;
    video_cap.open("../demo.mp4");
    if (!video_cap.isOpened()) {
      std::cerr << "Failed to open demo.mp4!" << std::endl;
      return -1;
    }
  }

  Detector detector; // 实例化检测器
  detector.init("../configs/detector.yaml");
  // 创建展示窗口
  if (enable_ui) {
    cv::namedWindow("demo", cv::WINDOW_NORMAL);
  #ifdef DEBUG_MODE
    cv::namedWindow("Grayscale", cv::WINDOW_NORMAL);
    cv::namedWindow("Binarized", cv::WINDOW_NORMAL);
  #endif
    cv::resizeWindow("demo", 800, 600);
  #ifdef DEBUG_MODE
    cv::resizeWindow("Grayscale", 800, 600);
    cv::resizeWindow("Binarized", 800, 600);
  #endif
  }

  while (true) {
    cv::Mat frame;
    if (use_camera) {
      if (!camera.getFrame(frame)) {
        continue;
      }
    } else {
      video_cap >> frame;
      if (frame.empty()) {
        // 视频播放完毕，循环播放
        video_cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        continue;
      }
    }

    // --- 核心算法处理 ---
    DetectionResult result = detector.process(frame);

    //  通信
    if (result.is_locked) {
      VisionData packet;

      // 1. 计算水平偏差
      packet.yaw_error = (int)result.error_x;

      // 2. 目标是否到达中心 (设置 5 像素的死区)
      packet.at_center = (std::abs(packet.yaw_error) < 10.0f) ? 1 : 0;

      // 2. 合理的开火判定逻辑：
      // 只有当目标稳定锁定（is_locked为真）且物理对准中心（at_center为真）时，才允许发射
      if (packet.at_center == 1) {
        printf("允许发射\n");
        packet.allow_fire = 1; // 允许发射
      } else {
        packet.allow_fire = 0; // 即使锁定了，如果没对准中心，也不准发射
      }

      // 发送数据
      serial.send(packet);
      std::cout << "Data Sent: Yaw=" << packet.yaw_error << std::endl;
    } else {
      // 目标丢失，也发一个空包告知电控
      VisionData lost_packet;
      lost_packet.yaw_error = 0;
      lost_packet.at_center = 0;
      lost_packet.allow_fire = 0;
      serial.send(lost_packet);
    }

    // --- 绘图与显示逻辑 (保持在 main 方便观察) ---
    cv::Mat display_frame =
        frame.clone(); // 深拷贝，防止原图像被污染，影响下一轮的图像识别

    if (result.is_locked) {
      // 绘制四个角点
      for (int i = 0; i < 4; i++) {
        cv::circle(display_frame, result.corners[i], 5,
                   cv::Scalar(0, 255, 255), -1);
      }
      // 绘制目标矩形
      for (int i = 0; i < 4; i++) {
        cv::line(display_frame, result.corners[i],
                 result.corners[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
      }
      // 绘制中心点
      cv::circle(display_frame, result.center, 8, cv::Scalar(0, 0, 255), -1);
      std::string info = "LOCKED | X_Err: " +
                         std::to_string((int)result.error_x);
      cv::putText(display_frame, info, cv::Point(30, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
    } else {
      cv::putText(display_frame, "SEARCHING...", cv::Point(30, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
    }

    if (enable_ui) {
      cv::imshow("demo", display_frame);
  #ifdef DEBUG_MODE
      cv::imshow("Grayscale", detector.getGray());
      cv::imshow("Binarized", detector.getMask());
  #endif
      int key = cv::waitKey(1000 / camera_fps);
      if (key == 27) break;
      if (key == 's') {
        // 保存逻辑...
      }
    }
  }

  return 0;
}