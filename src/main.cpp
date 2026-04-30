#include "camera/camera.hpp"
#include "detector/detector.hpp"
#include "serial/serial.hpp"
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

// ============ 共享数据 ============
std::mutex frame_mtx;
cv::Mat shared_frame;
bool frame_ready = false;

std::mutex result_mtx;
DetectionResult shared_result;
cv::Mat shared_display_frame;
bool result_ready = false;

std::atomic<bool> running{true};

// ============ 取帧线程 ============
void captureThread(Camera &camera, cv::VideoCapture &video_cap,
                   bool use_camera) {
  while (running) {
    cv::Mat frame;
    if (use_camera) {
      if (!camera.getFrame(frame))
        continue;
    } else {
      video_cap >> frame;
      if (frame.empty()) {
        video_cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        continue;
      }
    }
    {
      std::lock_guard<std::mutex> lock(frame_mtx);
      shared_frame = frame;
      frame_ready = true;
    }
  }
}

// ============ 检测+通信线程 ============
void detectThread(Detector &detector, SerialPort &serial) {
  while (running) {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(frame_mtx);
      if (!frame_ready)
        continue;
      frame = shared_frame.clone();
      frame_ready = false;
    }

    // 检测
    DetectionResult result = detector.process(frame);

    // 串口通信
    if (result.is_locked) {
      VisionData packet;
      packet.yaw_error = (int)result.error_x;
      packet.at_center = (std::abs(packet.yaw_error) < 10.0f) ? 1 : 0;
      if (packet.at_center == 1) {
        packet.allow_fire = 1;
      } else {
        packet.allow_fire = 0;
      }
      serial.send(packet);
    } else {
      VisionData lost_packet;
      lost_packet.yaw_error = 0;
      lost_packet.at_center = 0;
      lost_packet.allow_fire = 0;
      serial.send(lost_packet);
    }

    // 绘图（准备给主线程显示）
    cv::Mat display = frame.clone();
    if (result.is_locked) {
      for (int i = 0; i < 4; i++) {
        cv::circle(display, result.corners[i], 5, cv::Scalar(0, 255, 255), -1);
        cv::line(display, result.corners[i], result.corners[(i + 1) % 4],
                 cv::Scalar(0, 255, 0), 2);
      }
      cv::circle(display, result.center, 8, cv::Scalar(0, 0, 255), -1);
      std::string info =
          "LOCKED | X_Err: " + std::to_string((int)result.error_x);
      cv::putText(display, info, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX,
                  1, cv::Scalar(0, 255, 0), 2);
    } else {
      cv::putText(display, "SEARCHING...", cv::Point(30, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
    }

    {
      std::lock_guard<std::mutex> lock(result_mtx);
      shared_display_frame = display;
      shared_result = result;
      result_ready = true;
    }
  }
}

int main() {
  // 串口初始化
  SerialPort serial("/dev/ttyUSB0", 115200);
  serial.init();

  // 读取setting.yaml
  cv::FileStorage sysfs;
  bool use_demo = false, enable_ui = true;
  int camera_fps = 120;
  if (sysfs.open("../configs/setting.yaml", cv::FileStorage::READ)) {
    if (!sysfs["UseDemoVideo"].empty())
      sysfs["UseDemoVideo"] >> use_demo;
    if (!sysfs["EnableUI"].empty())
      sysfs["EnableUI"] >> enable_ui;
    if (!sysfs["CameraFPS"].empty())
      sysfs["CameraFPS"] >> camera_fps;
  } else {
    std::cerr << "Warning: setting.yaml not found, using defaults"
              << std::endl;
  }

  Camera camera;
  bool use_camera = !use_demo && camera.init("../configs/settings.yaml");
  cv::VideoCapture video_cap;
  if (!use_camera) {
    std::cerr << "Using demo.mp4" << std::endl;
    video_cap.open("../demo.mp4");
    if (!video_cap.isOpened()) {
      std::cerr << "Failed to open demo.mp4!" << std::endl;
      return -1;
    }
  }

  Detector detector;
  detector.init("../configs/detector.yaml");

  if (enable_ui) {
    cv::namedWindow("demo", cv::WINDOW_NORMAL);
    cv::resizeWindow("demo", 800, 600);
  }

  // 启动取帧线程和检测线程
  std::thread cap_thread(captureThread, std::ref(camera), std::ref(video_cap),
                         use_camera);
  std::thread det_thread(detectThread, std::ref(detector), std::ref(serial));

  // 主线程负责 UI 显示
  while (running) {
    if (enable_ui) {
      cv::Mat display;
      {
        std::lock_guard<std::mutex> lock(result_mtx);
        if (result_ready) {
          display = shared_display_frame.clone();
          result_ready = false;
        }
      }
      if (!display.empty()) {
        cv::imshow("demo", display);
      }
      int key = cv::waitKey(1000 / camera_fps);
      if (key == 27) {
        running = false;
        break;
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // 等待线程结束
  cap_thread.join();
  det_thread.join();

  return 0;
}