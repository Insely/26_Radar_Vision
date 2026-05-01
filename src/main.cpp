#include "camera/camera.hpp"
#include "detector/detector.hpp"
#include "serial/serial.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
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
cv::Mat shared_mask;
bool result_ready = false;

std::atomic<bool> running{true};
int g_demo_frame_delay_ms = 0; // demo视频帧间隔(ms)，0=不限速

// ============ 取帧线程 ============
void captureThread(Camera &camera, cv::VideoCapture &video_cap,
                   bool use_camera) {
  while (running) {
    // demo模式：等上一帧被消费后再取下一帧，避免跳帧
    if (!use_camera) {
      {
        std::lock_guard<std::mutex> lock(frame_mtx);
        if (frame_ready)
          continue; // 上一帧还没被消费，等一等
      }
      if (g_demo_frame_delay_ms > 0) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(g_demo_frame_delay_ms));
      }
    }

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
      packet.pitch_error = (int)result.error_y;
      packet.at_center = (std::abs(packet.yaw_error) < 10.0f &&
                          std::abs(packet.pitch_error) < 10.0f) ? 1 : 0;
      if (packet.at_center == 1) {
        packet.allow_fire = 1;
      } else {
        packet.allow_fire = 0;
      }
      serial.send(packet);
    } else {
      VisionData lost_packet;
      lost_packet.yaw_error = 0;
      lost_packet.pitch_error = 0;
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
          "LOCKED | Yaw: " + std::to_string((int)result.error_x) +
          " Pitch: " + std::to_string((int)result.error_y);
      cv::putText(display, info, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX,
                  0.8, cv::Scalar(0, 255, 0), 2);
    } else {
      cv::putText(display, "SEARCHING...", cv::Point(30, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
    }

    {
      std::lock_guard<std::mutex> lock(result_mtx);
      shared_display_frame = display;
      shared_mask = detector.getMask().clone();
      shared_result = result;
      result_ready = true;
    }
  }
}

int main() {
  cv::FileStorage sysfs;
  bool use_demo = false, enable_ui = true;
  int camera_fps = 120;
  int demo_fps_cfg = 0; // 0=自动读取视频原始帧率
  int display_wait_ms = 1000 / camera_fps;
  std::string serial_port = "/dev/ttyUSB0";
  int serial_baud = 115200;
  if (sysfs.open("../configs/setting.yaml", cv::FileStorage::READ)) {
    if (!sysfs["UseDemoVideo"].empty())
      sysfs["UseDemoVideo"] >> use_demo;
    if (!sysfs["EnableUI"].empty())
      sysfs["EnableUI"] >> enable_ui;
    if (!sysfs["CameraFPS"].empty())
      sysfs["CameraFPS"] >> camera_fps;
    if (!sysfs["DemoFPS"].empty())
      sysfs["DemoFPS"] >> demo_fps_cfg;
    if (!sysfs["SerialPort"].empty())
      sysfs["SerialPort"] >> serial_port;
    if (!sysfs["SerialBaud"].empty())
      sysfs["SerialBaud"] >> serial_baud;
  } else {
    std::cerr << "Warning: setting.yaml not found, using defaults"
              << std::endl;
  }

  // 串口初始化
  SerialPort serial(serial_port, serial_baud);
  serial.init();

  Camera camera;
  bool use_camera =
      !use_demo && camera.init("../configs/Camera.yaml", camera_fps);
  cv::VideoCapture video_cap;
  if (!use_camera) {
    std::cerr << "Using demo.mp4" << std::endl;
    video_cap.open("../demo.mp4");
    if (!video_cap.isOpened()) {
      std::cerr << "Failed to open demo.mp4!" << std::endl;
      return -1;
    }

    const double demo_fps = (demo_fps_cfg > 0) ? demo_fps_cfg : video_cap.get(cv::CAP_PROP_FPS);
    if (demo_fps > 0.0) {
      g_demo_frame_delay_ms = std::max(1, static_cast<int>(1000.0 / demo_fps));
      display_wait_ms = 1;
      std::cout << "Demo playback FPS: " << demo_fps << std::endl;
    } else {
      g_demo_frame_delay_ms = std::max(1, 1000 / camera_fps);
      display_wait_ms = 1;
      std::cerr << "Warning: failed to read demo FPS, using CameraFPS"
                << std::endl;
    }
  } else {
    display_wait_ms = std::max(1, 1000 / camera_fps);
  }

  Detector detector;
  detector.init("../configs/detector.yaml");

  if (enable_ui) {
    cv::namedWindow("demo", cv::WINDOW_NORMAL);
    cv::namedWindow("Binarized", cv::WINDOW_NORMAL);
    cv::resizeWindow("demo", 800, 600);
    cv::resizeWindow("Binarized", 800, 600);
  }

  // 启动取帧线程和检测线程
  std::thread cap_thread(captureThread, std::ref(camera), std::ref(video_cap),
                         use_camera);
  std::thread det_thread(detectThread, std::ref(detector), std::ref(serial));

  // 主线程负责 UI 显示
  auto fps_start = std::chrono::steady_clock::now();
  int fps_count = 0;
  double fps_value = 0.0;

  while (running) {
    if (enable_ui) {
      cv::Mat display, mask;
      {
        std::lock_guard<std::mutex> lock(result_mtx);
        if (result_ready) {
          display = shared_display_frame.clone();
          mask = shared_mask.clone();
          result_ready = false;
        }
      }
      if (!display.empty()) {
        // 计算 FPS
        fps_count++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_start).count();
        if (elapsed >= 1.0) {
          fps_value = fps_count / elapsed;
          fps_count = 0;
          fps_start = now;
        }
        std::string fps_text = "FPS: " + std::to_string((int)fps_value);
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(fps_text, cv::FONT_HERSHEY_SIMPLEX,
                                             0.7, 2, &baseline);
        cv::putText(display, fps_text,
                    cv::Point(display.cols - text_size.width - 15, 35),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        cv::imshow("demo", display);
      }
      if (!mask.empty()) {
        cv::imshow("Binarized", mask);
      }
      int key = cv::waitKey(display_wait_ms);
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
