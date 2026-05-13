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

std::mutex frame_mtx;
std::condition_variable frame_cv;
cv::Mat shared_frame;
bool frame_ready = false;

std::mutex result_mtx;
std::condition_variable result_cv;
DetectionResult shared_result;
cv::Mat shared_display_frame;
cv::Mat shared_mask;
cv::Mat shared_roi_display;
cv::Mat shared_gray;
bool result_ready = false;

std::atomic<bool> running{true};
int g_demo_frame_delay_ms = 0;
bool g_enable_ui = true;
bool g_show_camera = true;

void captureThread(Camera &camera, cv::VideoCapture &video_cap,
                   bool use_camera) {
  while (running) {
    if (!use_camera) {
      std::unique_lock<std::mutex> lock(frame_mtx);
      frame_cv.wait(lock, [] { return !frame_ready || !running; });
      if (!running)
        break;
      lock.unlock();
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
      std::unique_lock<std::mutex> lock(frame_mtx);
      std::swap(shared_frame, frame);
      frame_ready = true;
    }
    frame_cv.notify_one();
  }
}

void detectThread(Detector &detector, SerialPort &serial) {
  while (running) {
    cv::Mat frame;
    {
      std::unique_lock<std::mutex> lock(frame_mtx);
      frame_cv.wait(lock, [] { return frame_ready || !running; });
      if (!running && !frame_ready)
        break;
      std::swap(frame, shared_frame);
      frame_ready = false;
    }

    DetectionResult result = detector.process(frame);

    if (result.is_locked) {
      VisionData packet;
      packet.yaw_error = result.error_x;
      packet.pitch_error = result.error_y;
      packet.target_detected = 1;
      serial.send(packet);
    } else {
      VisionData lost_packet;
      lost_packet.yaw_error = 0;
      lost_packet.pitch_error = 0;
      lost_packet.target_detected = 0;
      serial.send(lost_packet);
    }

    cv::Mat display = frame.clone();
    if (g_enable_ui) {
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
    }

    cv::Mat mask = detector.getMask();
    cv::Mat roi_disp = detector.getRoiDisplay();
    cv::Mat gray = detector.getGray();

    {
      std::unique_lock<std::mutex> lock(result_mtx);
      std::swap(shared_display_frame, display);
      std::swap(shared_mask, mask);
      std::swap(shared_roi_display, roi_disp);
      std::swap(shared_gray, gray);
      shared_result = result;
      result_ready = true;
    }
    result_cv.notify_one();
  }
}

int main() {
  cv::FileStorage sysfs;
  bool use_demo = false, enable_ui = true, show_camera = true;
  bool show_binarized = true, show_gray = false, show_roi = true;
  int camera_fps = 120;
  int demo_fps_cfg = 0;
  int display_wait_ms = 1000 / camera_fps;
  std::string serial_port = "/dev/ttyUSB0";
  int serial_baud = 115200;
  if (sysfs.open("../configs/setting.yaml", cv::FileStorage::READ)) {
    if (!sysfs["UseDemoVideo"].empty())
      sysfs["UseDemoVideo"] >> use_demo;
    if (!sysfs["EnableUI"].empty())
      sysfs["EnableUI"] >> enable_ui;
    if (!sysfs["ShowCamera"].empty())
      sysfs["ShowCamera"] >> show_camera;
    if (!sysfs["ShowBinarized"].empty())
      sysfs["ShowBinarized"] >> show_binarized;
    if (!sysfs["ShowGray"].empty())
      sysfs["ShowGray"] >> show_gray;
    if (!sysfs["ShowROI"].empty())
      sysfs["ShowROI"] >> show_roi;
    if (!sysfs["CameraFPS"].empty())
      sysfs["CameraFPS"] >> camera_fps;
    if (!sysfs["DemoFPS"].empty())
      sysfs["DemoFPS"] >> demo_fps_cfg;
    if (!sysfs["SerialPort"].empty())
      sysfs["SerialPort"] >> serial_port;
    if (!sysfs["SerialBaud"].empty())
      sysfs["SerialBaud"] >> serial_baud;
    g_enable_ui = enable_ui;
    g_show_camera = show_camera;
  } else {
    std::cerr << "Warning: setting.yaml not found, using defaults"
              << std::endl;
  }

  SerialPort serial(serial_port, serial_baud);
  serial.init();

  Camera camera;
  bool use_camera = false;
  cv::VideoCapture video_cap;

  if (use_demo) {
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
    use_camera = camera.init("../configs/Camera.yaml", camera_fps);
    if (!use_camera) {
      std::cerr << "ERROR: Camera init failed! No camera found." << std::endl;
      return -1;
    }
    display_wait_ms = std::max(1, 1000 / camera_fps);
  }

  Detector detector;
  detector.init("../configs/detector.yaml");
  detector.initROI("../configs/roi.yaml");

  bool any_window = show_camera || show_binarized || show_gray || show_roi;
  if (any_window) {
    if (show_camera) {
      cv::namedWindow("demo", cv::WINDOW_NORMAL);
      cv::resizeWindow("demo", 800, 600);
    }
    if (show_binarized) {
      cv::namedWindow("Binarized", cv::WINDOW_NORMAL);
      cv::resizeWindow("Binarized", 800, 600);
    }
    if (show_gray) {
      cv::namedWindow("Gray", cv::WINDOW_NORMAL);
      cv::resizeWindow("Gray", 800, 600);
    }
    if (show_roi) {
      cv::namedWindow("ROI", cv::WINDOW_NORMAL);
      cv::resizeWindow("ROI", 400, 300);
    }
  }

  std::thread cap_thread(captureThread, std::ref(camera), std::ref(video_cap),
                         use_camera);
  std::thread det_thread(detectThread, std::ref(detector), std::ref(serial));

  auto fps_start = std::chrono::steady_clock::now();
  int fps_count = 0;
  double fps_value = 0.0;

  while (running) {
    if (any_window) {
      cv::Mat display, mask, roi_disp, gray;
      {
        std::unique_lock<std::mutex> lock(result_mtx);
        if (result_ready) {
          std::swap(display, shared_display_frame);
          std::swap(mask, shared_mask);
          std::swap(roi_disp, shared_roi_display);
          std::swap(gray, shared_gray);
          result_ready = false;
        }
      }
      if (!display.empty()) {
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

        if (show_camera)
          cv::imshow("demo", display);
      }
      if (show_binarized && !mask.empty()) {
        cv::imshow("Binarized", mask);
      }
      if (show_gray && !gray.empty()) {
        cv::imshow("Gray", gray);
      }
      if (show_roi && !roi_disp.empty()) {
        cv::imshow("ROI", roi_disp);
      }
      int key = cv::waitKey(display_wait_ms);
      if (key == 27) {
        running = false;
        frame_cv.notify_all();
        result_cv.notify_all();
        break;
      }
    } else {
      std::unique_lock<std::mutex> lock(result_mtx);
      result_cv.wait_for(lock, std::chrono::milliseconds(100),
                         [] { return !running; });
    }
  }

  cap_thread.join();
  det_thread.join();

  return 0;
}
