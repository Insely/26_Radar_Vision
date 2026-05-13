#pragma once
#include <chrono>
#include <opencv2/opencv.hpp>

struct LightBar {
  cv::RotatedRect rect;
  cv::Point2f left;
  cv::Point2f right;
  float length;
};

struct DetectionResult {
  bool is_locked;
  cv::Point2f center;
  float error_x;
  float error_y;
  cv::Point2f corners[4];
};

class Detector {
public:
  Detector();

  bool init(const std::string &config_path);
  bool initROI(const std::string &config_path);
  DetectionResult process(const cv::Mat &frame);

  cv::Mat getMask() const { return mask_.clone(); }
  cv::Mat getGray() const { return gray_.clone(); }
  cv::Mat getColorMask() const { return color_mask_.clone(); }
  cv::Mat getRoiDisplay() const { return roi_display_.clone(); }

private:
  void preprocess(const cv::Mat &input);
  bool findTarget(const cv::Mat &input, cv::Point2f &best_center,
                  cv::Point2f corners[4]);

  int binary_thresh_ = 200;
  double min_area_ = 30.0;
  int min_found_frame_ = 3;

  double bar_min_ratio_ = 1.2;
  double bar_max_ratio_ = 25.0;

  double pair_length_diff_ = 0.6;
  double pair_x_diff_ratio_ = 0.8;

  double rect_ratio_target_ = 1.11;
  double rect_ratio_tolerance_ = 0.7;

  int enemy_color_ = 0;

  double yaw_offset_ = 0.0;
  double pitch_offset_ = 0.0;

  int found_count_ = 0;
  cv::Rect roi_;
  cv::Mat gray_, mask_, color_mask_, roi_display_;

  cv::Mat hsv_;
  cv::Mat dilate_kernel_, h_kernel_, close_kernel_;
  std::vector<std::vector<cv::Point>> color_contours_;
  std::vector<std::vector<cv::Point>> contours_;

  // ---- ROI 色块抓取阈值 ----
  int red_h_low1_ = 0, red_s_low1_ = 50, red_v_low1_ = 50;
  int red_h_high1_ = 10, red_s_high1_ = 255, red_v_high1_ = 255;
  int red_h_low2_ = 170, red_s_low2_ = 50, red_v_low2_ = 50;
  int red_h_high2_ = 180, red_s_high2_ = 255, red_v_high2_ = 255;
  int blue_h_low_ = 100, blue_s_low_ = 50, blue_v_low_ = 50;
  int blue_h_high_ = 130, blue_s_high_ = 255, blue_v_high_ = 255;

  // ---- ROI 候选区域切换参数 ----
  double roi_min_blob_area_ = 100.0;
  int roi_max_candidates_ = 5;
  double roi_dwell_time_ = 0.5;
  double roi_pad_ratio_ = 0.5;

  // ---- ROI 切换运行时状态 ----
  std::vector<cv::Rect> roi_candidates_;
  int current_roi_index_ = 0;
  std::chrono::steady_clock::time_point roi_switch_time_;
  bool roi_timer_started_ = false;

  // 上一帧成功识别到目标的中心位置（用于优先就近取 ROI）
  cv::Point2f last_target_center_;
  bool has_last_target_ = false;
};
