#pragma once
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
};
