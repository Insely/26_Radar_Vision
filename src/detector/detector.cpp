#include "detector/detector.hpp"
#include <algorithm>

Detector::Detector() : found_count_(0) {}

bool Detector::init(const std::string &config_path) {
  if (config_path.empty())
    return false;

  cv::FileStorage fs(config_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Detector Config file not found: " << config_path << std::endl;
    return false;
  }

  if (!fs["Detector"]["BinaryThresh"].empty())
    fs["Detector"]["BinaryThresh"] >> binary_thresh_;

  if (!fs["Detector"]["MinArea"].empty())
    fs["Detector"]["MinArea"] >> min_area_;
  if (!fs["Detector"]["MinFoundFrame"].empty())
    fs["Detector"]["MinFoundFrame"] >> min_found_frame_;

  if (!fs["Detector"]["BarMinRatio"].empty())
    fs["Detector"]["BarMinRatio"] >> bar_min_ratio_;
  if (!fs["Detector"]["BarMaxRatio"].empty())
    fs["Detector"]["BarMaxRatio"] >> bar_max_ratio_;
  if (!fs["Detector"]["PairLengthDiff"].empty())
    fs["Detector"]["PairLengthDiff"] >> pair_length_diff_;
  if (!fs["Detector"]["PairXDiffRatio"].empty())
    fs["Detector"]["PairXDiffRatio"] >> pair_x_diff_ratio_;
  if (!fs["Detector"]["RectRatioTarget"].empty())
    fs["Detector"]["RectRatioTarget"] >> rect_ratio_target_;
  if (!fs["Detector"]["RectRatioTolerance"].empty())
    fs["Detector"]["RectRatioTolerance"] >> rect_ratio_tolerance_;

  if (!fs["Detector"]["EnemyColor"].empty()) {
    std::string color_str;
    fs["Detector"]["EnemyColor"] >> color_str;
    enemy_color_ = (color_str == "blue") ? 1 : 0;
  }

  if (!fs["Detector"]["YawOffset"].empty())
    fs["Detector"]["YawOffset"] >> yaw_offset_;
  if (!fs["Detector"]["PitchOffset"].empty())
    fs["Detector"]["PitchOffset"] >> pitch_offset_;

  dilate_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
  h_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 3));
  close_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

  std::cout << "[Detector] Config Loaded: "
            << "EnemyColor=" << (enemy_color_ == 0 ? "red" : "blue")
            << ", BinaryThresh=" << binary_thresh_
            << ", MinArea=" << min_area_
            << ", RectRatio=" << rect_ratio_target_
            << " +/-" << rect_ratio_tolerance_
            << ", MinFoundFrame=" << min_found_frame_ << std::endl;
  return true;
}

void Detector::preprocess(const cv::Mat &input) {
  cv::cvtColor(input, hsv_, cv::COLOR_BGR2HSV);

  cv::Mat color_mask;
  if (enemy_color_ == 0) {
    cv::Mat m1, m2;
    cv::inRange(hsv_, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), m1);
    cv::inRange(hsv_, cv::Scalar(170, 50, 50), cv::Scalar(180, 255, 255), m2);
    color_mask = m1 | m2;
  } else {
    cv::inRange(hsv_, cv::Scalar(100, 50, 50), cv::Scalar(130, 255, 255), color_mask);
  }

  cv::dilate(color_mask, color_mask, dilate_kernel_);

  color_contours_.clear();
  cv::findContours(color_mask, color_contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  cv::Rect roi(0, 0, input.cols, input.rows);
  if (!color_contours_.empty()) {
    int max_idx = 0;
    double max_area = 0;
    for (size_t i = 0; i < color_contours_.size(); i++) {
      double a = cv::contourArea(color_contours_[i]);
      if (a > max_area) {
        max_area = a;
        max_idx = (int)i;
      }
    }
    roi = cv::boundingRect(color_contours_[max_idx]);
    int pad = std::max(roi.width, roi.height) / 2;
    roi.x = std::max(0, roi.x - pad);
    roi.y = std::max(0, roi.y - pad);
    roi.width = std::min(input.cols - roi.x, roi.width + pad * 2);
    roi.height = std::min(input.rows - roi.y, roi.height + pad * 2);
  }

  color_mask_ = color_mask.clone();
  roi_display_ = input(roi).clone();
  roi_ = roi;

  cv::Mat roi_input = input(roi);
  cv::cvtColor(roi_input, gray_, cv::COLOR_BGR2GRAY);

  cv::threshold(gray_, mask_, binary_thresh_, 255, cv::THRESH_BINARY);
  cv::bitwise_and(mask_, color_mask(roi), mask_);

  cv::dilate(mask_, mask_, h_kernel_);
  cv::morphologyEx(mask_, mask_, cv::MORPH_CLOSE, close_kernel_);
  cv::GaussianBlur(mask_, mask_, cv::Size(5, 5), 0);
  cv::threshold(mask_, mask_, 128, 255, cv::THRESH_BINARY);
}

static LightBar makeLightBar(const cv::RotatedRect &rect) {
  LightBar bar;
  bar.rect = rect;

  cv::Point2f pts[4];
  rect.points(pts);

  float d01 = (float)cv::norm(pts[1] - pts[0]);
  float d12 = (float)cv::norm(pts[2] - pts[1]);

  if (d01 < d12) {
    bar.left = (pts[0] + pts[1]) * 0.5f;
    bar.right = (pts[2] + pts[3]) * 0.5f;
    bar.length = d12;
  } else {
    bar.left = (pts[1] + pts[2]) * 0.5f;
    bar.right = (pts[3] + pts[0]) * 0.5f;
    bar.length = d01;
  }

  if (bar.left.x > bar.right.x)
    std::swap(bar.left, bar.right);

  return bar;
}

bool Detector::findTarget(const cv::Mat &input, cv::Point2f &best_center,
                          cv::Point2f corners[4]) {
  contours_.clear();
  cv::findContours(mask_, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<LightBar> bars;
  for (const auto &contour : contours_) {
    double area = cv::contourArea(contour);
    if (area < min_area_)
      continue;

    cv::RotatedRect rrect = cv::minAreaRect(contour);
    rrect.center.x += roi_.x;
    rrect.center.y += roi_.y;
    float w = rrect.size.width;
    float h = rrect.size.height;
    float ratio = std::max(w, h) / (std::min(w, h) + 1e-5f);

    if (ratio >= bar_min_ratio_ && ratio <= bar_max_ratio_) {
      bars.push_back(makeLightBar(rrect));
    }
  }

  if (bars.size() < 2) {
    return false;
  }

  std::sort(bars.begin(), bars.end(),
            [](const LightBar &a, const LightBar &b) {
              return a.rect.center.y < b.rect.center.y;
            });

  double best_score = -1e9;
  int best_i = -1, best_j = -1;

  for (size_t i = 0; i < bars.size(); i++) {
    for (size_t j = i + 1; j < bars.size(); j++) {
      float avg_len = (bars[i].length + bars[j].length) / 2.0f;

      float len_diff = std::abs(bars[i].length - bars[j].length) /
                        std::max(bars[i].length, bars[j].length);
      if (len_diff > pair_length_diff_)
        continue;

      float dy = std::abs(bars[j].rect.center.y - bars[i].rect.center.y);
      if (dy < 1.0f)
        continue;

      float dx = std::abs(bars[j].rect.center.x - bars[i].rect.center.x);
      if (dx / avg_len > pair_x_diff_ratio_)
        continue;

      float rect_ratio = avg_len / dy;
      float ratio_err = std::abs(rect_ratio - (float)rect_ratio_target_);
      if (ratio_err > rect_ratio_tolerance_)
        continue;

      double score = -ratio_err * 200.0 + avg_len * 10.0 -
                     dx * 5.0 - len_diff * 30.0;

      if (score > best_score) {
        best_score = score;
        best_i = (int)i;
        best_j = (int)j;
      }
    }
  }

  if (best_i < 0)
    return false;

  const LightBar &upper = bars[best_i];
  const LightBar &lower = bars[best_j];

  corners[0] = upper.left;
  corners[1] = upper.right;
  corners[2] = lower.right;
  corners[3] = lower.left;

  best_center = (upper.rect.center + lower.rect.center) * 0.5f;
  return true;
}

DetectionResult Detector::process(const cv::Mat &frame) {
  preprocess(frame);

  cv::Point2f current_center(0, 0);
  cv::Point2f corners[4] = {};
  bool frame_found = findTarget(frame, current_center, corners);

  if (frame_found) {
    found_count_++;
  } else {
    found_count_ = 0;
  }

  DetectionResult result;
  result.is_locked = (found_count_ >= min_found_frame_);

  if (result.is_locked) {
    result.center = current_center;
    result.error_x = current_center.x - (frame.cols / 2.0f) - (float)yaw_offset_;
    result.error_y = current_center.y - (frame.rows / 2.0f) - (float)pitch_offset_;
    for (int i = 0; i < 4; i++)
      result.corners[i] = corners[i];
  } else {
    result.center = cv::Point2f(0, 0);
    result.error_x = 0;
    result.error_y = 0;
    for (int i = 0; i < 4; i++)
      result.corners[i] = cv::Point2f(0, 0);
  }

  return result;
}
