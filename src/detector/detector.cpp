#include "detector/detector.hpp"
#include <algorithm>
#include <chrono>

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

bool Detector::initROI(const std::string &config_path) {
  if (config_path.empty())
    return false;

  cv::FileStorage fs(config_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "ROI Config file not found: " << config_path << std::endl;
    return false;
  }

  // HSV 阈值 - 红色双区间
  if (!fs["ROI"]["RedHLow1"].empty()) fs["ROI"]["RedHLow1"] >> red_h_low1_;
  if (!fs["ROI"]["RedSLow1"].empty()) fs["ROI"]["RedSLow1"] >> red_s_low1_;
  if (!fs["ROI"]["RedVLow1"].empty()) fs["ROI"]["RedVLow1"] >> red_v_low1_;
  if (!fs["ROI"]["RedHHigh1"].empty()) fs["ROI"]["RedHHigh1"] >> red_h_high1_;
  if (!fs["ROI"]["RedSHigh1"].empty()) fs["ROI"]["RedSHigh1"] >> red_s_high1_;
  if (!fs["ROI"]["RedVHigh1"].empty()) fs["ROI"]["RedVHigh1"] >> red_v_high1_;
  if (!fs["ROI"]["RedHLow2"].empty()) fs["ROI"]["RedHLow2"] >> red_h_low2_;
  if (!fs["ROI"]["RedSLow2"].empty()) fs["ROI"]["RedSLow2"] >> red_s_low2_;
  if (!fs["ROI"]["RedVLow2"].empty()) fs["ROI"]["RedVLow2"] >> red_v_low2_;
  if (!fs["ROI"]["RedHHigh2"].empty()) fs["ROI"]["RedHHigh2"] >> red_h_high2_;
  if (!fs["ROI"]["RedSHigh2"].empty()) fs["ROI"]["RedSHigh2"] >> red_s_high2_;
  if (!fs["ROI"]["RedVHigh2"].empty()) fs["ROI"]["RedVHigh2"] >> red_v_high2_;

  // HSV 阈值 - 蓝色
  if (!fs["ROI"]["BlueHLow"].empty()) fs["ROI"]["BlueHLow"] >> blue_h_low_;
  if (!fs["ROI"]["BlueSLow"].empty()) fs["ROI"]["BlueSLow"] >> blue_s_low_;
  if (!fs["ROI"]["BlueVLow"].empty()) fs["ROI"]["BlueVLow"] >> blue_v_low_;
  if (!fs["ROI"]["BlueHHigh"].empty()) fs["ROI"]["BlueHHigh"] >> blue_h_high_;
  if (!fs["ROI"]["BlueSHigh"].empty()) fs["ROI"]["BlueSHigh"] >> blue_s_high_;
  if (!fs["ROI"]["BlueVHigh"].empty()) fs["ROI"]["BlueVHigh"] >> blue_v_high_;

  // 候选区域参数
  if (!fs["ROI"]["MinBlobArea"].empty()) fs["ROI"]["MinBlobArea"] >> roi_min_blob_area_;
  if (!fs["ROI"]["MaxCandidates"].empty()) fs["ROI"]["MaxCandidates"] >> roi_max_candidates_;
  if (!fs["ROI"]["DwellTime"].empty()) fs["ROI"]["DwellTime"] >> roi_dwell_time_;
  if (!fs["ROI"]["PadRatio"].empty()) fs["ROI"]["PadRatio"] >> roi_pad_ratio_;

  std::cout << "[ROI] Config Loaded: "
            << "MinBlobArea=" << roi_min_blob_area_
            << ", MaxCandidates=" << roi_max_candidates_
            << ", DwellTime=" << roi_dwell_time_ << "s"
            << ", PadRatio=" << roi_pad_ratio_ << std::endl;
  return true;
}

void Detector::preprocess(const cv::Mat &input) {
  cv::cvtColor(input, hsv_, cv::COLOR_BGR2HSV);

  cv::Mat color_mask;
  if (enemy_color_ == 0) {
    // 红色：使用可配置阈值
    cv::Mat m1, m2;
    cv::inRange(hsv_, cv::Scalar(red_h_low1_, red_s_low1_, red_v_low1_),
                cv::Scalar(red_h_high1_, red_s_high1_, red_v_high1_), m1);
    cv::inRange(hsv_, cv::Scalar(red_h_low2_, red_s_low2_, red_v_low2_),
                cv::Scalar(red_h_high2_, red_s_high2_, red_v_high2_), m2);
    color_mask = m1 | m2;
  } else {
    // 蓝色：使用可配置阈值
    cv::inRange(hsv_, cv::Scalar(blue_h_low_, blue_s_low_, blue_v_low_),
                cv::Scalar(blue_h_high_, blue_s_high_, blue_v_high_), color_mask);
  }

  cv::dilate(color_mask, color_mask, dilate_kernel_);

  color_contours_.clear();
  cv::findContours(color_mask, color_contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 收集所有符合面积条件的候选区域，按面积降序排列取前 N 个
  struct BlobInfo {
    cv::Rect rect;
    double area;
  };
  std::vector<BlobInfo> blobs;
  for (const auto &contour : color_contours_) {
    double a = cv::contourArea(contour);
    if (a >= roi_min_blob_area_) {
      blobs.push_back({cv::boundingRect(contour), a});
    }
  }
  std::sort(blobs.begin(), blobs.end(),
            [](const BlobInfo &a, const BlobInfo &b) { return a.area > b.area; });

  // 如果有上一帧目标位置，按距离上次目标中心的距离升序排列，否则保持面积降序
  if (has_last_target_) {
    cv::Point2f lc = last_target_center_;
    std::sort(blobs.begin(), blobs.end(),
              [&lc](const BlobInfo &a, const BlobInfo &b) {
                cv::Point2f ca(a.rect.x + a.rect.width * 0.5f,
                               a.rect.y + a.rect.height * 0.5f);
                cv::Point2f cb(b.rect.x + b.rect.width * 0.5f,
                               b.rect.y + b.rect.height * 0.5f);
                float da = (ca.x - lc.x) * (ca.x - lc.x) + (ca.y - lc.y) * (ca.y - lc.y);
                float db = (cb.x - lc.x) * (cb.x - lc.x) + (cb.y - lc.y) * (cb.y - lc.y);
                return da < db;
              });
  }

  // 取前 MaxCandidates 个作为候选 ROI
  roi_candidates_.clear();
  int n = std::min((int)blobs.size(), roi_max_candidates_);
  for (int i = 0; i < n; i++) {
    cv::Rect r = blobs[i].rect;
    int pad = (int)(std::max(r.width, r.height) * roi_pad_ratio_);
    r.x = std::max(0, r.x - pad);
    r.y = std::max(0, r.y - pad);
    r.width = std::min(input.cols - r.x, r.width + pad * 2);
    r.height = std::min(input.rows - r.y, r.height + pad * 2);
    roi_candidates_.push_back(r);
  }

  // 选择当前候选 ROI（有上帧目标时默认从最近的候选开始）
  cv::Rect roi(0, 0, input.cols, input.rows);
  if (!roi_candidates_.empty()) {
    if (current_roi_index_ >= (int)roi_candidates_.size())
      current_roi_index_ = 0;
    roi = roi_candidates_[current_roi_index_];
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
    // 识别到目标，记录位置并重置切换计时器，锁定当前 ROI
    last_target_center_ = current_center;
    has_last_target_ = true;
    roi_timer_started_ = false;
    current_roi_index_ = 0;  // 下一帧从最近候选开始
  } else {
    found_count_ = 0;
    // 未识别到目标，启动/检查切换计时器
    if (roi_candidates_.size() > 1) {
      if (!roi_timer_started_) {
        roi_switch_time_ = std::chrono::steady_clock::now();
        roi_timer_started_ = true;
      } else {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - roi_switch_time_).count();
        if (elapsed >= roi_dwell_time_) {
          // 停留时间到，切换到下一个候选区域
          current_roi_index_ = (current_roi_index_ + 1) % (int)roi_candidates_.size();
          roi_switch_time_ = now;
        }
      }
    }
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
