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

  if (!fs["Detector"]["H_min"].empty())
    fs["Detector"]["H_min"] >> h_min_;
  if (!fs["Detector"]["H_max"].empty())
    fs["Detector"]["H_max"] >> h_max_;
  if (!fs["Detector"]["S_min"].empty())
    fs["Detector"]["S_min"] >> s_min_;
  if (!fs["Detector"]["S_max"].empty())
    fs["Detector"]["S_max"] >> s_max_;
  if (!fs["Detector"]["V_min"].empty())
    fs["Detector"]["V_min"] >> v_min_;
  if (!fs["Detector"]["V_max"].empty())
    fs["Detector"]["V_max"] >> v_max_;

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

  std::cout << "[Detector] Config Loaded: "
            << "H=" << h_min_ << "-" << h_max_ << ", S=" << s_min_ << "-"
            << s_max_ << ", V=" << v_min_ << "-" << v_max_
            << ", MinArea=" << min_area_
            << ", RectRatio=" << rect_ratio_target_
            << " +/-" << rect_ratio_tolerance_
            << ", MinFoundFrame=" << min_found_frame_ << std::endl;
  return true;
}

void Detector::preprocess(const cv::Mat &input) {
  cv::cvtColor(input, hsv_, cv::COLOR_BGR2HSV);

  cv::inRange(hsv_, cv::Scalar(h_min_, s_min_, v_min_),
              cv::Scalar(h_max_, s_max_, v_max_), mask_);

  // 水平方向膨胀，把同一灯条内相邻的 LED 段合并成一条
  cv::Mat h_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 3));
  cv::dilate(mask_, mask_, h_kernel);

  // 闭运算填补内部空隙
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(mask_, mask_, cv::MORPH_CLOSE, kernel);
}

// 从 RotatedRect 构造 LightBar，计算长轴两端点
static LightBar makeLightBar(const cv::RotatedRect &rect) {
  LightBar bar;
  bar.rect = rect;

  cv::Point2f pts[4];
  rect.points(pts);

  // 判断哪一对边是短边（短边中点 = 长轴端点）
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

  // 保证 left 在左, right 在右
  if (bar.left.x > bar.right.x)
    std::swap(bar.left, bar.right);

  return bar;
}

bool Detector::findTarget(const cv::Mat &input, cv::Point2f &best_center,
                          cv::Point2f corners[4]) {
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask_.clone(), contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // 第一步：提取所有符合条件的灯条
  std::vector<LightBar> bars;
  for (const auto &contour : contours) {
    double area = cv::contourArea(contour);
    if (area < min_area_)
      continue;

    cv::RotatedRect rrect = cv::minAreaRect(contour);
    float w = rrect.size.width;
    float h = rrect.size.height;
    float ratio = std::max(w, h) / (std::min(w, h) + 1e-5f);

    if (ratio >= bar_min_ratio_ && ratio <= bar_max_ratio_) {
      bars.push_back(makeLightBar(rrect));
    }
  }

  // --- 粘连/单色块兜底 ---
  if (bars.size() < 2) {
    // 只找到一个色块或全部粘连，直接用最大色块做凸包
    if (!contours.empty()) {
      size_t max_idx = 0;
      double max_area = 0;
      for (size_t i = 0; i < contours.size(); ++i) {
        double a = cv::contourArea(contours[i]);
        if (a > max_area) {
          max_area = a;
          max_idx = i;
        }
      }
      std::vector<cv::Point> hull;
      cv::convexHull(contours[max_idx], hull);
      // 填充小空洞
      cv::drawContours(mask_, std::vector<std::vector<cv::Point>>{hull}, 0, 255, cv::FILLED);
      // 再次提取角点
      cv::RotatedRect rect = cv::minAreaRect(hull);
      cv::Point2f pts[4];
      rect.points(pts);
      for (int i = 0; i < 4; ++i) corners[i] = pts[i];
      best_center = rect.center;
      return true;
    }
    return false;
  }

  // 第二步：按 y 排序（从上到下）
  std::sort(bars.begin(), bars.end(),
            [](const LightBar &a, const LightBar &b) {
              return a.rect.center.y < b.rect.center.y;
            });

  // 第三步：配对灯条，核心判据为四点矩形宽高比
  // 实际物体: 宽 5cm, 高 4.5cm, 宽高比 ≈ 1.11
  double best_score = -1e9;
  int best_i = -1, best_j = -1;

  for (size_t i = 0; i < bars.size(); i++) {
    for (size_t j = i + 1; j < bars.size(); j++) {
      float avg_len = (bars[i].length + bars[j].length) / 2.0f;

      // 长度相似性检查
      float len_diff = std::abs(bars[i].length - bars[j].length) /
                        std::max(bars[i].length, bars[j].length);
      if (len_diff > pair_length_diff_)
        continue;

      // 垂直距离（矩形高度）
      float dy = std::abs(bars[j].rect.center.y - bars[i].rect.center.y);
      if (dy < 1.0f)
        continue; // 避免两条灯条几乎重叠

      // 水平对齐检查
      float dx = std::abs(bars[j].rect.center.x - bars[i].rect.center.x);
      if (dx / avg_len > pair_x_diff_ratio_)
        continue;

      // 核心判据：矩形宽高比
      float rect_ratio = avg_len / dy;
      float ratio_err = std::abs(rect_ratio - (float)rect_ratio_target_);
      if (ratio_err > rect_ratio_tolerance_)
        continue;

      // 评分：宽高比越接近目标越好，面积越大越好，水平对齐越好
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

  // 第四步：输出四个角点和中心
  const LightBar &upper = bars[best_i];
  const LightBar &lower = bars[best_j];

  corners[0] = upper.left;  // 左上
  corners[1] = upper.right; // 右上
  corners[2] = lower.right; // 右下
  corners[3] = lower.left;  // 左下

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
  result.center = current_center;
  result.is_locked = (found_count_ >= min_found_frame_);
  result.error_x = current_center.x - (frame.cols / 2.0f);

  for (int i = 0; i < 4; i++)
    result.corners[i] = corners[i];

  return result;
}
