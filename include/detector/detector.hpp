#pragma once
#include <opencv2/opencv.hpp>

// 灯条结构体
struct LightBar {
  cv::RotatedRect rect;
  cv::Point2f left;  // 灯条左端点（长轴方向）
  cv::Point2f right; // 灯条右端点（长轴方向）
  float length;      // 灯条长度
};

// 检测结果
struct DetectionResult {
  bool is_locked;        // 是否锁定
  cv::Point2f center;    // 目标中心点
  float error_x;         // X 轴偏差
  cv::Point2f corners[4]; // 目标矩形四个角点: TL, TR, BR, BL
};

class Detector {
public:
  Detector();

  bool init(const std::string &config_path);
  DetectionResult process(const cv::Mat &frame);

  cv::Mat getMask() const { return mask_; }
  cv::Mat getGray() const { return gray_; }

private:
  void preprocess(const cv::Mat &input);
  bool findTarget(const cv::Mat &input, cv::Point2f &best_center,
                  cv::Point2f corners[4]);

  // 灰度二值化阈值
  int binary_thresh_ = 200;  // 二值化阈值（亮度高于此为白）

  double min_area_ = 30.0;  // 最小面积过滤
  int min_found_frame_ = 3; // 连续检测帧数阈值

  // 灯条筛选参数
  double bar_min_ratio_ = 1.2;  // 灯条最小长宽比（放宽）
  double bar_max_ratio_ = 25.0; // 灯条最大长宽比（放宽）

  // 灯条配对参数
  double pair_length_diff_ = 0.6;    // 两灯条长度差异容忍度
  double pair_x_diff_ratio_ = 0.8;   // 水平偏移/平均长度 最大值

  // 矩形比例参数（核心判据）
  // 实际: 宽5cm, 高4.5cm, 宽高比 ≈ 1.11
  double rect_ratio_target_ = 1.11;  // 目标宽高比
  double rect_ratio_tolerance_ = 0.7; // 宽高比容差（允许 0.41 ~ 1.81）

  int found_count_ = 0;
  cv::Mat gray_, mask_;
};