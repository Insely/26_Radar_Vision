#pragma once

#include "MvCameraControl.h"
#include "CameraApi.h"
#include <opencv2/opencv.hpp>
#include <string>

enum class CameraType {
  HIKROBOT,    // 海康机器人
  MINDVISION   // 迈德威视
};

class Camera {
public:
  Camera();
  ~Camera();

  bool init(const std::string &config_path = "", float target_fps = 0.0f);
  bool getFrame(cv::Mat &frame);
  void close();

private:
  CameraType camera_type_;
  bool flip180_;

  // === 海康机器人 ===
  void *hik_handle_;
  unsigned char *hik_pData_;
  unsigned char *hik_pDataForRGB_;
  unsigned int hik_nDataSize_;
  MV_CC_DEVICE_INFO_LIST hik_device_list_;

  bool initHikrobot(const cv::FileStorage &fs, float target_fps);
  bool getFrameHikrobot(cv::Mat &frame);
  void closeHikrobot();

  // === 迈德威视 ===
  CameraHandle mv_handle_;
  unsigned char *mv_pRgbBuffer_;
  int mv_buffer_size_;

  bool initMindVision(const cv::FileStorage &fs, float target_fps);
  bool getFrameMindVision(cv::Mat &frame);
  void closeMindVision();
};
