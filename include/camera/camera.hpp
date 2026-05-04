#pragma once

#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>

class Camera {
public:
  Camera();
  ~Camera();

  bool init(const std::string &config_path = "", float target_fps = 0.0f);
  bool getFrame(cv::Mat &frame);
  void close();

private:
  void *handle_;
  unsigned char *pData_;
  unsigned char *pDataForRGB_;
  unsigned int nDataSize_;
  MV_CC_DEVICE_INFO_LIST device_list_;
};
