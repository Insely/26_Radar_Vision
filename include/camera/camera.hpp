#pragma once

#include "MVS/MvCameraControl.h"
#include <opencv2/opencv.hpp>
// Or just MvCameraControl.h if I added include/MVS to include_dirs.
// In CMakeLists I added `include/MVS`. So `#include "MvCameraControl.h"` is
// correct. However, to be safe and clear, let's look at how I updated CMake.
// `target_include_directories(... include/MVS ...)`
// So `#include "MvCameraControl.h"` is correct.

class Camera {
public:
  Camera();
  ~Camera();

  // 初始化相机：枚举设备、创建句柄、打开设备
  // config_path: 配置文件路径 (YAML)
  bool init(const std::string &config_path = "");

  // 获取一帧图像 (内部处理了格式转换)
  bool getFrame(cv::Mat &frame);

  // 关闭相机释放资源
  void close();

private:
  void *handle_;                       // 相机句柄
  unsigned char *pData_;               // 原始图像数据缓存
  unsigned char *pDataForRGB_;         // 转换后的RGB数据缓存
  unsigned int nDataSize_;             // 缓存大小
  MV_CC_DEVICE_INFO_LIST device_list_; // 设备列表
};
