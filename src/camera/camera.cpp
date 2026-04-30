#include "camera/camera.hpp"
#include <iostream>
#include <unistd.h>

Camera::Camera()
    : handle_(nullptr), pData_(nullptr), pDataForRGB_(nullptr), nDataSize_(0) {}

Camera::~Camera() { close(); }

bool Camera::init(const std::string &config_path) {
  int nRet = MV_OK;

  // 1. 枚举设备
  memset(&device_list_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list_);
  if (MV_OK != nRet) {
    std::cerr << "Enum Devices fail! nRet [0x" << std::hex << nRet << "]"
              << std::endl;
    return false;
  }

  if (device_list_.nDeviceNum > 0) {
    // 打印设备信息
    MV_CC_DEVICE_INFO *pDeviceInfo = device_list_.pDeviceInfo[0];
    if (NULL != pDeviceInfo) {
      if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
        std::cout << "[Camera] Found USB Device: "
                  << pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName
                  << " Model: "
                  << pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName
                  << std::endl;
      } else if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        std::cout << "[Camera] Found GigE Device." << std::endl;
      }
    }
  } else {
    std::cerr << "Find No Devices!" << std::endl;
    return false;
  }

  // 2. 选择第一个设备并创建句柄
  nRet = MV_CC_CreateHandle(&handle_, device_list_.pDeviceInfo[0]);
  if (MV_OK != nRet) {
    std::cerr << "Create Handle fail! nRet [0x" << std::hex << nRet << "]"
              << std::endl;
    return false;
  }

  // 3. 打开设备
  nRet = MV_CC_OpenDevice(handle_);
  if (MV_OK != nRet) {
    std::cerr << "Open Device fail! nRet [0x" << std::hex << nRet << "]"
              << std::endl;
    return false;
  }

  // 4. 加载参数
  if (!config_path.empty()) {
    cv::FileStorage fs(config_path, cv::FileStorage::READ);
    if (fs.isOpened()) {
      float exposure = 5000.0;
      float gain = 0.0;
      float gamma = 0.8; // default

      // ExposureTime
      if (!fs["Camera"]["ExposureTime"].empty()) {
        fs["Camera"]["ExposureTime"] >> exposure;
        nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", exposure);
        if (MV_OK != nRet)
          std::cerr << "Set ExposureTime fail!" << std::endl;
        else
          std::cout << "Set ExposureTime: " << exposure << " us" << std::endl;
      }

      // Gain
      if (!fs["Camera"]["Gain"].empty()) {
        fs["Camera"]["Gain"] >> gain;
        nRet = MV_CC_SetFloatValue(handle_, "Gain", gain);
        if (MV_OK != nRet)
          std::cerr << "Set Gain fail!" << std::endl;
        else
          std::cout << "Set Gain: " << gain << std::endl;
      }

      // Gamma
      if (!fs["Camera"]["Gamma"].empty()) {
        fs["Camera"]["Gamma"] >> gamma;
        // Enable Gamma first? Some cameras usually need GammaEnable
        nRet = MV_CC_SetBoolValue(handle_, "GammaEnable", true);
        nRet = MV_CC_SetFloatValue(handle_, "Gamma", gamma);
        if (MV_OK != nRet)
          std::cerr << "Set Gamma fail!" << std::endl;
        else
          std::cout << "Set Gamma: " << gamma << std::endl;
      }

      // TriggerMode (0: Off, 1: On)
      if (!fs["Camera"]["TriggerMode"].empty()) {
        int triggerMode = 0;
        fs["Camera"]["TriggerMode"] >> triggerMode;
        nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", triggerMode);
        if (MV_OK != nRet)
          std::cerr << "Set TriggerMode fail!" << std::endl;
        else
          std::cout << "Set TriggerMode: " << triggerMode << std::endl;
      }

    } else {
      std::cerr << "Config file not found: " << config_path << std::endl;
      // Default fallback
      nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
    }
  } else {
    // No config, default
    nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
  }

  // 获取图像宽高，分配缓冲区 (可选，MV_CC_GetImageForBGR
  // 需要预知的buffer大小吗？通常不需要，它内部处理或者我们给够大)
  // 简单起见，分配足量包含 PayloadSize
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    std::cerr << "Get PayloadSize fail! nRet [0x" << std::hex << nRet << "]"
              << std::endl;
    return false;
  }
  nDataSize_ = stParam.nCurValue;
  pData_ = (unsigned char *)malloc(nDataSize_);
  // RGB buffer, max size estimate (width * height * 3)
  // 假设 5MP (2592*1944*3) 约 15MB. PayloadSize通常是Raw大小.
  // 安全起见给个大点的固定值 或基于 PayloadSize * 3 if mono8 -> rgb
  pDataForRGB_ = (unsigned char *)malloc(nDataSize_ * 3 + 2048);

  // 5. 开始取流
  nRet = MV_CC_StartGrabbing(handle_);
  if (MV_OK != nRet) {
    std::cerr << "Start Grabbing fail! nRet [0x" << std::hex << nRet << "]"
              << std::endl;
    return false;
  }

  std::cout << "Camera Init Success!" << std::endl;
  return true;
}

bool Camera::getFrame(cv::Mat &frame) {
  if (nullptr == handle_)
    return false;

  MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

  // 获取一帧数据 (超时 1000ms)
  // 注意: 这里使用 MV_CC_GetOneFrameTimeout 获取原始流数据，然后转换
  // 或者直接用 internal 转换接口
  int nRet =
      MV_CC_GetOneFrameTimeout(handle_, pData_, nDataSize_, &stImageInfo, 1000);
  if (MV_OK != nRet) {
    // std::cerr << "Get Frame fail! nRet [0x" << std::hex << nRet << "]" <<
    // std::endl;
    return false;
  }

  // 转换到 OpenCV Mat (BGR)
  // PixelType detection
  // 如果是 Mono8, 直接构建 Mat
  // 如果是 Bayer, 需要转换

  // 我们统一把数据转成 BGR 给 OpenCV
  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
  stConvertParam.nWidth = stImageInfo.nWidth;
  stConvertParam.nHeight = stImageInfo.nHeight;
  stConvertParam.pSrcData = pData_;
  stConvertParam.nSrcDataLen = nDataSize_; // 注意这里用 buffer size 还是 frame
                                           // len? 通常 frame len <= buffer size
  stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
  stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
  stConvertParam.pDstBuffer = pDataForRGB_;
  stConvertParam.nDstBufferSize = nDataSize_ * 3 + 2048;

  nRet = MV_CC_ConvertPixelType(handle_, &stConvertParam);
  if (MV_OK != nRet) {
    std::cerr << "Convert Pixel fail! nRet [0x" << std::hex << nRet << "]"
              << std::endl;
    return false;
  }

  // Create Mat directly from the buffer
  // cv::Mat(height, width, type, data)
  frame =
      cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForRGB_)
          .clone();

  return true;
}

void Camera::close() {
  if (handle_ != nullptr) {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }
  if (pData_) {
    free(pData_);
    pData_ = nullptr;
  }
  if (pDataForRGB_) {
    free(pDataForRGB_);
    pDataForRGB_ = nullptr;
  }
}
