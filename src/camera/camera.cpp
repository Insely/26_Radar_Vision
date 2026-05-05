#include "camera/camera.hpp"
#include <iostream>
#include <unistd.h>

Camera::Camera()
    : camera_type_(CameraType::HIKROBOT),
      hik_handle_(nullptr), hik_pData_(nullptr),
      hik_pDataForRGB_(nullptr), hik_nDataSize_(0),
      mv_handle_(-1), mv_pRgbBuffer_(nullptr), mv_buffer_size_(0),
      flip180_(false) {}

Camera::~Camera() { close(); }

bool Camera::init(const std::string &config_path, float target_fps) {
  cv::FileStorage fs;
  if (!config_path.empty()) {
    fs.open(config_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "Config file not found: " << config_path << std::endl;
      return false;
    }
  }

  // 读取相机类型
  std::string type_str = "hikrobot";
  if (fs.isOpened() && !fs["CameraType"].empty()) {
    fs["CameraType"] >> type_str;
  }

  if (type_str == "mindvision") {
    camera_type_ = CameraType::MINDVISION;
    std::cout << "[Camera] Using MindVision camera" << std::endl;
    return initMindVision(fs, target_fps);
  } else {
    camera_type_ = CameraType::HIKROBOT;
    std::cout << "[Camera] Using Hikrobot camera" << std::endl;
    return initHikrobot(fs, target_fps);
  }
}

bool Camera::getFrame(cv::Mat &frame) {
  if (camera_type_ == CameraType::MINDVISION)
    return getFrameMindVision(frame);
  else
    return getFrameHikrobot(frame);
}

void Camera::close() {
  if (camera_type_ == CameraType::MINDVISION)
    closeMindVision();
  else
    closeHikrobot();
}

// ============================================================
//  海康机器人 (Hikrobot) 实现
// ============================================================

bool Camera::initHikrobot(const cv::FileStorage &fs, float target_fps) {
  int nRet = MV_OK;

  // 1. 枚举设备
  memset(&hik_device_list_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &hik_device_list_);
  if (MV_OK != nRet) {
    std::cerr << "[Hikrobot] Enum Devices fail! nRet [0x" << std::hex << nRet
              << "]" << std::endl;
    return false;
  }

  if (hik_device_list_.nDeviceNum > 0) {
    MV_CC_DEVICE_INFO *pDeviceInfo = hik_device_list_.pDeviceInfo[0];
    if (NULL != pDeviceInfo) {
      if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
        std::cout << "[Hikrobot] Found USB Device: "
                  << pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName
                  << " Model: "
                  << pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName
                  << std::endl;
      } else if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        std::cout << "[Hikrobot] Found GigE Device." << std::endl;
      }
    }
  } else {
    std::cerr << "[Hikrobot] Find No Devices!" << std::endl;
    return false;
  }

  // 2. 选择第一个设备并创建句柄
  nRet = MV_CC_CreateHandle(&hik_handle_, hik_device_list_.pDeviceInfo[0]);
  if (MV_OK != nRet) {
    std::cerr << "[Hikrobot] Create Handle fail! nRet [0x" << std::hex << nRet
              << "]" << std::endl;
    return false;
  }

  // 3. 打开设备
  nRet = MV_CC_OpenDevice(hik_handle_);
  if (MV_OK != nRet) {
    std::cerr << "[Hikrobot] Open Device fail! nRet [0x" << std::hex << nRet
              << "]" << std::endl;
    return false;
  }

  if (target_fps > 0.0f) {
    nRet =
        MV_CC_SetBoolValue(hik_handle_, "AcquisitionFrameRateEnable", true);
    if (MV_OK != nRet) {
      std::cerr << "[Hikrobot] Enable AcquisitionFrameRate fail!" << std::endl;
    }

    nRet = MV_CC_SetFrameRate(hik_handle_, target_fps);
    if (MV_OK != nRet) {
      std::cerr << "[Hikrobot] Set FrameRate fail!" << std::endl;
    } else {
      std::cout << "[Hikrobot] Set FrameRate: " << std::dec << target_fps
                << " fps" << std::endl;
    }
  }

  // 4. 加载参数
  if (fs.isOpened()) {
    int width = 0, height = 0;
    if (!fs["Hikrobot"]["Width"].empty())
      fs["Hikrobot"]["Width"] >> width;
    if (!fs["Hikrobot"]["Height"].empty())
      fs["Hikrobot"]["Height"] >> height;
    if (width > 0) {
      nRet = MV_CC_SetIntValue(hik_handle_, "Width", width);
      if (MV_OK != nRet)
        std::cerr << "[Hikrobot] Set Width fail!" << std::endl;
      else
        std::cout << "[Hikrobot] Set Width: " << width << std::endl;
    }
    if (height > 0) {
      nRet = MV_CC_SetIntValue(hik_handle_, "Height", height);
      if (MV_OK != nRet)
        std::cerr << "[Hikrobot] Set Height fail!" << std::endl;
      else
        std::cout << "[Hikrobot] Set Height: " << height << std::endl;
    }

    float exposure = 5000.0;
    float gain = 0.0;
    float gamma = 0.8;

    if (!fs["Hikrobot"]["ExposureTime"].empty()) {
      fs["Hikrobot"]["ExposureTime"] >> exposure;
      nRet = MV_CC_SetFloatValue(hik_handle_, "ExposureTime", exposure);
      if (MV_OK != nRet)
        std::cerr << "[Hikrobot] Set ExposureTime fail!" << std::endl;
      else
        std::cout << "[Hikrobot] Set ExposureTime: " << exposure << " us"
                  << std::endl;
    }

    if (!fs["Hikrobot"]["Gain"].empty()) {
      fs["Hikrobot"]["Gain"] >> gain;
      nRet = MV_CC_SetFloatValue(hik_handle_, "Gain", gain);
      if (MV_OK != nRet)
        std::cerr << "[Hikrobot] Set Gain fail!" << std::endl;
      else
        std::cout << "[Hikrobot] Set Gain: " << gain << std::endl;
    }

    if (!fs["Hikrobot"]["Gamma"].empty()) {
      fs["Hikrobot"]["Gamma"] >> gamma;
      nRet = MV_CC_SetBoolValue(hik_handle_, "GammaEnable", true);
      nRet = MV_CC_SetFloatValue(hik_handle_, "Gamma", gamma);
      if (MV_OK != nRet)
        std::cerr << "[Hikrobot] Set Gamma fail!" << std::endl;
      else
        std::cout << "[Hikrobot] Set Gamma: " << gamma << std::endl;
    }

    if (!fs["Hikrobot"]["TriggerMode"].empty()) {
      int triggerMode = 0;
      fs["Hikrobot"]["TriggerMode"] >> triggerMode;
      nRet = MV_CC_SetEnumValue(hik_handle_, "TriggerMode", triggerMode);
      if (MV_OK != nRet)
        std::cerr << "[Hikrobot] Set TriggerMode fail!" << std::endl;
      else
        std::cout << "[Hikrobot] Set TriggerMode: " << triggerMode << std::endl;
    }
  } else {
    nRet = MV_CC_SetEnumValue(hik_handle_, "TriggerMode", 0);
  }

  // 获取 PayloadSize，分配缓冲区
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(hik_handle_, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    std::cerr << "[Hikrobot] Get PayloadSize fail! nRet [0x" << std::hex
              << nRet << "]" << std::endl;
    return false;
  }
  hik_nDataSize_ = stParam.nCurValue;
  hik_pData_ = (unsigned char *)malloc(hik_nDataSize_);
  if (!hik_pData_) {
    std::cerr << "[Hikrobot] Failed to allocate raw image buffer" << std::endl;
    return false;
  }
  hik_pDataForRGB_ = (unsigned char *)malloc(hik_nDataSize_ * 3 + 2048);
  if (!hik_pDataForRGB_) {
    std::cerr << "[Hikrobot] Failed to allocate RGB conversion buffer"
              << std::endl;
    free(hik_pData_);
    hik_pData_ = nullptr;
    return false;
  }

  // 5. 开始取流
  nRet = MV_CC_StartGrabbing(hik_handle_);
  if (MV_OK != nRet) {
    std::cerr << "[Hikrobot] Start Grabbing fail! nRet [0x" << std::hex << nRet
              << "]" << std::endl;
    return false;
  }

  // 读取翻转配置
  if (fs.isOpened() && !fs["Hikrobot"]["Flip180"].empty()) {
    int flip = 0;
    fs["Hikrobot"]["Flip180"] >> flip;
    flip180_ = (flip != 0);
  }

  std::cout << "[Hikrobot] Camera Init Success!" << std::endl;
  return true;
}

bool Camera::getFrameHikrobot(cv::Mat &frame) {
  if (nullptr == hik_handle_)
    return false;

  MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

  int nRet = MV_CC_GetOneFrameTimeout(hik_handle_, hik_pData_, hik_nDataSize_,
                                      &stImageInfo, 1000);
  if (MV_OK != nRet) {
    return false;
  }

  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
  stConvertParam.nWidth = stImageInfo.nWidth;
  stConvertParam.nHeight = stImageInfo.nHeight;
  stConvertParam.pSrcData = hik_pData_;
  stConvertParam.nSrcDataLen = hik_nDataSize_;
  stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
  stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
  stConvertParam.pDstBuffer = hik_pDataForRGB_;
  stConvertParam.nDstBufferSize = hik_nDataSize_ * 3 + 2048;

  nRet = MV_CC_ConvertPixelType(hik_handle_, &stConvertParam);
  if (MV_OK != nRet) {
    std::cerr << "[Hikrobot] Convert Pixel fail! nRet [0x" << std::hex << nRet
              << "]" << std::endl;
    return false;
  }

  frame =
      cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3,
              hik_pDataForRGB_)
          .clone();

  if (flip180_)
    cv::flip(frame, frame, -1);

  return true;
}

void Camera::closeHikrobot() {
  if (hik_handle_ != nullptr) {
    MV_CC_StopGrabbing(hik_handle_);
    MV_CC_CloseDevice(hik_handle_);
    MV_CC_DestroyHandle(hik_handle_);
    hik_handle_ = nullptr;
  }
  if (hik_pData_) {
    free(hik_pData_);
    hik_pData_ = nullptr;
  }
  if (hik_pDataForRGB_) {
    free(hik_pDataForRGB_);
    hik_pDataForRGB_ = nullptr;
  }
}

// ============================================================
//  迈德威视 (MindVision) 实现
// ============================================================

bool Camera::initMindVision(const cv::FileStorage &fs, float target_fps) {
  // 1. 初始化SDK
  CameraSdkInit(0);

  // 2. 枚举设备
  tSdkCameraDevInfo sCameraList[10];
  int iCameraNums = 10;
  if (CameraEnumerateDevice(sCameraList, &iCameraNums) !=
          CAMERA_STATUS_SUCCESS ||
      iCameraNums == 0) {
    std::cerr << "[MindVision] No camera found!" << std::endl;
    return false;
  }
  std::cout << "[MindVision] Found " << iCameraNums << " camera(s)"
            << std::endl;
  std::cout << "[MindVision] Using: " << sCameraList[0].acFriendlyName
            << std::endl;

  // 3. 初始化相机
  if (CameraInit(&sCameraList[0], -1, -1, &mv_handle_) !=
      CAMERA_STATUS_SUCCESS) {
    std::cerr << "[MindVision] Camera init failed!" << std::endl;
    return false;
  }

  // 4. 获取相机能力
  tSdkCameraCapbility sCameraInfo;
  CameraGetCapability(mv_handle_, &sCameraInfo);

  // 分配RGB缓冲区
  mv_buffer_size_ = sCameraInfo.sResolutionRange.iWidthMax *
                    sCameraInfo.sResolutionRange.iHeightMax * 3;
  mv_pRgbBuffer_ = (unsigned char *)malloc(mv_buffer_size_);
  if (!mv_pRgbBuffer_) {
    std::cerr << "[MindVision] Failed to allocate RGB buffer" << std::endl;
    CameraUnInit(mv_handle_);
    mv_handle_ = -1;
    return false;
  }

  // 设置ISP输出为BGR
  CameraSetIspOutFormat(mv_handle_, CAMERA_MEDIA_TYPE_BGR8);

  // 5. 加载参数
  if (fs.isOpened()) {
    // 分辨率
    int width = 0, height = 0;
    if (!fs["MindVision"]["Width"].empty())
      fs["MindVision"]["Width"] >> width;
    if (!fs["MindVision"]["Height"].empty())
      fs["MindVision"]["Height"] >> height;
    if (width > 0 && height > 0) {
      tSdkImageResolution sResolution;
      memset(&sResolution, 0, sizeof(tSdkImageResolution));
      sResolution.iIndex = 0xff; // 自定义分辨率
      sResolution.iWidth = width;
      sResolution.iHeight = height;
      sResolution.iWidthFOV = width;
      sResolution.iHeightFOV = height;
      CameraSetImageResolution(mv_handle_, &sResolution);
      std::cout << "[MindVision] Set Resolution: " << width << "x" << height
                << std::endl;
    }

    // 曝光
    double exposure = 5000.0;
    if (!fs["MindVision"]["ExposureTime"].empty()) {
      fs["MindVision"]["ExposureTime"] >> exposure;
      CameraSetAeState(mv_handle_, FALSE); // 手动曝光
      CameraSetExposureTime(mv_handle_, exposure);
      std::cout << "[MindVision] Set ExposureTime: " << exposure << " us"
                << std::endl;
    }

    // 模拟增益
    int analogGain = 1;
    if (!fs["MindVision"]["AnalogGain"].empty()) {
      fs["MindVision"]["AnalogGain"] >> analogGain;
      CameraSetAnalogGain(mv_handle_, analogGain);
      std::cout << "[MindVision] Set AnalogGain: " << analogGain << std::endl;
    }

    // Gamma (MindVision gamma范围 0~800, 100=1.0)
    int gamma = 100;
    if (!fs["MindVision"]["Gamma"].empty()) {
      fs["MindVision"]["Gamma"] >> gamma;
      CameraSetGamma(mv_handle_, gamma);
      std::cout << "[MindVision] Set Gamma: " << gamma << std::endl;
    }

    // 触发模式 (0: Continuous, 1: Software, 2: Hardware)
    int triggerMode = 0;
    if (!fs["MindVision"]["TriggerMode"].empty()) {
      fs["MindVision"]["TriggerMode"] >> triggerMode;
      CameraSetTriggerMode(mv_handle_, triggerMode);
      std::cout << "[MindVision] Set TriggerMode: " << triggerMode << std::endl;
    }
  }

  // 读取翻转配置
  if (fs.isOpened() && !fs["MindVision"]["Flip180"].empty()) {
    int flip = 0;
    fs["MindVision"]["Flip180"] >> flip;
    flip180_ = (flip != 0);
  }

  // 6. 开始采集
  CameraPlay(mv_handle_);

  std::cout << "[MindVision] Camera Init Success!" << std::endl;
  return true;
}

bool Camera::getFrameMindVision(cv::Mat &frame) {
  if (mv_handle_ < 0)
    return false;

  tSdkFrameHead sFrameInfo;
  BYTE *pbyBuffer;

  if (CameraGetImageBuffer(mv_handle_, &sFrameInfo, &pbyBuffer, 1000) !=
      CAMERA_STATUS_SUCCESS) {
    return false;
  }

  // ISP处理: 将原始图像转换为BGR
  CameraImageProcess(mv_handle_, pbyBuffer, mv_pRgbBuffer_, &sFrameInfo);

  frame = cv::Mat(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3,
                  mv_pRgbBuffer_)
              .clone();

  CameraReleaseImageBuffer(mv_handle_, pbyBuffer);

  if (flip180_)
    cv::flip(frame, frame, -1);

  return true;
}

void Camera::closeMindVision() {
  if (mv_handle_ >= 0) {
    CameraUnInit(mv_handle_);
    mv_handle_ = -1;
  }
  if (mv_pRgbBuffer_) {
    free(mv_pRgbBuffer_);
    mv_pRgbBuffer_ = nullptr;
  }
}
