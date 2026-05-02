# Radar Vision — RoboMaster 雷达站自瞄视觉系统

基于 OpenCV + 海康 MVS 工业相机的装甲板自动识别与瞄准系统，通过串口将偏差数据发送至下位机，实现自动追踪与射击控制。

## 功能特性

- **装甲板灯条识别**：HSV 颜色过滤 + 灰度二值化 + 灯条配对
- **多线程架构**：取帧线程、检测+通信线程、UI 显示线程分离，充分利用多核性能
- **海康 MVS 工业相机**：支持 USB3.0 / GigE 工业相机，可配置曝光、增益、帧率等
- **串口通信**：自定义协议，向下位机发送 yaw/pitch 偏差与开火指令
- **Demo 模式**：无相机时可使用 `demo.mp4` 视频进行调试
- **参数热配置**：所有检测参数、相机参数、系统参数均通过 YAML 配置文件管理

## 目录结构

```
Radar_Vision/
├── CMakeLists.txt              # CMake 构建脚本
├── configs/
│   ├── setting.yaml            # 系统运行配置（相机/Demo切换、UI开关、串口等）
│   ├── detector.yaml           # 检测器参数（二值化阈值、灯条筛选、配对策略等）
│   └── Camera.yaml             # 相机参数（分辨率、曝光、增益、Gamma等）
├── include/
│   ├── camera/camera.hpp       # 相机类头文件
│   ├── detector/detector.hpp   # 检测器类头文件
│   ├── serial/serial.hpp       # 串口通信类头文件
│   └── MVS/                    # 海康 MVS SDK 头文件
├── src/
│   ├── main.cpp                # 主程序入口（多线程调度）
│   ├── camera/camera.cpp       # 相机驱动实现
│   ├── detector/detector.cpp   # 装甲板检测算法实现
│   └── serial/serial.cpp       # 串口通信实现
└── README.md
```

## 环境要求

| 项目 | 版本要求 |
|------|---------|
| 操作系统 | Ubuntu 22.04 LTS (x86_64 / aarch64) |
| 编译器 | GCC 11+ (支持 C++17) |
| CMake | >= 3.10 |
| OpenCV | >= 4.2 |
| 海康 MVS SDK | >= 3.x（安装至 `/opt/MVS`） |

---

## 一、系统基础环境配置

### 1.1 更新系统

```bash
sudo apt update && sudo apt upgrade -y
```

### 1.2 安装编译工具链

```bash
sudo apt install -y build-essential cmake git pkg-config
```

### 1.3 验证编译器版本

```bash
g++ --version   # 需要 >= 11，Ubuntu 22.04 默认自带 g++-11
cmake --version # 需要 >= 3.10
```

---

## 二、依赖安装

### 2.1 安装 OpenCV

**方式一：apt 安装（推荐，快速）**

```bash
sudo apt install -y libopencv-dev
```

验证安装：

```bash
pkg-config --modversion opencv4
```

**方式二：源码编译安装（需要特定版本或 CUDA 支持时）**

```bash
# 安装依赖
sudo apt install -y libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    libatlas-base-dev gfortran python3-numpy

# 下载源码
cd ~
git clone https://github.com/opencv/opencv.git -b 4.8.0 --depth 1
git clone https://github.com/opencv/opencv_contrib.git -b 4.8.0 --depth 1

# 编译
cd opencv && mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_TESTS=OFF \
      ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 2.2 安装海康 MVS SDK

1. 前往 [海康机器人官网](https://www.hikrobotics.com/cn/machinevision/service/download?module=0) 下载对应架构的 **MVS 运行环境 Linux 版**（`.deb` 或 `.tar.gz`）。

2. 安装 SDK：

```bash
# deb 包方式（推荐）
sudo dpkg -i MVS-x.x.x_x86_64.deb
# 或 aarch64 版本:
# sudo dpkg -i MVS-x.x.x_aarch64.deb

# tar.gz 方式
# tar -xzf MVS-x.x.x.tar.gz
# cd MVS-x.x.x
# sudo ./setup.sh
```

3. 配置动态库路径：

```bash
echo '/opt/MVS/lib/64' | sudo tee /etc/ld.so.conf.d/mvs.conf
sudo ldconfig
```

4. 验证安装：

```bash
ls /opt/MVS/lib/64/libMvCameraControl.so
# 应存在该文件
```

5. **设置 USB 权限**（USB 相机必须）：

```bash
# 创建 udev 规则，允许当前用户访问 USB 相机
sudo cp /opt/MVS/MVS/Misc/MVS.rules /etc/udev/rules.d/
# 如果上面的路径不存在，手动创建：
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bdf", MODE="0666"' | sudo tee /etc/udev/rules.d/99-mvs.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## 三、编译项目

### 3.1 克隆代码

```bash
git clone <仓库地址> Radar_Vision
cd Radar_Vision
```

### 3.2 构建

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

构建成功后会在 `build/` 目录下生成可执行文件 `RadarVision`。

### 3.3 常见编译问题排查

| 问题 | 解决方案 |
|------|---------|
| `MVS SDK not found` | 确认 `/opt/MVS/lib/64/libMvCameraControl.so` 存在，且已执行 `sudo ldconfig` |
| `OpenCV not found` | 确认 `pkg-config --modversion opencv4` 有输出，或检查 `CMAKE_PREFIX_PATH` |
| `MvCameraControl.h: No such file` | 确认 `include/MVS/` 目录下有海康 SDK 头文件 |

---

## 四、配置文件说明

所有配置文件位于 `configs/` 目录下，程序运行时从 `../configs/` 相对路径读取（即需要从 `build/` 目录运行）。

### 4.1 `setting.yaml` — 系统配置

```yaml
UseDemoVideo: 0        # 0=工业相机, 1=使用 demo.mp4
EnableUI: 1            # 是否显示可视化窗口
ShowBinarized: 1       # 显示二值化调试窗口
ShowGray: 0            # 显示灰度图调试窗口
ShowROI: 1             # 显示 ROI 区域窗口
CameraFPS: 120         # 工业相机采集帧率
DemoFPS: 30            # Demo 视频播放帧率（0=自动）
SerialPort: "/dev/ttyACM0"  # 串口设备路径
SerialBaud: 115200     # 串口波特率
```

### 4.2 `detector.yaml` — 检测器参数

```yaml
Detector:
  BinaryThresh: 90       # 灰度二值化阈值
  MinArea: 30.0           # 最小连通区域面积
  MinFoundFrame: 3        # 连续检测帧数确认阈值
  BarMinRatio: 1.5        # 灯条最小长宽比
  BarMaxRatio: 20.0       # 灯条最大长宽比
  PairLengthDiff: 0.6     # 灯条长度差异容忍度
  PairXDiffRatio: 0.8     # 水平偏移比最大值
  RectRatioTarget: 1.11   # 装甲板目标宽高比
  RectRatioTolerance: 0.7 # 宽高比容差
  EnemyColor: "blue"      # 敌方颜色: "red" 或 "blue"
  YawOffset: 0.0          # Yaw 偏置补偿
  PitchOffset: 0.0        # Pitch 偏置补偿
```

### 4.3 `Camera.yaml` — 相机参数

```yaml
Camera:
  Width: 0               # 图像宽度（0=相机最大分辨率）
  Height: 0              # 图像高度（0=相机最大分辨率）
  ExposureTime: 1000.0   # 曝光时间 (μs)
  Gain: 10.0             # 增益
  Gamma: 0.8             # Gamma 校正
  TriggerMode: 0         # 触发模式：0=连续采集, 1=外部触发
```

---

## 五、运行

### 5.1 工业相机模式

确保相机已连接，且 USB 权限已配置：

```bash
cd build
./RadarVision
```

### 5.2 Demo 视频模式

将测试视频放置在项目根目录下命名为 `demo.mp4`，并修改配置：

```bash
# 修改 configs/setting.yaml
# UseDemoVideo: 1
cd build
./RadarVision
```

### 5.3 运行时操作

- 按 `ESC` 退出程序
- 可视化窗口显示：
  - **demo** — 主画面（标注目标框、中心点、Yaw/Pitch 偏差、FPS）
  - **Binarized** — 二值化结果
  - **ROI** — 颜色过滤后的感兴趣区域
  - **Gray** — 灰度图（默认关闭）

---

## 六、串口通信协议

程序通过串口向下位机发送固定格式数据包：

| 字段 | 类型 | 字节数 | 说明 |
|------|------|--------|------|
| header | uint8_t | 1 | 帧头 `0xA5` |
| yaw_error | float | 4 | 水平偏差（像素） |
| pitch_error | float | 4 | 垂直偏差（像素） |
| at_center | uint8_t | 1 | 是否到达中心（0/1） |
| allow_fire | uint8_t | 1 | 是否允许发射（0/1） |

总包长度：**11 字节**（`__attribute__((packed))` 无填充对齐）

判定逻辑：当 `|yaw_error| < 10` 且 `|pitch_error| < 10` 时，`at_center = 1`，`allow_fire = 1`。

---

## 七、部署到目标设备

### 7.1 NUC / MiniPC 部署

```bash
# 在目标设备上安装依赖
sudo apt install -y build-essential cmake libopencv-dev
# 安装 MVS SDK (同上)

# 拷贝项目并编译
scp -r Radar_Vision/ user@target_ip:~/
ssh user@target_ip
cd ~/Radar_Vision
mkdir build && cd build
cmake .. && make -j$(nproc)
```

### 7.2 开机自启动（systemd）

创建 systemd 服务文件：

```bash
sudo tee /etc/systemd/system/radar-vision.service << 'EOF'
[Unit]
Description=Radar Vision Auto-aim Service
After=network.target

[Service]
Type=simple
User=<你的用户名>
WorkingDirectory=/home/<你的用户名>/Radar_Vision/build
ExecStart=/home/<你的用户名>/Radar_Vision/build/RadarVision
Restart=on-failure
RestartSec=3
Environment=DISPLAY=
Environment=MVCAM_COMMON_RUNENV=/opt/MVS/lib
Environment=LD_LIBRARY_PATH=/opt/MVS/lib/64

[Install]
WantedBy=multi-user.target
EOF
```

> **注意**：以 systemd 服务运行时，需关闭 UI 显示（`EnableUI: 0`），因为无桌面环境。

启用服务：

```bash
sudo systemctl daemon-reload
sudo systemctl enable radar-vision.service
sudo systemctl start radar-vision.service

# 查看运行状态
sudo systemctl status radar-vision.service

# 查看日志
journalctl -u radar-vision.service -f
```

### 7.3 性能调优建议

- **降低曝光时间**（`ExposureTime`）：减少运动模糊，提高检测稳定性
- **调整二值化阈值**（`BinaryThresh`）：根据场地光照条件调整
- **关闭 UI 显示**（`EnableUI: 0`）：部署时关闭可视化，节省 CPU/GPU 资源
- **提高帧率**（`CameraFPS`）：根据相机支持上限和处理能力适当提升
- **串口设备路径**：确认实际设备路径（`/dev/ttyUSB0` 或 `/dev/ttyACM0`），可通过 `ls /dev/ttyUSB* /dev/ttyACM*` 查看

---

## 八、调试技巧

```bash
# 查看串口设备
ls /dev/ttyUSB* /dev/ttyACM*

# 监控串口数据（需安装 minicom 或 screen）
sudo apt install -y minicom
minicom -D /dev/ttyACM0 -b 115200

# 查看相机是否被识别
lsusb | grep -i hik

# MVS 自带查看工具
/opt/MVS/bin/MVS
```

---

## License

本项目用于 RoboMaster 机器人竞赛，仅供学习和比赛使用。
