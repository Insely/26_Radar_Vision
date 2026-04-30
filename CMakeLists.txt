cmake_minimum_required(VERSION 3.10)
project(DartVision)

# 1. 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Generate compile_commands.json for editor support

# 2. 寻找 OpenCV 库
find_package(OpenCV REQUIRED)

# 3. 收集所有源文件 (递归搜索 src 目录下所有的 .cpp)
file(GLOB_RECURSE SOURCES "src/*.cpp")

# 4. 生成可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

# 5. 指定头文件包含路径 (核心修改点)
target_include_directories(${PROJECT_NAME} PRIVATE 
    include                # 这样可以用 #include "detector/detector.hpp"
    ${OpenCV_INCLUDE_DIRS} # 显式加入 OpenCV 的头文件路径
)

# 6. 链接库
# 查找 MVS 库
find_library(MVS_LIB MvCameraControl PATHS /opt/MVS/lib/64 /opt/MVS/lib /usr/lib NO_DEFAULT_PATH)
if (MVS_LIB)
    message(STATUS "Found MVS SDK: ${MVS_LIB}")
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${MVS_LIB})
else()
    message(WARNING "MVS SDK not found! Compilation might fail.")
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
endif()
# 查找 MVS 库 (假设在 /opt/MVS/lib/64 或 /opt/MVS/lib)
# find_library(MVS_LIB MvCameraControl PATHS /opt/MVS/lib/64 /opt/MVS/lib /usr/lib)
# if (MVS_LIB)
#     message(STATUS "Found MVS SDK: ${MVS_LIB}")
#     target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${MVS_LIB})
# else()
#     message(WARNING "MVS SDK not found! compilation might fail.")
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
# endif()