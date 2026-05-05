cmake_minimum_required(VERSION 3.10)
project(RadarVision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE 
    include
    include/MVS
    ${OpenCV_INCLUDE_DIRS}
)

find_package(Threads REQUIRED)

# 海康机器人 MVS SDK
find_library(MVS_LIB MvCameraControl PATHS /opt/MVS/lib/64 /opt/MVS/lib /usr/lib NO_DEFAULT_PATH)
if (MVS_LIB)
    message(STATUS "Found Hikrobot MVS SDK: ${MVS_LIB}")
else()
    message(WARNING "Hikrobot MVS SDK not found!")
endif()

# 迈德威视 MindVision SDK
find_library(MVSDK_LIB MVSDK PATHS /lib /usr/lib)
if (MVSDK_LIB)
    message(STATUS "Found MindVision SDK: ${MVSDK_LIB}")
else()
    message(WARNING "MindVision SDK not found!")
endif()

set(CAMERA_LIBS "")
if (MVS_LIB)
    list(APPEND CAMERA_LIBS ${MVS_LIB})
endif()
if (MVSDK_LIB)
    list(APPEND CAMERA_LIBS ${MVSDK_LIB})
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${CAMERA_LIBS} Threads::Threads)
