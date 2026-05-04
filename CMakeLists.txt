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

find_library(MVS_LIB MvCameraControl PATHS /opt/MVS/lib/64 /opt/MVS/lib /usr/lib NO_DEFAULT_PATH)
if (MVS_LIB)
    message(STATUS "Found MVS SDK: ${MVS_LIB}")
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${MVS_LIB} Threads::Threads)
else()
    message(WARNING "MVS SDK not found! Compilation might fail.")
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} Threads::Threads)
endif()
