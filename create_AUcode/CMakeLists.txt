cmake_minimum_required(VERSION 3.16)
project(create_AUcode)

set(CMAKE_CXX_STANDARD 17)

# 查找 OpenCV（需包含 calib3d 模块）
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d aruco)

if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found!")
endif()

# 包含头文件
include_directories(${OpenCV_INCLUDE_DIRS})
# 手动添加 OpenCV 头文件路径（根据搜索结果修改）
#include_directories(/usr/include/opencv4/opencv2/aruco.hpp)

# 生成可执行文件
add_executable(${PROJECT_NAME} src/create_AUcode.cpp)

# 链接 OpenCV 库（必须包含 calib3d）
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
