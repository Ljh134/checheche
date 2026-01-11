cmake_minimum_required(VERSION 3.16)
project(mapping)

set(CMAKE_CXX_STANDARD 17)


#设置OpenCV静态库路径
set(OpenCV_DIR "/opt/opencv-static/lib/cmake/opencv4")
set(Eigen3_DIR "/usr/share/eigen3/cmake")
find_package(Eigen3 REQUIRED NO_MODULE)


# 查找 OpenCV（需包含 calib3d 模块）
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d aruco)


  #set(CMAKE_BUILD_TYPE Debug) 

if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found!")
endif()

# 包含头文件
include_directories(${OpenCV_INCLUDE_DIRS})
# 手动添加 OpenCV 头文件路径（根据搜索结果修改）
#include_directories(/usr/include/opencv4/opencv2/aruco.hpp)
file(GLOB_RECURSE src "src/*.cpp")

# 生成可执行文件
add_executable(${PROJECT_NAME} )

target_sources(${PROJECT_NAME}
    PRIVATE
        ${src}
)
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ./include
        
)
# 链接 OpenCV 库（必须包含 calib3d）
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
