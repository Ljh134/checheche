cmake_minimum_required(VERSION 3.10)
project(create_AUcode)

set(CMAKE_CXX_STANDARD 11)

# 查找Eigen3
find_package(Eigen3 REQUIRED)

# 设置OpenCV静态库路径
#set(OpenCV_DIR "/opt/opencv-static/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

file(GLOB_RECURSE src "src/*.cpp")

# 生成可执行文件
add_executable(${PROJECT_NAME} )

target_sources(${PROJECT_NAME}
    PRIVATE
        ${src}
)

# 链接库 - 使用简单的iconv链接
target_link_libraries(create_AUcode 
    ${OpenCV_LIBS}
    Eigen3::Eigen
    # 在Ubuntu上，iconv通常是libc的一部分，但可以单独链接
    # 使用find_library来定位
)

# 包含头文件
target_include_directories(create_AUcode PRIVATE
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)