cmake_minimum_required(VERSION 3.16)
project(lock)

set(CMAKE_CXX_STANDARD 17)

# 关键：指定动态版 OpenCV 的配置文件路径（优先找动态版，忽略静态版）
set(OpenCV_DIR "/usr/lib/cmake/opencv4")  # 动态版 OpenCV 的默认配置路径

# 查找动态版 OpenCV（包含所需模块）
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d aruco)

# 注释掉错误的 include_directories（指向文件是无效的，且多余）
# include_directories(/usr/include/opencv4/opencv2/aruco.hpp)

if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found!")
endif()

# 包含 OpenCV 头文件（动态版的路径会自动添加）
include_directories(${OpenCV_INCLUDE_DIRS})

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

# 链接动态版 OpenCV 库（${OpenCV_LIBS} 会自动指向 .so 动态库）
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

