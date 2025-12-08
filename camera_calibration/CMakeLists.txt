cmake_minimum_required(VERSION 3.16)
project(camera_calibration)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV（Ubuntu 22.04默认安装OpenCV 4.x）
find_package(OpenCV REQUIRED)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found! Please install OpenCV first.")
endif()

file(GLOB_RECURSE src "src/*.cpp")

# 包含OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# 生成可执行文件
add_executable(${PROJECT_NAME})


target_sources(${PROJECT_NAME}
    PRIVATE  
        ${src}
)

# 链接OpenCV库
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# 安装可执行文件（可选）
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
