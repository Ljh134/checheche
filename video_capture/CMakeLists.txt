cmake_minimum_required(VERSION 3.8)
project(show_video)
find_package(OpenCV 4 REQUIRED)
file(GLOB_RECURSE src "src/*.cpp")
add_executable(show_video)
target_sources(show_video
    PRIVATE
        ${src}
)
target_include_directories(show_video
    PRIVATE
        ./include
)
target_link_libraries(show_video
    ${OpenCV_LIBS}
)
