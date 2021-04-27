cmake_minimum_required(VERSION 3.19)
project(test)


find_package(OpenCV 4.5.0 REQUIRED
        PATHS /home/martin/opencv/build)

message("OpenCV version: ${OpenCV_VERSION}")

set(CMAKE_CXX_STANDARD 14)

set(OpenCV_LIB_DIR
        /home/martin/opencv/build/lib
        )

set(OpenCV_LIBS
        opencv_core
        opencv_highgui
        opencv_imgcodecs
        opencv_features2d
        opencv_xfeatures2d
        )

link_directories(${OpenCV_LIB_DIR})

add_executable(test main.cpp)
target_link_libraries(
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
        )