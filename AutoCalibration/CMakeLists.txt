cmake_minimum_required( VERSION 3.5.1 )

project(AutoCalibration)

# Compiler flags
if (MSVC)
    set(compiler_options /W4)
    add_definitions(/D_USE_MATH_DEFINES)
else ()
    set(compiler_options -Wall -Wextra -Wpedantic)
endif (MSVC)

# Find libraries
find_package(OpenCV 4.5.0 REQUIRED
        PATHS /usr/local)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(GeographicLib REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        rosbag   rosconsole roscpp   roslib   sensor_msgs  std_msgs )

add_executable(${PROJECT_NAME} main.cpp autoCalibration.cpp autoCalibration.h )

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        Sophus::Sophus
        Eigen3::Eigen
        )

target_compile_options(${PROJECT_NAME} PRIVATE ${compiler_options})
set_target_properties(${PROJECT_NAME} PROPERTIES
        CXX_STANDARD_REQUIRED ON
        CXX_STANDARD 14
        )
