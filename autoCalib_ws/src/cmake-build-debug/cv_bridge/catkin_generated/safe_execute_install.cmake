execute_process(COMMAND "/home/martin/Code/Master/autoCalib_ws/src/cmake-build-debug/cv_bridge/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/martin/Code/Master/autoCalib_ws/src/cmake-build-debug/cv_bridge/catkin_generated/python_distutils_install.sh) returned error code ")
endif()