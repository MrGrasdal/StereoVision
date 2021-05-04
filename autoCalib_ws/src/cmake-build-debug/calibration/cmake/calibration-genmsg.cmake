# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "calibration: 3 messages, 0 services")

set(MSG_I_FLAGS "-Icalibration:/home/martin/Code/Master/autoCalib_ws/src/calibration/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(calibration_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg" NAME_WE)
add_custom_target(_calibration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "calibration" "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg" ""
)

get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA.msg" NAME_WE)
add_custom_target(_calibration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "calibration" "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA.msg" "std_msgs/Header:calibration/gnssGGA_status"
)

get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/orientation.msg" NAME_WE)
add_custom_target(_calibration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "calibration" "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/orientation.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration
)
_generate_msg_cpp(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration
)
_generate_msg_cpp(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/orientation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration
)

### Generating Services

### Generating Module File
_generate_module_cpp(calibration
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(calibration_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(calibration_generate_messages calibration_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg" NAME_WE)
add_dependencies(calibration_generate_messages_cpp _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA.msg" NAME_WE)
add_dependencies(calibration_generate_messages_cpp _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/orientation.msg" NAME_WE)
add_dependencies(calibration_generate_messages_cpp _calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_gencpp)
add_dependencies(calibration_gencpp calibration_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration
)
_generate_msg_eus(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration
)
_generate_msg_eus(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/orientation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration
)

### Generating Services

### Generating Module File
_generate_module_eus(calibration
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(calibration_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(calibration_generate_messages calibration_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg" NAME_WE)
add_dependencies(calibration_generate_messages_eus _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA.msg" NAME_WE)
add_dependencies(calibration_generate_messages_eus _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/orientation.msg" NAME_WE)
add_dependencies(calibration_generate_messages_eus _calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_geneus)
add_dependencies(calibration_geneus calibration_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration
)
_generate_msg_lisp(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration
)
_generate_msg_lisp(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/orientation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration
)

### Generating Services

### Generating Module File
_generate_module_lisp(calibration
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(calibration_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(calibration_generate_messages calibration_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg" NAME_WE)
add_dependencies(calibration_generate_messages_lisp _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA.msg" NAME_WE)
add_dependencies(calibration_generate_messages_lisp _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/orientation.msg" NAME_WE)
add_dependencies(calibration_generate_messages_lisp _calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_genlisp)
add_dependencies(calibration_genlisp calibration_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/calibration
)
_generate_msg_nodejs(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/calibration
)
_generate_msg_nodejs(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/orientation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/calibration
)

### Generating Services

### Generating Module File
_generate_module_nodejs(calibration
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/calibration
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(calibration_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(calibration_generate_messages calibration_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg" NAME_WE)
add_dependencies(calibration_generate_messages_nodejs _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA.msg" NAME_WE)
add_dependencies(calibration_generate_messages_nodejs _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/orientation.msg" NAME_WE)
add_dependencies(calibration_generate_messages_nodejs _calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_gennodejs)
add_dependencies(calibration_gennodejs calibration_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration
)
_generate_msg_py(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration
)
_generate_msg_py(calibration
  "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/orientation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration
)

### Generating Services

### Generating Module File
_generate_module_py(calibration
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(calibration_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(calibration_generate_messages calibration_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA_status.msg" NAME_WE)
add_dependencies(calibration_generate_messages_py _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/gnssGGA.msg" NAME_WE)
add_dependencies(calibration_generate_messages_py _calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martin/Code/Master/autoCalib_ws/src/calibration/msg/orientation.msg" NAME_WE)
add_dependencies(calibration_generate_messages_py _calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_genpy)
add_dependencies(calibration_genpy calibration_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(calibration_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(calibration_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(calibration_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/calibration
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(calibration_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(calibration_generate_messages_py std_msgs_generate_messages_py)
endif()
