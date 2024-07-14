cmake_minimum_required(VERSION 3.8)
project(phantom_omni)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

#find_path(OPENHAPTICS_INCLUDE_DIR NAMES HD/hd.h HL/hl.h HDU/hdu.h)

# if(NOT OPENHAPTICS_INCLUDE_DIR)
#   set(OPENHAPTICS_MESSAGE "OPENHAPTICS was not found. If you do not have it you will not be able to use the Phantom Omni.")
#   message(STATUS "${OPENHAPTICS_MESSAGE}") 
#   return()
# endif(NOT OPENHAPTICS_INCLUDE_DIR)

#####################
##  Messages etc.  ##
#####################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OmniButtonEvent.msg"
  "msg/OmniFeedback.msg"
 )
#ament_export_dependencies(rosidl_default_runtime)

#############
##  Build  ##
#############

add_executable(omni src/omni.cpp)
ament_target_dependencies(omni rclcpp tf2_geometry_msgs geometry_msgs tf2_ros std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(omni HL HD HDU "${cpp_typesupport_target}")
target_link_libraries(omni "${cpp_typesupport_target}")

#target_include_directories(omni PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(omni PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS omni
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
