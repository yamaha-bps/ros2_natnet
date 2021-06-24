cmake_minimum_required(VERSION 3.14)
project(natnet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---------------------------------------------------------------------------------------
# VENDOR LIBRARY
# ---------------------------------------------------------------------------------------

include(FetchContent)
fetchcontent_declare(natnet_sdk_content
  URL https://s3.amazonaws.com/naturalpoint/software/NatNetSDKLinux/ubuntu/NatNet_SDK_3.1_ubuntu.tar
)
fetchcontent_makeavailable(natnet_sdk_content)

add_library(natnet_sdk SHARED IMPORTED)
target_include_directories(natnet_sdk INTERFACE ${natnet_sdk_content_SOURCE_DIR}/include)
set_target_properties(natnet_sdk PROPERTIES IMPORTED_LOCATION ${natnet_sdk_content_SOURCE_DIR}/lib/libNatNetLibShared.so)

# ---------------------------------------------------------------------------------------
# TARGETS
# ---------------------------------------------------------------------------------------

add_library(natnet_lib SHARED
  src/natnet_node.cpp
)
target_include_directories(natnet_lib
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${geometry_msgs_INCLUDE_DIRS}
)
target_link_libraries(natnet_lib
PUBLIC
  natnet_sdk
  rclcpp::rclcpp
  ${geometry_msgs_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
)

rclcpp_components_register_node(natnet_lib
  PLUGIN     "cbr::NatnetNode"
  EXECUTABLE natnet_node
)

# ---------------------------------------------------------------------------------------
# INSTALLATION
# ---------------------------------------------------------------------------------------

install(
  TARGETS               natnet_lib natnet_node
  EXPORT                ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION   lib
  LIBRARY DESTINATION   lib
)

install(
  FILES                 ${natnet_sdk_content_SOURCE_DIR}/lib/libNatNetLibShared.so
  DESTINATION           lib
)

install(
  DIRECTORY     include
)

# ---------------------------------------------------------------------------------------
# TESTING
# ---------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake)
  add_launch_test(test/bringup.test.py TIMEOUT 60)
endif()

# ---------------------------------------------------------------------------------------
# CMAKE CONFIGURATION
# ---------------------------------------------------------------------------------------

ament_export_targets(${PROJECT_NAME}_targets)

ament_package()

