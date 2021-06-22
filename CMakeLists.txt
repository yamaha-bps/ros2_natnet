cmake_minimum_required(VERSION 3.14)
project(natnet_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Sophus REQUIRED)
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

add_library(natnet_component SHARED
  src/natnet_component.cpp
)
target_include_directories(natnet_component
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${geometry_msgs_INCLUDE_DIRS}
)
target_link_libraries(natnet_component
PUBLIC
  natnet_sdk
  rclcpp::rclcpp
  ${geometry_msgs_LIBRARIES}
  Sophus::Sophus
  yaml-cpp
)

add_executable(natnet_node
  src/natnet_node.cpp
)
target_link_libraries(natnet_node
  natnet_component
)

# ---------------------------------------------------------------------------------------
# INSTALLATION
# ---------------------------------------------------------------------------------------

include(GNUInstallDirs)

# Header files
install(
  DIRECTORY    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION  ${CMAKE_INSTALL_INCLUDEDIR}
)

# Third-party shared lib
install(
  FILES                 ${natnet_sdk_content_SOURCE_DIR}/lib/libNatNetLibShared.so
  DESTINATION           ${CMAKE_INSTALL_LIBDIR}
)

# Targets
install(
  TARGETS               natnet_component
  EXPORT                ${PROJECT_NAME}_targets
  LIBRARY DESTINATION   ${CMAKE_INSTALL_LIBDIR}
)

# Executable to run with "ros2 run"
install(
  TARGETS               natnet_node
  EXPORT                ${PROJECT_NAME}_targets
  DESTINATION           ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------------------
# TESTING
# ---------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(ament_cmake QUIET)
if(${ament_cmake_FOUND})
  ament_package()
endif()
