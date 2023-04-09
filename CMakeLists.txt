cmake_minimum_required(VERSION 3.8)
project(rmoss_mindvision_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(rmoss_cam REQUIRED)
find_package(rmoss_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 设置依赖列表
set(dependencies
  rclcpp
  rclcpp_components
  cv_bridge
  OpenCV
  rmoss_interfaces
  rmoss_cam
  rmoss_util
)

# options
option(WITH_MVSDK "Build with MindVisionSDK" ON)

message(STATUS "System architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# MVSDK
if(WITH_MVSDK)
    include_directories("dependencies/MVSDK/include")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(MVSDK_LIB_DIR "dependencies/MVSDK/lib/x64")
        link_directories("dependencies/MVSDK/lib/x64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
        set(MVSDK_LIB_DIR "dependencies/MVSDK/lib/arm64")
        link_directories("dependencies/MVSDK/lib/arm64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(MVSDK_LIB_DIR "dependencies/MVSDK/lib/arm64")
        link_directories("dependencies/MVSDK/lib/arm64")
    else()
        message(FATAL_ERROR "Unkown host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
    endif()
    set(MVSDK_LIB_SO "${PROJECT_SOURCE_DIR}/${MVSDK_LIB_DIR}/libMVSDK.so")
endif()

# Include
include_directories(include)

# mindvision_cam_component
add_library(mindvision_cam_component SHARED
  "src/mindvision_cam.cpp"
  "src/mindvision_cam_node.cpp"
)
if(WITH_MVSDK)
    target_link_libraries(mindvision_cam_component MVSDK pthread)
endif()
ament_target_dependencies(mindvision_cam_component ${dependencies})
# register component nodes
rclcpp_components_register_node(mindvision_cam_component
  PLUGIN "rmoss_mindvision_driver::MindVisionCamNode"
  EXECUTABLE mindvision_cam)

# install cam driver lib
install(
    FILES ${MVSDK_LIB_SO}
    DESTINATION lib
)

# Install libraries
install(TARGETS mindvision_cam_component
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install launch resource config directory
install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
