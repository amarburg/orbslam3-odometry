cmake_minimum_required(VERSION 3.5)
project(orbslam3_odometry)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native
             -Wno-deprecated-declarations
             -Wno-narrowing
             -Wno-overloaded-virtual
             -Wno-reorder
             -Wno-sign-compare
             -Wno-unused-parameter
             -Wno-unused-variable
)
endif()

find_package(ament_cmake REQUIRED)

set(AMENT_DEPENDENCIES
    cv_bridge
    image_transport
    message_filters
    nav_msgs
    ORB_SLAM3
    Pangolin
    Sophus
    rclcpp
    sensor_msgs
    tf2_geometry_msgs
    tf2_ros
    yaml-cpp
)

foreach(Dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(OpenCV REQUIRED COMPONENTS highgui)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

# orbslam3_stereo
#
add_executable(orbslam3_stereo
  src/stereo/stereo.cpp
  src/stereo/stereo_node.cpp
  src/stereo/stereo_rectification.cpp
)
ament_target_dependencies(orbslam3_stereo
  ${AMENT_DEPENDENCIES} )
target_link_libraries(orbslam3_stereo
  ${OpenCV_LIBRARIES})


# orbslam3_mono
#
add_executable(orbslam3_mono
  src/monocular/monocular.cpp
  src/monocular/monocular_node.cpp
)
ament_target_dependencies(orbslam3_mono
  ${AMENT_DEPENDENCIES} )
target_link_libraries(orbslam3_mono
  ${OpenCV_LIBRARIES})

# orbslam3_check_stereo
#
add_executable(orbslam3_check_stereo
  src/stereo/just_check_stereo_calibration.cpp
  src/stereo/stereo_rectification.cpp
)
ament_target_dependencies(orbslam3_check_stereo
  ${AMENT_DEPENDENCIES} )
target_link_libraries(orbslam3_check_stereo
  ${OpenCV_LIBRARIES})

#
# Install targets
#
install(TARGETS
    orbslam3_stereo
    orbslam3_mono
    orbslam3_check_stereo
DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
