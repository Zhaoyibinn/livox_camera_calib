cmake_minimum_required(VERSION 3.5)
project(livox_camera_calib)

if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
  set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl" CACHE PATH "" FORCE)
  set(Boost_DIR "/usr/lib/x86_64-linux-gnu/cmake/Boost-1.71.0" CACHE PATH "" FORCE)
  set(BOOST_ROOT "/usr" CACHE PATH "" FORCE)
  set(BOOST_INCLUDEDIR "/usr/include" CACHE PATH "" FORCE)
  set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu" CACHE PATH "" FORCE)
  set(Boost_NO_SYSTEM_PATHS ON CACHE BOOL "" FORCE)
  foreach(component system filesystem date_time iostreams regex)
    string(TOUPPER "${component}" component_upper)
    set(Boost_${component_upper}_LIBRARY_RELEASE
      "/usr/lib/x86_64-linux-gnu/libboost_${component}.so" CACHE FILEPATH "" FORCE)
    set(Boost_${component_upper}_LIBRARY_DEBUG
      "/usr/lib/x86_64-linux-gnu/libboost_${component}.so" CACHE FILEPATH "" FORCE)
  endforeach()
  set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4" CACHE PATH "" FORCE)
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "" FORCE)
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "" FORCE)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  #add_compile_options(-Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common io features filters kdtree sample_consensus search segmentation)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(lidar_camera_calib src/lidar_camera_calib.cpp)
add_executable(lidar_camera_multi_calib src/lidar_camera_multi_calib.cpp)
add_executable(bag_to_pcd src/bag_to_pcd.cpp)
add_executable(capture_calib_sample src/capture_calib_sample.cpp)

target_link_libraries(lidar_camera_calib 
  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
)

ament_target_dependencies(lidar_camera_calib
cv_bridge
pcl_conversions
rclcpp
sensor_msgs
std_msgs
  )

target_link_libraries(lidar_camera_multi_calib
  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
)
ament_target_dependencies(lidar_camera_multi_calib
  cv_bridge pcl_conversions rclcpp sensor_msgs std_msgs
)

target_link_libraries(bag_to_pcd ${PCL_LIBRARIES})
ament_target_dependencies(bag_to_pcd
  livox_ros_driver2 pcl_conversions rclcpp rosbag2_cpp sensor_msgs
)

target_link_libraries(capture_calib_sample ${OpenCV_LIBS} ${PCL_LIBRARIES})
ament_target_dependencies(capture_calib_sample
  cv_bridge pcl_conversions rclcpp sensor_msgs
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  lidar_camera_calib
  lidar_camera_multi_calib
  bag_to_pcd
  capture_calib_sample
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
