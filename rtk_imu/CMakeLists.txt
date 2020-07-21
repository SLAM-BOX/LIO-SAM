cmake_minimum_required(VERSION 2.8.3)
project(rtk_imu)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs
  robot_localization
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

link_directories(
	include
)

set(THIRDPARTY ${CMAKE_SOURCE_DIR}/rtk_imu/Thirdparty)

## Geographic lib
set(GeographicLib_DIR ${THIRDPARTY}/geographic/lib/cmake/GeographicLib)
find_package(GeographicLib REQUIRED)

add_executable(multiSensorFusion src/multi_sensor_fusion.cpp)

target_include_directories(multiSensorFusion PUBLIC
		${GeographicLib_INCLUDE_DIRS})
target_link_libraries(multiSensorFusion ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})

# add_executable(convertSimlation src/convert_simlation.cpp)
# target_link_libraries(convertSimlation ${catkin_LIBRARIES})

# add_executable(tfRviz src/tf_rviz.cpp)
# target_link_libraries(tfRviz ${catkin_LIBRARIES})

# add_executable(evo src/evo.cpp)
# target_link_libraries(evo ${catkin_LIBRARIES})
