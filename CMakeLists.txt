cmake_minimum_required(VERSION 3.0.2)
project(plane_estimator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  # tf
  roscpp
  rospy
  # cv_bridge
  # image_transport

  # pcl_ros
  # pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  # nav_msgs
  # cloud_msgs
)

find_package(GTSAM REQUIRED QUIET)
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
# find_package(PCL REQUIRED QUIET)
# find_package(OpenCV REQUIRED QUIET)

catkin_package(
  # INCLUDE_DIRS include
  # CATKIN_DEPENDS cloud_msgs
  # DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	# ${PCL_INCLUDE_DIRS}
	# ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  "/usr/include/eigen3"
)

link_directories(
	include
	# ${OpenCV_LIBRARY_DIRS}
	# ${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

# define the plane estimator node
add_executable(plane_estimator 
  src/estimator_node.cpp
  src/estimator.cpp)

target_link_libraries(plane_estimator  ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lgtsam -ltbb)
install(TARGETS plane_estimator  RUNTIME DESTINATION bin)

# define the test node
add_executable(test_gtsam
  src/test/t_gtsam.cpp
)
target_link_libraries(test_gtsam  ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lgtsam -ltbb)
install(TARGETS test_gtsam  RUNTIME DESTINATION bin)
