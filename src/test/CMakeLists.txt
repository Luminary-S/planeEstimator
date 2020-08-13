cmake_minimum_required(VERSION 3.0.2)
project(test_project)

add_compile_options(-std=c++11)


find_package(GTSAM REQUIRED QUIET)
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
# find_package(PCL REQUIRED QUIET)
# find_package(OpenCV REQUIRED QUIET)

include_directories(
	include
	# ${catkin_INCLUDE_DIRS}
	# ${PCL_INCLUDE_DIRS}
	# ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
#   "/usr/include/eigen3"
)

link_directories(
	include
	# ${OpenCV_LIBRARY_DIRS}
	# ${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

# define test file
add_executable(test_gtsam 
src/imuExample.cpp
# src/test.cpp
#   src/Pose2SLAMExample.cpp
  # src/t_gtsam.cpp
  #src/crcCompute.cpp

  )

# add_executable(test_gtsam src/imageProjection.cpp)
# add_dependencies(test_gtsam ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_gtsam ${Boost_LIBRARIES} gtsam tbb)


# target_link_libraries(test_gtsam   ${Boost_LIBRARIES} -lgtsam -ltbb)
# install(TARGETS test_gtsam  RUNTIME DESTINATION bin)

# target_link_libraries(test ${catkin_LIBRARIES})
