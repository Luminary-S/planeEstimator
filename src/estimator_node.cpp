#include "plane_estimator/estimator.h"
// #include "ros/ros.h"

using plane::PlaneEstimator;

int main(int argc, char** argv) {
  // ros::init(argc, argv, "lego_loam");
  ros::init(argc, argv, "plane_estimator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");

  PlaneEstimator estimator(nh, nh_private);

  std::thread loopthread(&PlaneEstimator::performLoop, &estimator);
  // std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread,
  // &MO);

  ros::Rate rate(200);
  while (ros::ok()) {
    ros::spinOnce();

    estimator.run();

    rate.sleep();
  }

  loopthread.join();
  // visualizeMapThread.join();

  return 0;
}