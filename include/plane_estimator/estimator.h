#ifndef ESTIMATOR_H
#define ESTIMATOR_H

// #include "plane_estimator/utility.h"

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <mutex>
#include <thread>

#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserEcho.h"

using namespace gtsam;

namespace plane {
class PlaneEstimator {
 public:
  PlaneEstimator(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

  virtual ~PlaneEstimator() {
    // if (send_motor_cmd_thread_) {
    //   delete send_motor_cmd_thread_;
    // }
  }

  void init_gtsam();

  void run();
  void performLoop();

  void loopThread() {
    if (loopEnableFlag == false) return;

    ros::Rate rate(1);
    while (ros::ok()) {
      rate.sleep();
      performLoop();
    }
  }

  bool CheckNewNodeCome();
  void resetParams();

  void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn);
  void planeHandler(const sensor_msgs::LaserEchoConstPtr &planeIn);

 private:
  NonlinearFactorGraph newGraph;
  Values initialEstimate;
  Values optimizedEstimate;
  ISAM2 *isam;
  Values isamCurrentEstimate;
  // PreintegrationParams *params;
  PreintegratedImuMeasurements *accum;
  // boost::share_ptr<> params;

  noiseModel::Diagonal::shared_ptr priorNoise;
  noiseModel::Diagonal::shared_ptr odometryNoise;
  noiseModel::Diagonal::shared_ptr constraintNoise;

  int imuPointerFront;
  int imuPointerLast;
  string planeTopic = "/plane";
  string imuTopic = "/imu/data";

  bool newFittingLast = 0;
  bool loopEnableFlag = false;
  double mappingProcessInterval = 0.3;
  const double kGravity = 9.81;

  int systemDelay = 0;
  int imuQueLength = 200;

  double imuTime[200];
  float imuRoll[200];
  float imuPitch[200];

  vector<int> surroundingExistingKeyPosesID;
  bool aLoopIsClosed;
  // deque<pcl::PointCloud<PointType>::Ptr> surroundingCornerCloudKeyFrames;
  // PointType previousRobotPosPoint;
  // PointType currentRobotPosPoint;

  std::mutex mtx;

  ros::NodeHandle nh;

  ros::Publisher pubKeyPoses;

  ros::Subscriber subImu, subPlane;
};  // class Estimator

}  // namespace plane

#endif