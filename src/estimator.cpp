#include "plane_estimator/estimator.h"

namespace plane{
PlaneEstimator::PlaneEstimator(ros::NodeHandle &nh, ros::NodeHandle &nh_private) {

}

void PlaneEstimator::init_gtsam() {
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = new ISAM2(parameters);

  // IMU preintegrator
  auto params = PreintegrationParams::MakeSharedU(kGravity);
  params->setAccelerometerCovariance(I_3x3 * 0.1);
  params->setGyroscopeCovariance(I_3x3 * 0.1);
  params->setIntegrationCovariance(I_3x3 * 0.1);
  params->setUse2ndOrderCoriolis(false);
  params->setOmegaCoriolis(Vector3(0, 0, 0));
  accum = new PreintegratedImuMeasurements(params);
  // pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin",
  // 2); pubLaserCloudSurround =

  subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 50,
                                          &PlaneEstimator::imuHandler, this);
  subPlane = nh.subscribe<sensor_msgs::>(planeTopic,50,&PlaneEstimator::planeHandler, this);
}

bool PlaneEstimator::CheckNewNodeCome(){}

void PlaneEstimator::resetParams(){}

void PlaneEstimator::run() {

  if( CheckNewNodeCome ){
    resetParams();
  }

}

void PlaneEstimator::performLoop() {
  /*
      get pose constraint
      */
//   float x, y, z, roll, pitch, yaw;

//   gtsam::Pose3 poseFrom =
//       Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));

//   gtsam::Vector Vector6(6);
//   float noiseScore = 0;
//   Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
//       noiseScore;
//   constraintNoise = noiseModel::Diagonal::Variances(Vector6);
//   /*
//           add constraints
//           */
//   std::lock_guard<std::mutex> lock(mtx);
//   gtSAMgraph.add(
//       BetweenFactor<Pose3>(latestFrameIDLoopCloure, closestHistoryFrameID,
//                            poseFrom.between(poseTo), constraintNoise));
//   isam->update(gtSAMgraph);
//   isam->update();
//   gtSAMgraph.resize(0);

//   aLoopIsClosed = true;
}

void PlaneEstimator::imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn) {
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  imuPointerLast = (imuPointerLast + 1) % imuQueLength;
  imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
}

void PlaneEstimator::planeHandler(const sensor_msgs::LaserEchoConstPtr &planeIn){

}

}