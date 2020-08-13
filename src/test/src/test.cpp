/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample.cpp
 * @brief A 2D Pose SLAM example
 * @date Oct 21, 2010
 * @author Yong Dian Jian
 */

/**
 * A simple 2D pose slam example
 *  - The robot moves in a 2 meter square
 *  - The robot moves 2 meters each step, turning 90 degrees after each step
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have a loop closure constraint when the robot returns to the first position
 */

// In planar SLAM example we use Pose2 variables (x, y, theta) to represent the robot poses
#include <gtsam/geometry/Pose2.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// We will also use a Between Factor to encode the loop closure constraint
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h> 

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// a Gauss-Newton solver
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>
// #inlcude "Eigen.h"

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  // 1. Create a factor graph container and add factors to it
  std::cout<<"main"<<std::endl;
NonlinearFactorGraph graph;
//噪声定义 对角线矩阵：
namespace NM = noiseModel;
cout<<"111111"<<endl;
Vector priorSigmas = ::Vector3(1,1,M_PI);
Vector odoSigmas = Vector3(0.05, 0.01, 0.2);
const NM::Base::shared_ptr // 基类型
  priorNoise = NM::Diagonal::Sigmas(priorSigmas), //prior 对角线噪声
  odoNoise = NM::Diagonal::Sigmas(odoSigmas); // odometry
//   // gaussian = NM::Isotropic::Sigma(1, sigmaR), // non-robust 各项同性噪声
//   // tukey = NM::Robust::Create(NM::mEstimator::Tukey::Create(15), gaussian), //robust 
noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
// //在因子图中加入一个因子
// //   二元因子
graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2, 0, 0     ), model);
// 			//参数解释 ^ ：因子类型  ^边		key 1、2  ^ 边的值			^ 噪声模型
// //   一元因子
graph.emplace_shared<PriorFactor<Pose2> >(1, Pose2(0, 0, 0), priorNoise);
  // graph.addPrior(1, Pose2(0, 0, 0), priorNoise);
cout<<"222"<<endl;
  return 0;
}
