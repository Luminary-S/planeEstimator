#ifndef _UTILITY_H_
#define _UTILITY_H_


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

// #include <opencv/cv.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace std;


extern const string planeTopic = "/sonic";
extern const string imuTopic = "/imu/data";

// // Save pcd
// extern const string fileDirectory = "/tmp/";

// Using velodyne cloud "ring" channel for image projection (other lidar may have different name for this channel, change "PointXYZIR" below)
extern const bool useCloudRing = true; // if true, ang_res_y and ang_bottom are not used


extern const bool loopEnableFlag = false;
extern const double mappingProcessInterval = 0.3;

extern const int systemDelay = 0;
extern const int imuQueLength = 200;


struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};


#endif
