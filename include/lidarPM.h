// "Copyright [2019] <Zimo Li, Chengqian Che>\"  [legal/copyright]",
#ifndef LIDARPM_H_
#define LIDARPM_H_
#include <vector>
#include <iostream>
#include <map>
#include <string>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include "pointmatcher/PointMatcher.h"


typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

using namespace Eigen;


struct singleScan{
  DP scan;
  Matrix3d R; // Transformation from last to current
  Vector3d t;
};

class Lidar{
  public:
    std::vector<singleScan> lidarScans;

    Lidar();
    Lidar(std::string fileName);
    ~Lidar();

    void addScan(std::string fileName);
    
    gtsam::Pose3 convert2GTSAM(int i);
    
    size_t size(){return lidarScans.size();};
  
  private:
    int numOfScans;

    void ICPTransform(int source, int target);
};

#endif