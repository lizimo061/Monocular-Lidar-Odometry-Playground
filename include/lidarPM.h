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
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

using namespace Eigen;


struct singleScan{
  DP scan;
  Matrix3d R; // Transformation from last to current
  Matrix3d R_ref; // Transformation relative to reference frame
  Vector3d t;
  Vector3d t_ref;
};

class Lidar{
  public:
    std::vector<singleScan> lidarScans;

    Lidar();
    Lidar(std::string fileName);
    ~Lidar();

    void addScan(std::string fileName);
    
    gtsam::Pose3 convert2GTSAM(int i);

    gtsam::Pose3 refConvert2GTSAM(int i);
    
    size_t size(){return lidarScans.size();};

    cv::Mat eigen2cv(const MatrixXd& mat, int h, int w);

    void writeLiDARPose(const std::string& fileName);
  
  private:
    int numOfScans;

    void ICPTransform(int source, int target);

    Matrix4d T_l_c; // calibration
};

#endif