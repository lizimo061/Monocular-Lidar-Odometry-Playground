// "Copyright [2019] <Zimo Li, Chengqian Che>\"  [legal/copyright]",
#ifndef LIDAR_H_
#define LIDAR_H_
#include <vector>
#include <iostream>
#include <map>
#include <string>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>

using namespace Eigen;

struct singleScan{
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud;
  Matrix3d R; // Transformation from current frame to
  Vector3d t;
};

class Lidar{
  public:
    std::vector<singleScan> lidarScans;

    Lidar();
    Lidar(std::string fileName);
    ~Lidar();
    void addScan(std::string fileName);

  private:
    int numOfScans;

    void ICPTransform(int source, int target);
};

#endif