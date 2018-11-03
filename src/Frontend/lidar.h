// "Copyright [2019] <Zimo Li, Chengqian Che>\"  [legal/copyright]",
#ifndef LIDAR_H_
#define LIDAR_H_
#include <vector>
#include <iostream>
#include <map>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>


class Lidar{
  public:
    struct singleScan{
      pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
      Eigen::Matrix3d R; // Transformation from current frame to
      Eigen::Vector3d t;
    };
    std::vector<singleScan> lidarScans;

  private:
    Lidar();
    Lidar(std::string dir);
    void addScan(std::string fileName);
    void ICPTransform();
};
