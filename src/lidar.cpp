#include "lidar.h"
#include <pcl/io/ply_io.h>
using namespace Eigen;

Lidar::Lidar(){
	numOfScans = 0;
}

Lidar::Lidar(std::string fileName){
	//lidarScans 
}

void Lidar::addScan(std::string fileName){
	numOfScans++;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PLYReader reader;
	reader.read(fileName, *cloud);
	if(lidarScans.size() == 0){
		Matrix3d R = MatrixXd::Identity(3,3);
		Vector3d t;
		t << 0,0,0;
		singleScan newScan = {cloud, R, t};
		lidarScans.push_back(newScan);
	}
	else{
		singleScan last = lidarScans.back();
		pcl::PointCloud<pcl::PointXYZI> cloud_aligned;
		// Previous one is input
		// Next one is target
		pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
		icp.setInputSource(last.raw_cloud);
		icp.setInputTarget(cloud);
		icp.align(cloud_aligned);
		MatrixX4f T = icp.getFinalTransformation();
		Matrix3d R;
		Vector3d t;
		R << T(0,0), T(0,1), T(0,2), T(1,0), T(1,1), T(1,2), T(2,0), T(2,1), T(2,2);
		t << T(0,3), T(1,3), T(2,3);

		singleScan newScan = {cloud, R, t};
		lidarScans.push_back(newScan);
	}

}

void Lidar::ICPTransform(int source, int target){

}

Lidar::~Lidar(){
	// for(int i=0;i<numOfScans;i++){
	// 	lidarScans[i].raw_cloud->clear();
	// }
	lidarScans.clear();
}