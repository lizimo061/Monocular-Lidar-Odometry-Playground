#include "lidarPM.h"
using namespace Eigen;

Lidar::Lidar(){
	numOfScans = 0;
}

Lidar::Lidar(std::string fileName){
	//lidarScans 
}

void Lidar::addScan(std::string fileName){
	numOfScans++;
	DP tmp(DP::load(fileName));

	if(lidarScans.size() == 0){
		Matrix3d R = MatrixXd::Identity(3,3);
		Vector3d t;
		t << 0,0,0;
		singleScan newScan = {tmp, R, t};
		lidarScans.push_back(newScan);
	}
	else{
		singleScan last = lidarScans.back();
		// Previous one is input
		// Next one is target
		PM::ICP icp;
		icp.setDefault();
		PM::TransformationParameters T = icp(tmp, last.scan);
		Matrix3d R;
		Vector3d t;
		R << T(0,0), T(0,1), T(0,2), T(1,0), T(1,1), T(1,2), T(2,0), T(2,1), T(2,2);
		t << T(0,3), T(1,3), T(2,3);

		singleScan newScan = {tmp, R, t};
		lidarScans.push_back(newScan);
	}

}

gtsam::Pose3 Lidar::convert2GTSAM(int i){
	if(i >= lidarScans.size()){
		std::cout << "ERROR: Accessing out of range!" << std::endl;
		exit(-1);
	}
	else{
		return gtsam::Pose3(gtsam::Rot3(lidarScans[i].R), gtsam::Point3(lidarScans[i].t));
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