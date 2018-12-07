#include "lidarPM.h"
using namespace Eigen;

Lidar::Lidar(){
	numOfScans = 0;
	T_l_c << -0.9999, 0.0138, -0.0055, 0.1494,
    		  0.0055, 0.0024, -1.0000, -0.0745,
   			 -0.0138,-0.9999, -0.0025, -0.1248,
   			       0,      0,       0,       1;

}

Lidar::Lidar(std::string fileName){
	//lidarScans 
}

void Lidar::addScan(std::string fileName){
	numOfScans++;
	DP tmp(DP::load(fileName));

	if(lidarScans.size() == 0){
		Matrix3d R = MatrixXd::Identity(3,3);
		Matrix3d R_ref = T_l_c.block<3,3>(0,0);
		Vector3d t;
		t << 0,0,0;
		Vector3d t_ref(T_l_c(0,3), T_l_c(1,3), T_l_c(2,3));
		singleScan newScan = {tmp, R, R_ref, t, t_ref};
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

		// Transform to camera frame
		// R = R_l_c*R;
		// t = R_l_c*t;

		Matrix3d R_ref = last.R_ref*R;
		Vector3d t_ref = last.R_ref*t + last.t_ref;
		singleScan newScan = {tmp, R, R_ref, t, t_ref};
		lidarScans.push_back(newScan);
	}

}

gtsam::Pose3 Lidar::convert2GTSAM(int i){
	if(i >= lidarScans.size()){
		std::cout << "ERROR: Accessing out of range!" << std::endl;
		exit(-1);
	}
	else{
		gtsam::Pose3 pose(gtsam::Rot3(lidarScans[i].R), gtsam::Point3(lidarScans[i].t));
		return pose.inverse();
	}
}

gtsam::Pose3 Lidar::refConvert2GTSAM(int i){
	if(i >= lidarScans.size()){
		std::cout << "ERROR: Accessing out of range!" << std::endl;
		exit(-1);
	}
	else{
		return gtsam::Pose3(gtsam::Rot3(lidarScans[i].R_ref), gtsam::Point3(lidarScans[i].t_ref));
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

 cv::Mat Lidar::eigen2cv(const MatrixXd& mat, int h, int w){
 	cv::Mat tmp;
 	tmp.create(h,w,CV_64F);
 	for(int i=0;i<h;i++){
 		for(int j=0;j<w;j++){
 			tmp.at<double>(i,j) = mat(i,j);
 		}
 	}
 	return tmp;

 }

void Lidar::writeLiDARPose(const std::string& fileName){
	std::ofstream outFile;
	outFile.open(fileName);
	for(int i=0;i < lidarScans.size();i++){
		Matrix3d R = lidarScans[i].R_ref;
		Vector3d t = lidarScans[i].t_ref;
		outFile << R(0,0) << " " << R(0,1) << " " << R(0,2) << " "
		 << R(1,0) << " " << R(1,1) << " " << R(1,2) << " "
		 << R(2,0) << " " << R(2,1) << " " << R(2,2) << " "
		 << t(0) << " " << t(1) << " " << t(2) << std::endl;
	}
	outFile.close();
	std::cout << "Written LiDAR poses to " << fileName << std::endl;
}
