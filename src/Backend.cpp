/* /Users/zimoli/Documents/Monocular-Lidar-Odometry-Playground/src/Backend/Backend.cpp
 * @file Backend.cpp
 * @author Zimo Li (zimol@andrew.cmu.edu)
 * @date 2018-10-11
 * @brief Backend for pose estimation
 */


#include "Backend.h"

using gtsam::symbol_shorthand::X; // Pose3, x y z r p y
using gtsam::symbol_shorthand::V; // Velocity, x_dot y_dot z_dot
using gtsam::symbol_shorthand::B; // Bias, ax ay az gx gy gz
using gtsam::symbol_shorthand::L; // Landmarks
using gtsam::symbol_shorthand::K; // Calibration

Backend::Backend(BackendParams bp){
	pose_prior_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());
	mono_meas_noise_ = gtsam::noiseModel::Isotropic::Sigma(2, 2.0);
	point_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(0.1));
	odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());

}

Backend::Backend(){
	pose_prior_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());
	mono_meas_noise_ = gtsam::noiseModel::Isotropic::Sigma(2, 2.0);
	point_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(0.1));
	odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());
	pose_num_ = 0;
	landmark_num_ = 0;
	calib_prior_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(5) << 100, 100, 0.01, 100, 100).finished());
}

void Backend::solve(){
	gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_estimates_); // Can add LM params later
	current_estimates_ = optimizer.optimize();
	initial_estimates_ = current_estimates_;
}

void Backend::initializeK(double fx, double fy, double cx, double cy, double s){
    //gtsam::Cal3_S2::shared_ptr K_tmp(new gtsam::Cal3_S2(fx,fy,s,cx,cy));
    //K_cal = K_tmp;
    gtsam::Cal3_S2 K_tmp(fx,fy,s,cx,cy);
	initial_estimates_.insert(K(0), gtsam::Cal3_S2(fx,fy,s,cx,cy));
	graph_.add(gtsam::PriorFactor<gtsam::Cal3_S2>(K(0), K_tmp, calib_prior_));
}

void Backend::addFirstPose(double timestamp, const gtsam::Pose3& pose){
    
    if (pose_num_ != 0) { std::cout << "Not first pose!\n"; exit(-1); }
    odom_poses_.push_back(pose);
    initial_estimates_.insert(X(pose_num_), pose);
  	graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(pose_num_), pose, pose_prior_));
  	timestamps_.push_back(timestamp);
  	pose_num_++;
}

void Backend::addVisualPose(const gtsam::Pose3& pose){
	initial_estimates_.insert(X(pose_num_), pose);
	pose_num_++;
}

void Backend::addNewPose(double timestamp, const gtsam::Pose3& odom_diff){
	
	//gtsam::Pose3 initial_pose = current_estimates_.at<gtsam::Pose3>(X(pose_num_ - 1))*odom_diff;
	gtsam::Pose3 initial_pose = initial_estimates_.at<gtsam::Pose3>(X(pose_num_ - 1))*odom_diff;
	initial_estimates_.insert(X(pose_num_), initial_pose);
	graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(pose_num_-1), X(pose_num_), odom_diff, odom_noise_));
	pose_num_++;
}

void Backend::addLandMark(double timestamp, const gtsam::Point3& point, int landmark_id){
	graph_.add(gtsam::PriorFactor<gtsam::Point3>(L(landmark_id), point, point_noise_));
	timestamps_.push_back(timestamp);
}

void Backend::addLandMarkInitial(const gtsam::Point3& point, int landmark_id){
	initial_estimates_.insert<gtsam::Point3>(L(landmark_id), point);
	landmark_num_++;
}

void Backend::addPixelMeasurement(double timestamp, const gtsam::Point2& pixel, int pose_id, int landmark_id){
	graph_.add(gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>(pixel, mono_meas_noise_, X(pose_id), L(landmark_id), K(0)));
}

gtsam::Pose3 Backend::getPoseEstimate(int idx){
	return current_estimates_.at<gtsam::Pose3>(X(idx));
}

void Backend::writePose2File(const std::string& fileName){
	std::ofstream outFile;
	outFile.open(fileName);
	for(int i=0;i<pose_num_;i++){
		gtsam::Quaternion rot = getPoseEstimate(i).rotation().toQuaternion(); // w x y z
		gtsam::Point3 trans = getPoseEstimate(i).translation();
		outFile << rot.w() << " " << rot.x() << " " << rot.y() << " " << rot.z() << " " << " " << trans.x() << " " << trans.y() << " " << trans.z() << "\n";
	}
	outFile.close();
	std::cout << "Written poses into " << fileName << std::endl;
}

void Backend::writeLandmark2File(const std::string& fileName){
	std::ofstream outFile;
	outFile.open(fileName);
	current_estimates_.print();	
	for(int i=0;i<landmark_num_;i++){
		gtsam::Point3 pt = current_estimates_.at<gtsam::Point3>(L(i));
		outFile << pt.x() << " " << pt.y() << " " << pt.z() << std::endl;
	}
	outFile.close();
	std::cout << "Written optimized sparse features into " << fileName << std::endl;
}
