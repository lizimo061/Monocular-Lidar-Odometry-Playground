/* /Users/zimoli/Documents/Monocular-Lidar-Odometry-Playground/src/Backend/Backend.cpp
 * @file Backend.cpp
 * @author Zimo Li (zimol@andrew.cmu.edu)
 * @date 2018-10-11
 * @brief Backend for pose estimation
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>

#include "Backend.h"

using gtsam::symbol_shorthand::X; // Pose3, x y z r p y
using gtsam::symbol_shorthand::V; // Velocity, x_dot y_dot z_dot
using gtsam::symbol_shorthand::B; // Bias, ax ay az gx gy gz
using gtsam::symbol_shorthand::L; // Landmarks

Backend::Backend(BackendParams bp){
	pose_prior_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());
}

void Backend::solve(){
	result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_estimates_).optimize();
}

void Backend::addFirstPose(double timestamp, const gtsam::Pose3& pose){
    pose_num_++;
    if (pose_num_ != 0) { std::cout << "Not first pose!\n"; exit(-1); }
    odom_poses_.push_back(pose);
    initial_estimates_.insert(X(pose_num_), pose);
  	graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(pose_num_), pose, pose_prior_));
  	timestamps_.push_back(timestamp);
}

void Backend::addNewPose(double timestamp, const gtsam::Pose3& odom_diff, int pose_id){
	pose_num_++;
	gtsam::Pose3 initial_pose = current_estimates_.at<gtsam::Pose3>(X(pose_num_ - 1))*odom_diff;
	initial_estimates_.insert(X(pose_num_), initial_pose);
	graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(pose_num_-1), X(pose_num_), odom_diff, odom_noise_));

}

gtsam::Pose3 Backend::getPoseEstimate(int idx){

}

void Backend::addLandMark(double timestamp, const gtsam::Point3& point, int landmark_id){
	initial_estimates_.insert<gtsam::Point3>(L(landmark_id), point);
	graph_.add(gtsam::PriorFactor<gtsam::Point3>(L(landmark_id), point, point_noise_));
	timestamps_.push_back(timestamp);
}

void Backend::addPixelMeasurement(double timestamp, const gtsam::Point2& pixel, int pose_id, int landmark_id){
	graph_.add(gtsam::GeneralSFMFactor2<Cal3_S2>(pixel, mono_meas_noise_, X(pose_id), L(landmark_id), K));
}