/**
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

Backend::Backend(BackendParams bp){

}

void Backend::solve(){

}

void Backend::addFirstPose(double timestamps, const gtsam::Pose3& pose){
    pose_num_++;
    if (pose_num_ != 0) { std::cout << "Not first pose!\n"; exit(-1); }
    // TODO
}

void addNewPose(double timestamp, const gtsam::Pose3& odom_pose, const gtsam::Pose3& odom_diff){

}

gtsam::Pose3 getPoseEstimate(int idx){

}

