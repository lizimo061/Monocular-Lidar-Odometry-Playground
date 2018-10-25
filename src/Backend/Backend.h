/**
 * @file Backend.h
 * @author Zimo Li (zimol@andrew.cmu.edu)
 * @date 2018-10-11
 * @brief Backend for pose estimation
 */

#ifndef BACKEND_H
#define BACKEND_H

#include <iostream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

enum class Solver
{
	LM,
	ISAM2
};

struct BackendParams
{
	Solver solver;
 	bool diagonalDamping;
 	int relinearizeSkip;
 	double relinearizeThreshold, lambdaInitial;
	//TODO: Other params
};


class Backend
{
public:
	Backend(BackendParams bp);

	void solve();

	void addFirstPose(double timestamp, const gtsam::Pose3& pose);

	void addNewPose(double timestamp, const gtsam::Pose3& odom_pose, const gtsam::Pose3& odom_diff);

	gtsam::Pose3 getPoseEstimate(int idx);
	
	//TODO: More functions

private:
	int pose_num_;
	std::vector<double> timestamps_;
	std::vector<gtsam::Pose3> odom_poses_;

	BackendParams bp_;

	gtsam::ISAM2 isam2_;
	gtsam::GaussNewtonParams GN_params_;
	gtsam::LevenbergMarquardtParams LM_params_;
	gtsam::NonlinearFactorGraph graph_;
	gtsam::Values initial_estimates_, current_estimates_;
	gtsam::noiseModel::Diagonal::shared_ptr pose_prior_;

		
};

#endif