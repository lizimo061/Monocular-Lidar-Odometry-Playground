/**
 * @file Backend.h
 * @author Zimo Li (zimol@andrew.cmu.edu)
 * @date 2018-10-11
 * @brief Backend for pose estimation
 */

#ifndef BACKEND_H
#define BACKEND_H

#include <iostream>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/nonlinear/Marginals.h>
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

	void addNewPose(double timestamp, const gtsam::Pose3& odom_diff, int pose_id);

	gtsam::Pose3 getPoseEstimate(int idx);
	
	void addLandMark(double timestamp, const gtsam::Point3& point, int landmark_id);

	void addPixelMeasurement(double timestamp, const gtsam::Point2& pixel, int pose_id, int landmark_id);

	void initializeK(double fx, double fy, double cx, double cy, double s);



private:
	int pose_num_;
	std::vector<double> timestamps_;
	std::vector<gtsam::Pose3> odom_poses_;

	BackendParams bp_;

	gtsam::ISAM2 isam2_;
	gtsam::GaussNewtonParams GN_params_;
	gtsam::LevenbergMarquardtParams LM_params_;
	gtsam::NonlinearFactorGraph graph_;
	gtsam::Values initial_estimates_, current_estimates_, result;
	gtsam::noiseModel::Diagonal::shared_ptr pose_prior_, mono_meas_noise_, point_noise_, calib_prior_, odom_noise_;

	gtsam::Cal3_S2::shared_ptr K_cal;
};

#endif