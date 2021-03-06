/**
 * @file Backend.h
 * @author Zimo Li (zimol@andrew.cmu.edu)
 * @date 2018-10-11
 * @brief Backend for pose estimation
 */

#ifndef BACKEND_H
#define BACKEND_H

#include <iostream>
#include <fstream>
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
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include "image.h"


typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;
typedef gtsam::PinholePose<gtsam::Cal3_S2> Camera;


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
	Backend();

	Backend(BackendParams bp);

	void solve();

	void addFirstPose(double timestamp, const gtsam::Pose3& pose);

	void addNewPose(double timestamp, const gtsam::Pose3& odom_diff);

	void addNewPose(double timestamp, const gtsam::Pose3& odom_diff, const gtsam::Pose3& init_pose);

	void addVisualPose(const gtsam::Pose3& pose);
	
	void addLandMark(double timestamp, const gtsam::Point3& point, int landmark_id);

	void addLandMarkInitial(const gtsam::Point3& point, int landmark_id);

	void addPixelMeasurement(double timestamp, const gtsam::Point2& pixel, int pose_id, int landmark_id);

	void addSmartFactors(std::vector<Image>& img_set, size_t landmark_size);

	void intrinsicsInit(gtsam::Cal3_S2 K);

	gtsam::Pose3 getPoseEstimate(int idx);

	bool existLandmark(int idx);

	void writePose2File(const std::string& fileName);

	void writeLandmark2File(const std::string& fileName);

	double printError();

	
	gtsam::NonlinearFactorGraph graph_;

	gtsam::noiseModel::Diagonal::shared_ptr pose_prior_, mono_meas_noise_, point_noise_, calib_prior_, odom_noise_;

private:
	int pose_num_;
	int landmark_num_;
	std::vector<int> landmark_ids_;
	std::vector<double> timestamps_;
	std::vector<gtsam::Pose3> odom_poses_;
	std::vector<SmartFactor::shared_ptr> smartfactors_ptr;

	BackendParams bp_;

	gtsam::ISAM2 isam2_;
	gtsam::GaussNewtonParams GN_params_;
	gtsam::LevenbergMarquardtParams LM_params_;
	
	gtsam::Values initial_estimates_, current_estimates_;
	

	gtsam::Cal3_S2::shared_ptr K_cal;
};

#endif