#include "lidarPM.h"
#include "image.h"
#include "Backend.h"
#include <string>




void readImages(const std::string& img_path, std::vector<Image> &img_set)
{
	std::string imgs = img_path + "*.jpg";
	cv::String cv_imgs(imgs);
	std::vector<cv::String> fn;
	cv::glob(cv_imgs, fn, true);
	for(size_t k=0;k<fn.size();++k)
	{
		Image image_tmp(fn[k]);
		img_set.push_back(image_tmp);
		std::cout << "Load image " << k << std::endl;
	}

}

void readLidars(const std::string& scan_path, Lidar& lidar_set)
{
	std::string scans = scan_path + "*.ply";
	cv::String scans_cvstr(scans);
	std::vector<cv::String> fn;
	cv::glob(scans_cvstr, fn, true);
	for(size_t k=0;k<fn.size();++k)
	{
		lidar_set.addScan(std::string(fn[k]));
		std::cout << "Load scan " << k << std::endl;
	}	
}

void writeLandmark2File(const std::string& txt_path, std::vector<Landmark> lm)
{
	std::ofstream outFile;
	outFile.open(txt_path);
	for(int i=0;i<lm.size();i++){
		Landmark lm_tmp = lm[i];
		outFile << lm_tmp.pt.x << " " << lm_tmp.pt.y << " " << lm_tmp.pt.z << std::endl;
	}
	outFile.close();
	std::cout << "Written sparse points before optimization to " << txt_path << std::endl;

}

void writeCamPose2File(const std::string& txt_path, std::vector<Image> cams)
{
	std::ofstream outFile;
	outFile.open(txt_path);
	for(int i=0; i<cams.size(); i++){
		Image tmp = cams[i];
		outFile << tmp.T.at<double>(0,0) << " " << tmp.T.at<double>(0,1) << " " << tmp.T.at<double>(0,2) << " "
			<< tmp.T.at<double>(1,0) << " " << tmp.T.at<double>(1,1) << " " << tmp.T.at<double>(1,2) << " "
			<< tmp.T.at<double>(2,0) << " " << tmp.T.at<double>(2,1) << " " << tmp.T.at<double>(2,2) << " " 
			<< tmp.T.at<double>(0,3) << " " << tmp.T.at<double>(1,3) << " " << tmp.T.at<double>(2,3) << " " << std::endl;
	}
	outFile.close();
	std::cout << "Written pose into " << txt_path << std::endl;
}

int main(int argc, char** argv)
{
	if (argc != 3) {
    	std::cout <<" Usage: ./test_main images_folder lidar_folder" << std::endl;
    	return -1;
    }

    // Parameters
    int downsample = 1; // 1 is not downsampling
    float inlier_ratio = 0.7;
    int min_landmark_seen = 3;

	double fx = 719;
	double fy = fx;
	double cx = 0;
	double cy = 0;

	std::string img_path = argv[1];
	std::string lidar_path = argv[2];

	Lidar lidar_set;
	//readLidars(lidar_path, lidar_set);
	std::vector<Image> img_set; // Images set
	std::vector<Landmark> l_set; // Landmark set
	// Read in the images
	readImages(img_path, img_set);
		
	{
		using namespace cv;

		// Features extraction
		Ptr<AKAZE> feature = AKAZE::create();
		Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
		for(int i=0;i<img_set.size();i++){
			Image& current = img_set[i];
			current.downsampleImg(downsample);
			current.featureExtraction(feature);
			std::cout << "Feature extraction for image " << i << std::endl;
		}
		
		// Feature matching
		for(int i=0; i < (img_set.size()-1); i++){
			Image& img_first = img_set[i];
			for(int j=i+1; j < img_set.size(); j++){
				Image& img_second = img_set[j];
				std::vector<std::vector<DMatch>> matches;
				std::vector<Point2f> src, dst;
				std::vector<int> first_kp, second_kp;
				std::vector<uchar> mask;

				matcher->knnMatch(img_first.desc, img_second.desc, matches, 2);
				for(int m_ind = 0; m_ind < matches.size(); m_ind++){
					std::vector<DMatch> m_temp = matches[m_ind];

					if(m_temp[0].distance < inlier_ratio*m_temp[1].distance){
						src.push_back(img_first.kp[m_temp[0].queryIdx].pt);
						dst.push_back(img_second.kp[m_temp[0].trainIdx].pt);
										
						first_kp.push_back(m_temp[0].queryIdx);
						second_kp.push_back(m_temp[0].trainIdx);
					}
				}
				// Use fundamental matrix to rule out outliers
				findFundamentalMat(src, dst, FM_RANSAC, 3, 0.99, mask);
			
				for(int k = 0; k < mask.size(); k++){
					if(mask[k]){
						img_first.addMatchId(first_kp[k], j, second_kp[k]);
						img_second.addMatchId(second_kp[k], i, first_kp[k]);
					}
				}

				std::cout << "Feature matching: " << i << " " << j << " has " << sum(mask)[0] << " good matches." << std::endl;

			}
		}

		// Triangulate and recover the 3D points

		Mat K_intr = Mat::eye(3,3,CV_64F);
		// double fx = 1359.38;
		// double fy = fx;
		// double cx = 1395.56;
		// double cy = 1105.93;

		K_intr.at<double>(0,0) = fx; // fx
		K_intr.at<double>(1,1) = fy; // fy
		K_intr.at<double>(0,2) = cx; // cx
		K_intr.at<double>(1,2) = cy; // cy

		Point2d pp(cx,cy);

		std::cout << "Initial K: \n" << K_intr << std::endl;

		img_set[0].T = Mat::eye(4,4,CV_64F);
		img_set[0].P = K_intr * Mat::eye(3,4,CV_64F);

		for(int i=0; i<img_set.size()-1; i++){
			Image& prev = img_set[i];
			Image& curr = img_set[i+1];

			std::vector<Point2f> src,dst;
			std::vector<size_t> kp_used;

			for(int k=0; k<prev.kp.size(); k++){
				if(prev.kp_match_exist(k, i+1)){
					size_t match_idx = prev.kp_match_idx(k, i+1);
					src.push_back(prev.kp[k].pt);
					dst.push_back(curr.kp[match_idx].pt);

					kp_used.push_back(k);
				}
			}
			
			// Debug
			// std::cout << "dst size: " << dst.size() << std::endl;
			// std::cout << "src size: " << src.size() << std::endl;

			// Recover the poses from dst to src
			Mat mask;
			Mat E = findEssentialMat(dst, src, fx, pp, RANSAC, 0.999, 1.0, mask);
			Mat local_R, local_t;
			recoverPose(E, dst, src, local_R, local_t, fx, pp, mask);
			Mat T = Mat::eye(4, 4, CV_64F);
        	local_R.copyTo(T(Range(0, 3), Range(0, 3)));
			local_t.copyTo(T(Range(0, 3), Range(3, 4)));

			curr.T = prev.T*T;

			Mat R = curr.T(Range(0, 3), Range(0, 3));
        	Mat t = curr.T(Range(0, 3), Range(3, 4));

        	Mat P(3, 4, CV_64F);

        	P(Range(0, 3), Range(0, 3)) = R.t();
        	P(Range(0, 3), Range(3, 4)) = -R.t()*t;
        	P = K_intr*P;

			curr.P = P;		

			Mat points4D;
			triangulatePoints(prev.P, curr.P, src, dst, points4D);

			for(int j=0; j<kp_used.size(); j++){
				if(mask.at<uchar>(j)){
					size_t k = kp_used[j];
					size_t match_idx = prev.kp_match_idx(k, i+1);

					Point3f pt3d;
					pt3d.x = points4D.at<float>(0, j) / points4D.at<float>(3, j);
                    pt3d.y = points4D.at<float>(1, j) / points4D.at<float>(3, j);
					pt3d.z = points4D.at<float>(2, j) / points4D.at<float>(3, j);

					if(prev.kp_3d_exist(k)){
						curr.addLandmarkId(match_idx, prev.kp_3d(k));
						l_set[prev.kp_3d(k)].pt += pt3d;
						l_set[curr.kp_3d(match_idx)].seen++;
					}
					else{
						Landmark lm;
						lm.pt = pt3d;
						lm.seen = 2;
						l_set.push_back(lm);

						prev.addLandmarkId(k, l_set.size()-1);
						curr.addLandmarkId(match_idx, l_set.size()-1);
					}
				}
			}
		}

		for(int j=0;j<l_set.size();j++){
			Landmark& l = l_set[j];
			if (l.seen >= 3) {
                l.pt /= (l.seen - 1);
			}
		}
	}
	
	std::cout << "In total " << l_set.size() << " points are observed\n" << std::endl;
	
	writeLandmark2File("/home/zimol/Data/sparse.txt", l_set);
	writeCamPose2File("/home/zimol/Data/poses.txt", img_set);

	// Add them into graph
	Backend backend;
	backend.initializeK(fx,fy,0,cx,cy);

	gtsam::Pose3 initPrior = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3());
	

	for(int i=0;i < img_set.size(); i++){
		// if(i==0){
		// 	backend.addFirstPose(0, initPrior);

		// }
		// else{
		// 	backend.addNewPose(0, lidar_set.convert2GTSAM(i));
		// }
		
		//std::cout << "Add pose for " << i << "th pose" << std::endl;


		// Test bundle adjustment
		Image curr_img = img_set[i];

		gtsam::Rot3 R(
                curr_img.T.at<double>(0,0),
                curr_img.T.at<double>(0,1),
                curr_img.T.at<double>(0,2),

                curr_img.T.at<double>(1,0),
                curr_img.T.at<double>(1,1),
                curr_img.T.at<double>(1,2),

                curr_img.T.at<double>(2,0),
                curr_img.T.at<double>(2,1),
                curr_img.T.at<double>(2,2)
		);

		gtsam::Point3 t;
		t(0) = curr_img.T.at<double>(0,3);
        t(1) = curr_img.T.at<double>(1,3);
		t(2) = curr_img.T.at<double>(2,3);

		gtsam::Pose3 pose(R,t);

		std::cout << pose << std::endl;

		if(i==0){
			backend.addFirstPose(0, pose);
		}
		else{
			backend.addVisualPose(pose);
		}


		for(int j=0;j < curr_img.kp.size(); j++){
			if(curr_img.kp_3d_exist(j)){
				size_t lm_id = curr_img.kp_3d(j);

				if(l_set[lm_id].seen >= min_landmark_seen){
					gtsam::Point2 pt;

					pt(0) = curr_img.kp[j].pt.x;
					pt(1) = curr_img.kp[j].pt.y;

					backend.addPixelMeasurement(0, pt, i, lm_id);
				}
			}
		}

		//backend.solve();
	}

	bool init_prior = false;
	for(int i=0;i < l_set.size(); i++){
		if(l_set[i].seen >= min_landmark_seen){
			cv::Point3f p = l_set[i].pt;

			backend.addLandMarkInitial(gtsam::Point3(p.x,p.y,p.z), i);

			if(!init_prior){
				init_prior = true;
				backend.addLandMark(0, gtsam::Point3(p.x,p.y,p.z), i);
			}
		}
	}
	backend.solve();
	backend.writePose2File("/home/zimol/Data/test.txt");
	backend.writeLandmark2File("/home/zimol/Data/opt_sparse.txt");
}
