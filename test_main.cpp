#include "lidarPM.h"
#include "image.h"
#include "Backend.h"
#include <string>




void readImages(const std::string& img_path, std::vector<Image> &img_set)
{
	std::string imgs = img_path + "*.png";
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

int main(int argc, char** argv)
{
	if (argc != 3) {
    	std::cout <<" Usage: ./test_main images_folder lidar_folder" << std::endl;
    	return -1;
    }

    // Parameters
    int downsample = 1; // 1 is not downsampling
    float inlier_ratio = 0.7;


	std::string img_path = argv[1];
	std::string lidar_path = argv[2];

	Lidar lidar_set;
	readLidars(lidar_path, lidar_set);
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
		
		Feature matching
		for(int i=0; i < img_set.size()-1; i++){
			Image& img_first = img_set[i];
			for(int j=i+1; j < img_set.size()-1; j++){
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
	}

	// Add them into graph
	Backend backend;
	gtsam::Pose3 initPrior = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3());
	backend.addFirstPose(0, initPrior);

	for(int i=1;i < lidar_set.size(); i++){
		backend.addNewPose(0, lidar_set.convert2GTSAM(i));
		std::cout << "Add pose for " << i << "th pose" << std::endl;
		backend.solve();
	}

	backend.writePose2File("/home/zimol/Data/test.txt");



}
