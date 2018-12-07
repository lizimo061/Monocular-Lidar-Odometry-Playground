// "Copyright [2019] <Zimo Li, Chengqian Che>\"  [legal/copyright]",
#ifndef IMAGE_H_
#define IMAGE_H_
#include <vector>
#include <iostream>
#include <map>
#include <string>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>


struct Landmark{
  cv::Point3f pt;
  int seen = 0;
};

class Image {
  public:
    cv::Mat img;
    cv::Mat desc;
    std::vector<cv::KeyPoint> kp;

    cv::Mat T;  // Pose matrix 4x4
    cv::Mat P;  // Projection matrix 3x4

    using kp_idx_t = size_t;
    using landmark_idx_t = size_t;
    using img_idx_t = size_t;

    Image();
    Image(std::string fileName);
    Image(cv::String fileName);

    void downsampleImg(int rate);

    bool kp_match_exist(size_t kp_idx, size_t img_idx){
      return kp_matches[kp_idx].count(img_idx) > 0;
    }

    landmark_idx_t kp_3d(size_t kp_idx) { return kp_landmark[kp_idx]; }

    kp_idx_t kp_match_idx(size_t kp_idx, size_t img_idx) { return kp_matches[kp_idx][img_idx]; };

    bool kp_3d_exist(size_t kp_idx) { return kp_landmark.count(kp_idx) > 0; }

    void addMatchId(int kp_idx, int img_idx, int dest_kp_idx);

    void addLandmarkId(int kp_idx, int dest_kp_idx);

    void featureExtraction(const cv::Ptr<cv::FeatureDetector>& detector);

    int findKpIdx(size_t idx);

    void release();

  private:
    // keypoint matches in other images
    std::map<kp_idx_t, std::map<img_idx_t, kp_idx_t>> kp_matches;
    std::map<kp_idx_t, landmark_idx_t> kp_landmark;  // keypoint to 3d points
};

#endif  // IMAGE_H_
