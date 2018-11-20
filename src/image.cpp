// "Copyright [2019] <Zimo Li, Chengqian Che>\"  [legal/copyright]",
#include "image.h"
#include <string>

using namespace cv;
using namespace cv::xfeatures2d;
Image::Image() {
    /*** TODO ***/
}

Image::Image(std::string fileName) {
    img = imread(fileName);
}

void Image::downsampleImg(int rate) {
    resize(img, img, img.size()/rate);
}

void Image::featureExtraction(const Ptr<FeatureDetector> detector) {
    detector->detect(img, kp);
    detector->compute(img, kp, desc);
}

// int main(int argc, char** argv) {
//     if (argc != 3) {
//      std::cout <<" Usage: ./match inputImg1.jpg inputImg2.jpg" << std::endl;
//      return -1;
//     }
//     int minHessian = 400;
//     Mat image1, image2, image1_wk, image2_wk;
//     image1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
//     image2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

//     Ptr<SURF> detector = SURF::create();
//     detector->setHessianThreshold(minHessian);
//     std::vector<KeyPoint> keypoints_1, keypoints_2;
//     Mat descriptors_1, descriptors_2;
//     detector->detectAndCompute(image1, Mat(), keypoints_1, descriptors_1);
//     detector->detectAndCompute(image2, Mat(), keypoints_2, descriptors_2);
//     drawKeypoints(image1, keypoints_1, image1_wk, Scalar(0, 0, 255));
//     drawKeypoints(image2, keypoints_2, image2_wk, Scalar(255, 0, 0));
//     // circle(image1, Point(100, 100), 20, Scalar(0, 0, 255),CV_FILLED, 8, 0);
//     namedWindow("Display Image1", CV_WINDOW_AUTOSIZE);
//     imshow("Display Image1", image1_wk);
//     namedWindow("Display Image2", CV_WINDOW_AUTOSIZE);
//     imshow("Display Image2", image2_wk);
//     waitKey(0);
//     return 0;

// }
