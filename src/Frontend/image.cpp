#include <image.h>

Image::Image(){
    /*** TODO ***/
}

Image::Image(std::string fileName){
    img = cv::imread(fileName);
}

void Image::downsampleImg(int rate){
    cv::resize(img, img, img.size()/rate);
}

void Image::featureExtraction(const cv::Ptr<cv::FeatureDetector> detector){
    detector->detect(img, kp);
    detector->compute(img, kp, desc);
}