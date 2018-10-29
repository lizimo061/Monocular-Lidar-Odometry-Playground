// "Copyright [2019] <Zimo Li, Chengqian Che>\"  [legal/copyright]",
#include "image.h"
#include <string>
using namespace cv;
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

int main(int argc, char** argv) {
    if (argc != 2) {
     std::cout <<" Usage: ./match inputImg.jpg" << std::endl;
     return -1;
    }

    Mat image1, image2;
    image1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);

    namedWindow("Display Image", CV_WINDOW_AUTOSIZE);
    imshow("Display Image", image1);

    waitKey(0);
    return 0;

}
