#include "lidarPM.h"

int main(int argc, char** argv)
{
	std::string frame_1(argv[1]);
	std::string frame_2(argv[2]);
	Lidar allScans;

	allScans.addScan(frame_1);
	allScans.addScan(frame_2);
	std::cout << "R: \n" << allScans.lidarScans[1].R << std::endl;
	std::cout << "t: \n" << allScans.lidarScans[1].t << std::endl; 
	return 0;

}

// #include "pointmatcher/PointMatcher.h"
// #include <string>


// typedef PointMatcher<float> PM;
// typedef PM::DataPoints DP;

// int main(int argc, char** argv)
// {
// 	DP d1(DP::load(argv[1]));
// 	DP d2(DP::load(argv[2]));
// 	PM::ICP icp;
// 	icp.setDefault();

// 	PM::TransformationParameters T = icp(d1,d2);

// 	std::cout << "trans:\n" << T << std::endl;


// 	return 0;

// }