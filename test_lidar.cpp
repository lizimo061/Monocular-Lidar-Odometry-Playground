#include "lidar.h"

int main(int argc, char** argv)
{
	std::string frame_1 = "../data/frame_1.pcd";
	std::string frame_2 = "../data/frame_2.pcd";
	Lidar allScans;

	allScans.addScan(frame_1);
	allScans.addScan(frame_2);
	std::cout << "R: \n" << allScans.lidarScans[1].R << std::endl;
	std::cout << "t: \n" << allScans.lidarScans[1].t << std::endl; 
	return 0;

}
