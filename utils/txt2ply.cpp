#include <iostream>
#include <cstdlib>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

void writePLY(std::string out_str, std::string in_str){
	pcl::PointCloud<pcl::PointXYZI> tmp_cloud;
    pcl::PLYWriter ply_writer;

    tmp_cloud.clear();

    std::ifstream file_tmp(in_str.c_str());
    while(true){
        double x,y,z,i;
        file_tmp >> x >> y >> z >> i;
        pcl::PointXYZI pt(i);
        pt.x = x;
        pt.y = y;
        pt.z = z;
        tmp_cloud.push_back(pt);

        if(file_tmp.eof()){
            break;
        }
    }
    ply_writer.write<pcl::PointXYZI>(out_str, tmp_cloud, false);
}

int main(int argc, char ** argv){

    std::string path_prefix = argv[1];

    std::string files_full = path_prefix + "*.txt";
    cv::String lidar_scans(files_full);
    std::vector<cv::String> fn;
    cv::glob(lidar_scans, fn, true);
    for(size_t k=0; k<fn.size(); k++)
    {
    	std::string tmp = std::string(fn[k]);
    	std::string out_name(tmp);
    	out_name.replace(out_name.length()-3,3,"ply");
    	std::cout << "Processing " << tmp << " with output name " << out_name << std::endl;
    	writePLY(tmp, out_name);
    }
    
    return 0;
}