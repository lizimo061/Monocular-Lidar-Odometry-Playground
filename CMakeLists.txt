
cmake_minimum_required(VERSION 2.8.9)
project(test)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
#find_package(PCL 1.8 REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAMCMakeTools)
find_package(libpointmatcher REQUIRED)
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")


#Bring the headers, such as Student.h into the project
include_directories(
	include
	${OpenCV_INCLUDE_DIRS} 
	${libpointmatcher_INCLUDE_DIRS}
	/usr/include/eigen3
	/usr/local/include

)

#add_library(lidarPM
#	src/lidarPM.cpp
#)
 
#add_executable(test_lidar test_lidar.cpp)

#target_link_libraries(test_lidar 
#		${Boost_LIBRARIES}
#		${libpointmatcher_LIBRARIES}
#		lidarPM
#		)

#add_library(utils
#	src/lidarPM.cpp
#	src/image.cpp
#	src/Backend.cpp
#)

add_executable(test_main test_main.cpp src/lidarPM.cpp src/image.cpp src/Backend.cpp)

target_link_libraries(test_main
		${OpenCV_LIBS} 
		${Boost_LIBRARIES}
		${libpointmatcher_LIBRARIES}
		gtsam
		)


