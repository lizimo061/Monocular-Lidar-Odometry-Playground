
cmake_minimum_required(VERSION 2.8.9)
project(test)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

#Bring the headers, such as Student.h into the project
include_directories(
	include
	${OpenCV_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
	/usr/include/eigen3
	/usr/local/include
)

#add_library(lidar
#	src/lidar.cpp
#)
 
#add_executable(test_lidar test_lidar.cpp)

#target_link_libraries(test_lidar 
#		${OpenCV_LIBS} 
#		${PCL_LIBRARIES}
#		${Boost_LIBRARIES}
#		lidar		
#		-lboost_system		
#		)

add_library(utils
	src/lidar.cpp
	src/image.cpp
	src/Backend.cpp
)

add_executable(test_main test_main.cpp)

target_link_libraries(test_main
		${OpenCV_LIBS} 
		${PCL_LIBRARIES}
		${Boost_LIBRARIES}
		utils
		-lboost_system		
		-lgtsam
		)

add_executable(txt2ply utils/txt2ply.cpp)
target_link_libraries(txt2ply
		${OpenCV_LIBS} 
		${PCL_LIBRARIES}
		${Boost_LIBRARIES}
		-lboost_system		
		)

