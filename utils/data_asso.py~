#Author: Zimo Li
#Robot Perception Lab, RI, CMU 
#!/usr/bin/env python

import rospy
import argparse
import os
import rosbag
import sys

def main():
	parser = argparse.ArgumentParser(description="Extract timestamps from ROS bags into text file")
	parser.add_argument('-d', action='store', dest='input_folder', help="Folder containing the bag files")
	parser.add_argument('-lt', action='store', dest='lidar_topic', help="Lidar topic")
	parser.add_argument('-it', action='store', dest='img_topic', help="Image topic")
	args = parser.parse_args()	
	
	print "LiDAR topic: %s" % args.lidar_topic
	print "Image topic: %s" % args.img_topic
	
	files = [f for f in os.listdir(args.input_folder)]
	files.sort()

	img_ts = []
	lidar_ts = []
	for file in files:
		bag = rosbag.Bag(os.path.join(args.input_folder,file), "r")
		for topic, msg, t in bag.read_messages(topics = [args.lidar_topic, args.img_topic]):
			if topic == args.lidar_topic:
				lidar_ts.append(msg.header.stamp.to_nsec()/1e9)
			elif topic == args.img_topic:
				img_ts.append(msg.header.stamp.to_nsec()/1e9)
	
	# Finish timestamps collecting
	# Lidar more than img

	match_pair = []
	ind_pair = []

	 
	for ind in range(len(img_ts)):
		img_t = img_ts[ind]
		curr_match = 0
		curr_min = 999
		for ld_ind in range(len(lidar_ts)):
			lidar_t = lidar_ts[ld_ind]
			dist = abs(lidar_t - img_t)
			if dist < curr_min:
				curr_match = ld_ind
				curr_min = dist
		match_pair.append((img_t, lidar_ts[curr_match]))
		ind_pair.append((ind, curr_match))

	print ind_pair
		


if __name__ == '__main__':
	main()
