#Author: Zimo Li
#Robot Perception Lab, RI, CMU 
#!/usr/bin/env python

import rospy
import argparse
import os
import rosbag
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from decimal import *
from shutil import copyfile


def main():
    parser = argparse.ArgumentParser(description="Extract timestamps from ROS bags into text file")
    parser.add_argument('-d', action='store', dest='input_folder', help="Folder containing the bag files")
    parser.add_argument('-lt', action='store', dest='lidar_topic', help="Lidar topic")
    parser.add_argument('-it', action='store', dest='img_topic', help="Image topic")
    parser.add_argument('-o', action='store', dest='output_dir', help='Output directory.')
    parser.add_argument('-r', action='store', dest='raw_dir', help='Raw LiDAR scan directory.')
    args = parser.parse_args()  
    
    print "LiDAR topic: %s" % args.lidar_topic
    print "Image topic: %s" % args.img_topic

    cvbridge_obj = CvBridge()
    if not os.path.exists(args.output_dir):
        os.mkdir(args.output_dir)

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
        bag.close()
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
        #print (img_t, lidar_ts[curr_match])
        match_pair.append((img_t, lidar_ts[curr_match]))
        ind_pair.append((ind, curr_match))

    print "%i matches found" % len(match_pair)
    print "Total lidar count %i" % len(lidar_ts)
    # Read image
    match_id = 0
    for file in files:
        bag = rosbag.Bag(os.path.join(args.input_folder,file), "r")
        for topic, msg, t in bag.read_messages(topics = [args.lidar_topic, args.img_topic]):
            curr_t = msg.header.stamp.to_nsec()/1e9
            if topic == args.img_topic and curr_t == match_pair[match_id][0]:
                cv_img = cvbridge_obj.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                cv2.imwrite(os.path.join(args.output_dir, "%06i.png" % match_id), cv_img)
                print "Saving image %i" % match_id
                match_id += 1
        bag.close()
    
    # Read LiDAR 
    match_id = 0
    scans = os.listdir(args.raw_dir)
    scans.sort()
    print "Total LiDAR files %i" % len(scans)
    for file in scans:
        curr_t = Decimal(file[:-4])     

        if abs(curr_t - Decimal(match_pair[match_id][1])) < Decimal(1e-1):
            src = os.path.join(args.raw_dir,file)
            dst = os.path.join(args.output_dir, "%06i.txt" % match_id)
            copyfile(src, dst)
            print "Saving scan %i" % match_id
            match_id += 1

if __name__ == '__main__':
    main()
