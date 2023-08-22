#!/usr/bin/env python3

import os
import rosbag
import cv2
from cv_bridge import CvBridge

prefix = '/APAslam/slam_0728/slam/'
bagtime = '2023-07-28-16-05-10'
imu_topic = '/imu/data'
img_topic = '/usb_cam/image_raw'

bagfile = prefix + bagtime + '.bag'
imu_file = prefix + bagtime + '-orb-ros1/imu/imu.txt'
img_folder = prefix + bagtime + '-orb-ros1/cam0/img/'
timestamp_file = prefix + bagtime + '-orb-ros1/cam0/timestamp.txt'

# print some info to screen
print('========= Going to read from file: ===========')
print('bagfile: %s'%bagfile)
print('imu_topic: %s'%imu_topic)
print('img_topic: %s'%img_topic)

print('========= Going to write to file: =============')
print('imu_file: %s'%imu_file)
print('img_folder: %s'%img_folder)
print('timestamp_file: %s'%timestamp_file)

        
print('========= Start reading rosbag: =============')
bag = rosbag.Bag(bagfile)

n_imu = bag.get_message_count(topic_filters=[imu_topic])
n_img = bag.get_message_count(topic_filters=[img_topic])
print('Total number of img message: %i,\n Total number of imu message: %i'%(n_img, n_imu))
print('========= Done reading rosbag: =============')

# read imu data from bag and write in imu file
if os.path.exists(imu_file):
    os.remove(imu_file)
os.makedirs(os.path.dirname(imu_file), exist_ok=True)   

imu_no = 0
with open(imu_file, 'a') as imu_file:  
    imu_file.write('#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n')

    for topic, msg, t in bag.read_messages(topics=[imu_topic]):
        # print(t)
        
        if imu_no % 100 == 1:
            print('Imu Received [%i/%i]'%(imu_no, n_imu))
            
        # imu_timestamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        # timestamp = str(imu_timestamp)
        timestamp = str(msg.header.stamp)

        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        datastr = ','.join(map(lambda x:str(x), [timestamp, wx, wy, wz, ax, ay, az]))

        imu_file.write(datastr + '\n')

        imu_no += 1


# read images from rosbag and write to .png
os.makedirs(os.path.dirname(img_folder), exist_ok=True)    

if os.path.exists(timestamp_file):
    os.remove(timestamp_file)
os.makedirs(os.path.dirname(timestamp_file), exist_ok=True)       

bridge = CvBridge()
frame_no = 0
with open(timestamp_file, 'a') as timestamp_file:  

    for topic, msg, t in bag.read_messages(topics=[img_topic]):

        if frame_no % 100 == 1:
            print('Image Received [%i/%i]'%(frame_no, n_img))

        img = bridge.imgmsg_to_cv2(msg)

        # img_timestamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        # timestamp = str(img_timestamp)
        timestamp = str(msg.header.stamp)
        
        # filename = timestamp + '.png'

        cv2.imwrite(img_folder + timestamp + '.png', img)
        timestamp_file.write(timestamp + '\n')
        
        frame_no += 1

bag.close()