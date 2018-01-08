#!/usr/bin/env python
import rospy
import serial
import string
import math
import sys
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

degrees2rad = math.pi/180.0
rospy.init_node("imu_node")
default_port='/dev/ttyACM1'
port = rospy.get_param('~port', default_port)
ser = serial.Serial(port=port, baudrate=57600, timeout=1)
pub = rospy.Publisher('imu', Imu, queue_size=1)
imuMsg = Imu()
seq = 0
calib_num = 20
roll_offset = 0
pitch_offset = 0
yaw_offset = 0

rospy.loginfo("Opening %s...", port)

while not rospy.is_shutdown():
    line = ser.readline()
    words = string.split(line,",")
    imuMsg.header.stamp = rospy.Time.now()
    imuMsg.header.frame_id = 'base_link'
    imuMsg.header.seq = seq
    seq = seq + 1
    imuMsg.linear_acceleration.x = float(words[1])
    imuMsg.linear_acceleration.y = float(words[2]) 
    imuMsg.linear_acceleration.z = float(words[3]) 
    (roll, pitch, yaw) = euler_from_quaternion([float(words[5]), float(words[6]), float(words[7]), float(words[4])])
    if seq<=calib_num:
	roll_offset += roll
	pitch_offset += pitch
	yaw_offset += yaw
    else:
	roll = roll - roll_offset/calib_num
	pitch = pitch - pitch_offset/calib_num
	yaw = yaw - yaw_offset/calib_num
	q = quaternion_from_euler(roll,pitch,yaw)
    	imuMsg.orientation.x = q[0]
    	imuMsg.orientation.y = q[1]
    	imuMsg.orientation.z = q[2]
    	imuMsg.orientation.w = q[3]
        pub.publish(imuMsg)
'''
    imuMsg.orientation.x = float(words[5])
    imuMsg.orientation.y = float(words[6])
    imuMsg.orientation.z = float(words[7])
    imuMsg.orientation.w = float(words[4]) 
    pub.publish(imuMsg)

    yaw_deg = float(words[6])
    if yaw_deg > 180.0:
        yaw_deg = yaw_deg - 360.0
    if yaw_deg < -180.0:
        yaw_deg = yaw_deg + 360.0
    yaw = yaw_deg*degrees2rad
    pitch = (180+float(words[4]))*degrees2rad
    roll = float(words[5])*degrees2rad
    q = quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    '''
'''
    imuMsg.angular_velocity.x = float(words[4])
    imuMsg.angular_velocity.y = float(words[5])
    imuMsg.angular_velocity.z = float(words[6])
    imuMsg.header.stamp = float(words[0])
    imuMsg.header.frame_id = 'base_link'
    imuMsg.header.seq = seq
    seq = seq + 1'''


