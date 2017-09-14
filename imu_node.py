#!/usr/bin/env python

from sensor_msgs.msg import Imu
import rospy
import serial
#import numpy as np
import re
import sys
import scipy
from scipy.constants import g #gravitational acceleration 9.80665

#var = var.rstrip('character to strip')
#float64(var) should ignore /n
try:
    ser = serial.Serial(
        port = '/dev/ttyACM2',
        baudrate = 9600,
        parity = serial.PARITY_ODD,
        stopbits = serial.STOPBITS_TWO,
        bytesize = serial.SEVENBITS
    )
    print 'connected to imu 9dof'
except serial.serialutil.SerialException:
    print "No device found"
    sys.exit()

pub = rospy.Publisher('imu', Imu, queue_size=10)
rospy.init_node('imu_9dof', anonymous=True)
#rate = rospy.Rate(10) # 10hz

imu = Imu()
pi = scipy.pi

def main():
    seq = 0
    while ser.isOpen():
        var = ser.readline()
        var_ = re.split(', ', var)
        
        imu.orientation.w = float(var_[7])
        imu.orientation.x = float(var_[8])
        imu.orientation.y = float(var_[9])
        imu.orientation.z = float(var_[10])
        
        #convert angular degree to radians
        imu.angular_velocity.x = (float(var_[4])*pi/180)
        imu.angular_velocity.y = (float(var_[5])*pi/180)
        imu.angular_velocity.z = (float(var_[6])*pi/180)
        
        #convert to m/s^2
        imu.linear_acceleration.x = float(var_[1])*g
        imu.linear_acceleration.y = float(var_[2])*g
        imu.linear_acceleration.z = float(var_[3])*g - 10
        
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = 'base_imu_link'
        imu.header.seq = seq
        seq += 1
        
        pub.publish(imu)
        
        #rate.sleep()
main()
