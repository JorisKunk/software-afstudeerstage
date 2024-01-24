#!/usr/bin/env python3
# Voorbeeldcode voor ROS Master UWB Data Subscriber
import rospy
from std_msgs.msg import String

def process_uwb_data_msg(data):
    print("UWB Data Received: " + str(data))

def create_uwb_data_subscriber():
    rospy.init_node("uwb_data_sub_node")
    rospy.Subscriber("uwb_data", String, process_uwb_data_msg)
    rospy.spin()

if __name__ == '__main__':
    create_uwb_data_subscriber()
