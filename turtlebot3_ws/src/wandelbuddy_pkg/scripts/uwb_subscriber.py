#!/usr/bin/env python3
# Example code for ROS Master UWB Data Subscriber

import rospy
from std_msgs.msg import String

def process_uwb_data_msg(data):
    # Process the received UWB data
    print("UWB Data Received: " + str(data))

def create_uwb_data_subscriber():
    # Create a subscriber node for UWB data
    rospy.init_node("uwb_data_sub_node")
    rospy.Subscriber("uwb_data", String, process_uwb_data_msg)
    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    create_uwb_data_subscriber()
