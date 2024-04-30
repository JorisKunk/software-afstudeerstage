#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
import serial

# Define the threshold distance as a global variable
THRESHOLD_DISTANCE = 2.0

# Define the publishers as global variables
cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def read_distance_from_serial():
    # Read UWB data from the serial port
    uwb_data_bytes = ser.readline()

    # Decode the bytes into a Unicode string
    uwb_data = uwb_data_bytes.decode().strip()

    # Split the UWB data into timestamp and distance
    timestamp, distance_str = uwb_data.split(',')

    # Convert the distance to a float
    distance = float(distance_str)

    return distance

def stop_navigation():
    # Stop the local planner by setting zero velocity
    print("Stopping navigation")
    twist_msg = Twist()
    twist_msg.linear.x = 0  # Set linear velocity to 0 m/s
    cmd_vel_pub.publish(twist_msg)
    print("Stopped driving")

def resume_navigation():
    # Resume the local planner by setting normal velocity
    twist_msg = Twist()
    twist_msg.linear.x = 0.22  # Set linear velocity to 0.2 m/s (example)
    cmd_vel_pub.publish(twist_msg)

def check_and_control_navigation():
    # Read the distance from the serial port
    distance = read_distance_from_serial()
    print(f"Distance = {distance}")
    if distance > THRESHOLD_DISTANCE:
        # Stop navigation if the distance is too large
        print(f"Warning: Distance bigger than {THRESHOLD_DISTANCE}")
        stop_navigation()
        while distance > THRESHOLD_DISTANCE:
            distance = read_distance_from_serial()
            rospy.sleep(1)  # Wait for 1 second
        print(f"Distance is OK again: {distance}")
        resume_navigation()
    else:
        # Continue navigation if the distance is acceptable
        print("Keep driving")

    # Publish the UWB data on the ROS topic
    pub.publish(str(distance))

if __name__ == '__main__':
    rospy.init_node("uwb_distance_control_node")
    pub = rospy.Publisher("uwb_data", String, queue_size=10)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set the correct COM port and baudrate
    serial_port = "/dev/ttyUSB1"  # Replace this with the correct COM port
    baud_rate = 115200

    try:
        ser = serial.Serial(serial_port, baud_rate)
        print(f"Connected to {serial_port} at {baud_rate} bps")

        while not rospy.is_shutdown():
            # Check the distance and control the navigation
            check_and_control_navigation()

    except serial.SerialException as e:
        print(f"Error connecting to {serial_port}: {e}")
    finally:
        ser.close()
