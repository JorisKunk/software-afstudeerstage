#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionClient
import serial

# Define the threshold distance as a global variable
THRESHOLD_DISTANCE = 2.0

old_goal = None

# Define the publishers as global variables
cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def read_distance_from_serial():
    # Read UWB data from the serial port
    uwb_data_bytes = ser.readline()

    # Decode the bytes into a Unicode string
    uwb_data = uwb_data_bytes.decode().strip()

    # Split the uwb_data into timestamp and distance
    timestamp, distance_str = uwb_data.split(',')

    # Convert the distance to a float
    distance = float(distance_str)

    return distance

def stop_navigation():
    # Stop the local planner by publishing zero velocity
    print("try to stop driving")
    global old_goal
    # Save the current goal before canceling it
    #old_goal = rospy.wait_for_message('/move_base/current_goal', PoseStamped)
    print("old goal saved")
    cancel_goal = GoalID()
    cancel_goal_pub.publish(cancel_goal)
    print("old goal cancelled")
    #rospy.sleep(1.0)  # Wait for a while to ensure the goal is canceled

def resume_navigation():
    # Resume the local planner by resetting the old goal
    global old_goal
    if old_goal is not None:
        print("Resuming navigation with the old goal")
        move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
        move_base_client.wait_for_server()
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose = old_goal
        move_base_client.send_goal(goal_msg)
        move_base_client.wait_for_result()
        old_goal = None  # Reset the old goal variable after resuming
        print("Navigation resumed")
    else:
        print("No old goal to resume")

def check_and_control_navigation():
    # Read the distance from the serial port
    distance = read_distance_from_serial()
    print(f"Distance = {distance}")
    if distance > THRESHOLD_DISTANCE:
        # Stop navigation if the distance is too large
        print(f"Warning: Distance bigger than {THRESHOLD_DISTANCE}")
        stop_navigation()
        print("Stopped navigation. Waiting for distance to become acceptable.")
        while distance > THRESHOLD_DISTANCE:
            distance = read_distance_from_serial()
            print(f"Distance still too big: {distance}")
        #rospy.sleep(0.5)
        print(f"Distance is OK again: {distance}")
        resume_navigation()
    else:
        # Resume navigation if the distance becomes acceptable again
        print("Keep driving")

    # Publish the uwb-data on the ROS topic (if needed)
    pub.publish(str(distance))

if __name__ == '__main__':
    rospy.init_node("uwb_distance_control_node")
    pub = rospy.Publisher("uwb_data", String, queue_size=10)
    #cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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
