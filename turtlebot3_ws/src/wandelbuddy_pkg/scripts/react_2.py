#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib import SimpleActionClient
import serial

# Define the threshold distance as a global variable
THRESHOLD_DISTANCE = 2.0

old_goal = None

# Define the publishers and subscribers as global variables
cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Callback function for receiving current goal information
def current_goal_callback(msg):
    global old_goal
    old_goal = msg

# Subscriber for receiving current goal information
current_goal_sub = rospy.Subscriber('/move_base/current_goal', PoseStamped, current_goal_callback)

# Function to stop the navigation by cancelling the goal
def stop_navigation():
    # Stop the local planner by publishing zero velocity
    print("Try to stop driving")
    global old_goal
    # Save the current goal before cancelling it
    cancel_goal = GoalID()
    cancel_goal_pub.publish(cancel_goal)
    print("Old goal cancelled")

    # Wait until the old goal is updated
    while old_goal is None:
        rospy.sleep(0.1)

    print("Old goal saved")

# Function to resume the navigation by re-setting the old goal
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
        # Immediately check the distance after resuming
        check_and_control_navigation()
    else:
        print("No old goal to resume")

# Function to read the distance from the serial port
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

# Function to check and control navigation based on distance
def check_and_control_navigation():
    # Read the distance from the serial port
    distance = read_distance_from_serial()
    print(f"Distance = {distance}")
    if distance > THRESHOLD_DISTANCE:
        # Stop the navigation if the distance is too large
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
        # Resume the navigation if the distance becomes acceptable again
        print("Keep driving")

    # Publish the UWB data on the ROS topic (if needed)
    pub.publish(str(distance))

if __name__ == '__main__':
    rospy.init_node("uwb_distance_control_node")
    pub = rospy.Publisher("uwb_data", String, queue_size=10)

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
