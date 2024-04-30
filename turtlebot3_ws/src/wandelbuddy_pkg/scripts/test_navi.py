#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
import serial

# Define the threshold distance as a global variable
THRESHOLD_DISTANCE = 2.0

# Define publishers as global variables
cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
move_base_client = SimpleActionClient('/move_base', MoveBaseAction)

def read_distance_from_serial():
    # Read UWB data from the serial port
    uwb_data_bytes = ser.readline()

    # Decode the bytes into a Unicode string
    uwb_data = uwb_data_bytes.decode().strip()

    # Split the UWB data into timestamp and distance
    timestamp, distance_str = uwb_data.split(',')

    # Convert distance to a float
    distance = float(distance_str)

    return distance

def get_robot_current_pose():
    # Implement this function to obtain the current position of the robot
    # This can use odometry, "amcl_pose", or another suitable message
    # This function should return a PoseStamped message
    pass

def stop_navigation():
    # Stop the local planner by publishing zero velocity
    print("try to stop driving")
    twist_msg = Twist()
    twist_msg.linear.x = 0
    cmd_vel_pub.publish(twist_msg)

def resume_navigation(goal):
    # Resume the local planner by setting the original goal
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()

def check_and_control_navigation():
    # Read distance from the serial port
    distance = read_distance_from_serial()
    print(f"Distance = {distance}")

    global move_base_client

    if distance > THRESHOLD_DISTANCE:
        # Stop navigation if distance is too large
        print(f"Warning: Distance bigger than {THRESHOLD_DISTANCE}")
        stop_navigation()

        # Save the current goal before canceling it
        old_goal = rospy.wait_for_message('/move_base/current_goal', PoseStamped)
        cancel_goal = GoalID()
        cancel_goal_pub.publish(cancel_goal)
        rospy.sleep(1.0)  # Wait a bit to ensure the goal is canceled

        print("Setting current pose as the new goal to stop")
        current_pose = get_robot_current_pose()
        if current_pose is not None:
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose = current_pose
            move_base_client.send_goal(move_base_goal)
            move_base_client.wait_for_result()
            print("New goal set successfully to stop")
    else:
        # Resume navigation if distance becomes acceptable again
        print("Keep driving")
        move_base_goal = MoveBaseGoal()  # Set your original goal here
        resume_navigation(move_base_goal)

    # Publish the UWB data on the ROS topic (if needed)
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

        move_base_client.wait_for_server()

        while not rospy.is_shutdown():
            # Check distance and control navigation
            check_and_control_navigation()

    except serial.SerialException as e:
        print(f"Error connecting to {serial_port}: {e}")
    finally:
        ser.close()
