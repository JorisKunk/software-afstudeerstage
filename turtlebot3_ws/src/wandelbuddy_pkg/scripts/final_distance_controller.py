#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionClient
import serial
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import time

# Distance from which the robot will stop and wait until the person is within range again
THRESHOLD_DISTANCE = 4

current_pose = None
start_pose = None
old_goal = None
goal_reached = False
soundhandle = SoundClient()
started_nav = False
done_navigating = False
# Volume at which the speaker on the robot plays. 1.0 equals 100%. 0.0 equals 0%
speaker_volume = 0.7

# Callback function to get the current position information from the robot
def current_pose_callback(msg):
    global current_pose
    current_pose = msg

# Function to save the start position so the robot can return to it's base station to start new navigation 
def save_start_pose():
    global start_pose
    start_pose = current_pose
    if start_pose is None:
        rospy.loginfo("couldn't get start pose %s", start_pose)
    else:
        rospy.loginfo("start pose saved")

# Define the publishers
cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
current_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, current_pose_callback)

# Get the current goal information of navigation
def current_goal_callback(msg):
    global old_goal
    old_goal = msg

# Define the subsriber for the current goal
current_goal_sub = rospy.Subscriber('/move_base/current_goal', PoseStamped, current_goal_callback)

# Callback function to retreive the goal status
def goal_status_callback(msg):
    global goal_reached
    if msg.status_list:
        latest_status = msg.status_list[-1].status
        if latest_status == 3:  # 3 equals goal reached
            goal_reached = True
        else:
            goal_reached = False

# Define the subscriber for the goal status
goal_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, goal_status_callback)

# Play the sound from a given sound file
def play_sound(sound_file):
    global speaker_volume
    rospy.loginfo("playing sound...")
    soundhandle.stopAll()
    soundhandle.playWave(sound_file, volume = speaker_volume)
    rospy.sleep(3.0)
    rospy.loginfo("sound played")

# Stop the robot from driving to it's goal by cancelling the goal. The cancelled goal is saved so the robot can resume navigating to it later when navigation is resumed
def stop_navigation():
    rospy.loginfo("try to stop driving")
    global old_goal, sound_played
    cancel_goal = GoalID()
    cancel_goal_pub.publish(cancel_goal)
    rospy.loginfo("old goal cancelled")
    play_sound('/home/ubuntu/workspaces/software-afstudeerstage/turtlebot3_ws/src/wandelbuddy_pkg/Sounds/Attention_call.wav')
    while old_goal is None:
        rospy.sleep(0.1)
    rospy.loginfo("old goal saved")

# Resume the navigation to the old goal
def resume_navigation():
    global old_goal
    if old_goal is not None:
        rospy.loginfo("Resuming navigation with the old goal")
        move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
        move_base_client.wait_for_server()
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose = old_goal
        move_base_client.send_goal(goal_msg)
        old_goal = None
        rospy.loginfo("Navigation resumed")
    else:
        rospy.logwarn("No old goal to resume")

# Main function which handles distance control while robot is navigating to the goal
def check_and_control_navigation():
    global goal_reached
    global old_goal
    global started_nav

    distance = read_distance_from_serial()

    if distance is not None and started_nav == True:
        rospy.loginfo("Distance = %f", distance)
        rospy.sleep(0.05)

        if distance - 0.1> THRESHOLD_DISTANCE:
            rospy.logwarn("Warning: Distance bigger than %f", THRESHOLD_DISTANCE)
            stop_navigation()
            rospy.loginfo("Stopped navigation. Waiting for distance to become acceptable.")
            while True:
                distance = read_distance_from_serial()
                if distance is not None:
                    if distance + 0.1 > THRESHOLD_DISTANCE:
                        distance = read_distance_from_serial()
                        rospy.loginfo("Distance still too big: %f", distance)
                    else:
                        break
                else:
                    rospy.logwarn("No distance data received")
            rospy.loginfo("Distance is OK again: %f", distance)
            resume_navigation()
        elif goal_reached:
            rospy.loginfo("Goal reached. try to play sound...")
            play_sound('/home/ubuntu/workspaces/software-afstudeerstage/turtlebot3_ws/src/wandelbuddy_pkg/Sounds/De bestemming is ber.wav')
            rospy.sleep(5)
            if start_pose is not None:
                play_sound('/home/ubuntu/workspaces/software-afstudeerstage/turtlebot3_ws/src/wandelbuddy_pkg/Sounds/Ik keer nu terug naa.wav')
                rospy.loginfo("Returning to start position...")
                move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
                move_base_client.wait_for_server()
                goal_msg = MoveBaseGoal()
                goal_msg.target_pose.header.frame_id = "map"
                goal_msg.target_pose.pose = start_pose.pose.pose
                move_base_client.send_goal(goal_msg)
                move_base_client.wait_for_result()
                rospy.loginfo("reached base station")
            else:
                rospy.logwarn("Start position is not saved. Cannot return to start.")

            while True:
                if goal_reached == False:
                    rospy.loginfo("New navigation started")
                    started_nav = False
                    break
                rospy.loginfo("waiting for new navigation goal...")
                rospy.sleep(0.1)
    elif distance is None:
        rospy.logwarn("Couldn't read distance from serial port, trying to receive new data...")
    elif started_nav == False:
        rospy.loginfo("Waiting for navigation goal")
        rospy.sleep(0.5)

    else:
        rospy.logerr("Unknown error")

    if old_goal is not None and started_nav == False:
        play_sound('/home/ubuntu/workspaces/software-afstudeerstage/turtlebot3_ws/src/wandelbuddy_pkg/Sounds/U kunt mij nu volgen.wav')
        started_nav = True

# Read the distance data coming from the serial port
def read_distance_from_serial():
    try:
        ser.flushInput()
        uwb_data_bytes = ser.readline()
        uwb_data = uwb_data_bytes.decode().strip()
        timestamp, distance_str = uwb_data.split(',')
        distance = float(distance_str)
        return distance
    except ValueError:
        # Als er een ValueError optreedt, bijvoorbeeld als de data onvolledig is
        rospy.logerr("Received incomplete or invalid data from UWB module.")
        return None

# main function which connects the serial port, tries to save the start position of the robot and starts the loop for the navigation control
if __name__ == '__main__':
    rospy.init_node("uwb_distance_control_node")
    rospy.loginfo("starting distance navigation control")
    serial_port = "/dev/ttyUSB1"
    baud_rate = 115200

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout = 0.1)
        rospy.loginfo("%s Connected with serial port with following baud rate :%s", serial_port, baud_rate)
        save_start_pose()
        while not rospy.is_shutdown():
            check_and_control_navigation()

    except serial.SerialException as e:
        rospy.loginfo("%s Fout bij verbinden met %s ", serial_port,e)
    finally:
        ser.close()
