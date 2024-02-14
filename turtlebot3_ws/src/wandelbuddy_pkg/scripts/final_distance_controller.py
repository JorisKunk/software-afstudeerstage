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

# Definieer de drempelwaarde als een globale variabele
THRESHOLD_DISTANCE = 1.70

current_pose = None
start_pose = None
old_goal = None
goal_reached = False
sound_played = False
soundhandle = SoundClient()
started_nav = False
done_navigating = False

def current_pose_callback(msg):
    global current_pose
    current_pose = msg

def save_start_pose():
    global start_pose
    #rospy.sleep(1)  # Wacht een seconde om ervoor te zorgen dat de huidige pose wordt gepubliceerd
    start_pose = current_pose
    if start_pose is None:
        print("couldn't get start pose")
    else:
        print("start pose saved")
    #rospy.sleep(2.0)
# Definieer de publishers als globale variabelen
cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
current_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, current_pose_callback)

def current_goal_callback(msg):
    global old_goal
    old_goal = msg

# Voeg de subscriber toe voor het huidige doel
current_goal_sub = rospy.Subscriber('/move_base/current_goal', PoseStamped, current_goal_callback)

def goal_status_callback(msg):
    global goal_reached
    if msg.status_list:
        latest_status = msg.status_list[-1].status
        if latest_status == 3:  # Goal reached
            goal_reached = True
        else:
            goal_reached = False

# Voeg de subscriber toe voor de doelstatus
goal_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, goal_status_callback)

def play_sound(sound_file):
    global sound_played
    if not sound_played:
        print("playing sound...")
        soundhandle.stopAll()
        soundhandle.playWave(sound_file, volume = 0.7)
        rospy.sleep(3.0)
        print("sound played")
        #sound_played = True


def stop_navigation():
    # Stop de lokale planner door nul-snelheid te publiceren
    print("try to stop driving")
    global old_goal, sound_played
    # Opgeslagen het huidige doel voordat het wordt geannuleerd
    cancel_goal = GoalID()
    cancel_goal_pub.publish(cancel_goal)
    print("old goal cancelled")

    play_sound('/home/ubuntu/workspaces/software-afstudeerstage/turtlebot3_ws/src/wandelbuddy_pkg/Sounds/Attention_call.wav')
    # Wacht tot het oude doel is bijgewerkt
    while old_goal is None:
        rospy.sleep(0.1)

    print("old goal saved")

def resume_navigation():
    # Hervat de lokale planner door het oude doel opnieuw in te stellen
    global old_goal
    if old_goal is not None:
        print("Resuming navigation with the old goal")
        move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
        move_base_client.wait_for_server()
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose = old_goal
        move_base_client.send_goal(goal_msg)
        #move_base_client.wait_for_result()
        old_goal = None  # Reset de oude doelvariabele na hervatten
        print("Navigation resumed")
        sound_played = False  # Reset de variabele zodat het geluid opnieuw kan worden afgespeeld
    else:
        print("No old goal to resume")

def check_and_control_navigation():
    global goal_reached
    global old_goal
    global started_nav
    # Lees de afstand van de seriele poort
    distance = read_distance_from_serial()

    if distance is not None and started_nav == True:
        print(f"Distance = {distance}")
        if distance - 0.1> THRESHOLD_DISTANCE:
            # Stop de navigatie als de afstand te groot is
            print(f"Warning: Distance bigger than {THRESHOLD_DISTANCE}")
            stop_navigation()
            print("Stopped navigation. Waiting for distance to become acceptable.")
            while True:
                distance = read_distance_from_serial()
                if distance is not None:
                    if distance + 0.1 > THRESHOLD_DISTANCE:
                        #play_sound('/home/ubuntu/workspaces/software-afstudeerstage/turtlebot3_ws/src/wandelbuddy_pkg/Sounds/Attention_call.wav')
                        distance = read_distance_from_serial()
                        print(f"Distance still too big: {distance}")
                    else:
                        break
                else:
                    print("no distance data received")
            print(f"Distance is OK again: {distance}")
            resume_navigation()
        elif goal_reached:
            # Speel geluid af als het doel is bereikt
            print("Goal reached. try to play sound:")
            play_sound('/home/ubuntu/workspaces/software-afstudeerstage/turtlebot3_ws/src/wandelbuddy_pkg/Sounds/De bestemming is ber.wav')

            #goal_reached = False  # Reset de variabele na het afspelen van het geluid
            rospy.sleep(5)
            play_sound('/home/ubuntu/workspaces/software-afstudeerstage/turtlebot3_ws/src/wandelbuddy_pkg/Sounds/Ik keer nu terug naa.wav')
            if start_pose is not None:
                print("Returning to start position...")
                move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
                move_base_client.wait_for_server()
                goal_msg = MoveBaseGoal()
                goal_msg.target_pose.header.frame_id = "map"  # Frame ID instellen op "map"
                goal_msg.target_pose.pose = start_pose.pose.pose  # Gebruik de pose uit start_pose
                move_base_client.send_goal(goal_msg)
                move_base_client.wait_for_result()
                print("reached base station")
                #print("returned to start position")
            else:
                print("Start position is not saved. Cannot return to start.")

            # Stop het script na het afspelen van het geluid
            #rospy.signal_shutdown("Goal reached")

            while True:
                if goal_reached == False:
                    print("New navigation started")
                    started_nav = False
                    break
                print("waiting for new navigation goal...")
                rospy.sleep(0.1)
        #else:
            # Hervat de navigatie als de afstand weer acceptabel is
            #print("Keep driving")

        # Publiceer de uwb-data op het ROS-topic (als je het nog nodig hebt)
        pub.publish(str(distance))
    elif distance is None:
        print("Error reading distance. Skipping check and control.")
    elif started_nav == False:
        print("Waiting for navigation goal")
    else:
        print("Unknown error")

    if old_goal is not None and started_nav == False:
        play_sound('/home/ubuntu/workspaces/software-afstudeerstage/turtlebot3_ws/src/wandelbuddy_pkg/Sounds/U kunt mij nu volgen.wav')
        started_nav = True

def read_distance_from_serial():
    try:
        # Lees uwb-data van de seriële poort
        uwb_data_bytes = ser.readline()

        # Decodeer de bytes naar een Unicode-string
        uwb_data = uwb_data_bytes.decode().strip()

        # Split de uwb_data in timestamp en afstand
        timestamp, distance_str = uwb_data.split(',')

        # Converteer de afstand naar een float
        distance = float(distance_str)

        return distance
    except ValueError:
        # Als er een ValueError optreedt, bijvoorbeeld als de data onvolledig is
        print("Received incomplete or invalid data from UWB module.")
        return None

if __name__ == '__main__':
    rospy.init_node("uwb_distance_control_node")
    pub = rospy.Publisher("uwb_data", String, queue_size=10)

    # Stel de juiste COM-poort en baudrate in
    serial_port = "/dev/ttyUSB1"  # Vervang dit door de juiste COM-poort
    baud_rate = 115200

    try:
        ser = serial.Serial(serial_port, baud_rate)
        print(f"Verbonden met {serial_port} op {baud_rate} bps")
        save_start_pose()
        while not rospy.is_shutdown():
            # Controleer de afstand en bestuur de navigatie
            check_and_control_navigation()
            


    except serial.SerialException as e:
        print(f"Fout bij verbinden met {serial_port}: {e}")
    finally:
        ser.close()
