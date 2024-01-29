#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionClient
import serial

# Definieer de drempelwaarde als een globale variabele
THRESHOLD_DISTANCE = 2.0

old_goal = None

# Definieer de publishers als globale variabelen
cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def current_goal_callback(msg):
    global old_goal
    old_goal = msg

# Voeg de subscriber toe voor het huidige doel
current_goal_sub = rospy.Subscriber('/move_base/current_goal', PoseStamped, current_goal_callback)

def stop_navigation():
    # Stop de lokale planner door nul-snelheid te publiceren
    print("try to stop driving")
    global old_goal
    # Opgeslagen het huidige doel voordat het wordt geannuleerd
    cancel_goal = GoalID()
    cancel_goal_pub.publish(cancel_goal)
    print("old goal cancelled")

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
    else:
        print("No old goal to resume")

def check_and_control_navigation():
    # Lees de afstand van de seriele poort
    distance = read_distance_from_serial()
    if distance is not None and distance > THRESHOLD_DISTANCE:
        print(f"Distance = {distance}")
        # Stop de navigatie als de afstand te groot is
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
        # Hervat de navigatie als de afstand weer acceptabel is
        print("Keep driving")

    # Publiceer de uwb-data op het ROS-topic (als je het nog nodig hebt)
    pub.publish(str(distance))

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

        while not rospy.is_shutdown():
            # Controleer de afstand en bestuur de navigatie
            check_and_control_navigation()

    except serial.SerialException as e:
        print(f"Fout bij verbinden met {serial_port}: {e}")
    finally:
        ser.close()
