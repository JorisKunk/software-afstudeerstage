#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
import serial

# Definieer de drempelwaarde als een globale variabele
THRESHOLD_DISTANCE = 2.0

# Definieer de publishers als globale variabelen
cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
move_base_client = SimpleActionClient('/move_base', MoveBaseAction)

def read_distance_from_serial():
    # Lees uwb-data van de seriële poort
    uwb_data_bytes = ser.readline()

    # Decodeer de bytes naar een Unicode-string
    uwb_data = uwb_data_bytes.decode().strip()

    # Split de uwb_data in timestamp en afstand
    timestamp, distance_str = uwb_data.split(',')

    # Converteer de afstand naar een float
    distance = float(distance_str)

    return distance

def get_robot_current_pose():
    # Implementeer deze functie om de huidige positie van de robot te verkrijgen
    # Dit kan odometrie, "amcl_pose", of een ander geschikt bericht gebruiken
    # Deze functie moet een PoseStamped-bericht retourneren
    pass

def stop_navigation():
    # Stop de lokale planner door nul-snelheid te publiceren
    print("try to stop driving")
    twist_msg = Twist()
    twist_msg.linear.x = 0
    cmd_vel_pub.publish(twist_msg)

def resume_navigation(goal):
    # Hervat de lokale planner door het oorspronkelijke doel in te stellen
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()

def check_and_control_navigation():
    # Lees de afstand van de seriele poort
    distance = read_distance_from_serial()
    print(f"Distance = {distance}")

    global move_base_client

    if distance > THRESHOLD_DISTANCE:
        # Stop de navigatie als de afstand te groot is
        print(f"Warning: Distance bigger than {THRESHOLD_DISTANCE}")
        stop_navigation()

        # Opgeslagen het huidige doel voordat het wordt geannuleerd
        old_goal = rospy.wait_for_message('/move_base/current_goal', PoseStamped)
        cancel_goal = GoalID()
        cancel_goal_pub.publish(cancel_goal)
        rospy.sleep(1.0)  # Wacht even om ervoor te zorgen dat het doel wordt geannuleerd

        print("Setting current pose as the new goal to stop")
        current_pose = get_robot_current_pose()
        if current_pose is not None:
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose = current_pose
            move_base_client.send_goal(move_base_goal)
            move_base_client.wait_for_result()
            print("New goal set successfully to stop")
    else:
        # Hervat de navigatie als de afstand weer acceptabel is
        print("Keep driving")
        move_base_goal = MoveBaseGoal()  # Stel hier je oorspronkelijke doel in
        resume_navigation(move_base_goal)

    # Publiceer de uwb-data op het ROS-topic (als je het nog nodig hebt)
    pub.publish(str(distance))

if __name__ == '__main__':
    rospy.init_node("uwb_distance_control_node")
    pub = rospy.Publisher("uwb_data", String, queue_size=10)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Stel de juiste COM-poort en baudrate in
    serial_port = "/dev/ttyUSB1"  # Vervang dit door de juiste COM-poort
    baud_rate = 115200

    try:
        ser = serial.Serial(serial_port, baud_rate)
        print(f"Verbonden met {serial_port} op {baud_rate} bps")

        move_base_client.wait_for_server()

        while not rospy.is_shutdown():
            # Controleer de afstand en bestuur de navigatie
            check_and_control_navigation()

    except serial.SerialException as e:
        print(f"Fout bij verbinden met {serial_port}: {e}")
    finally:
        ser.close()
