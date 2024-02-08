#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import serial

# Definieer de drempelwaarde als een globale variabele
THRESHOLD_DISTANCE = 2.0

old_goal = None

# Definieer de publishers als globale variabelen
cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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

def stop_navigation():
    # Stop de lokale planner door nul-snelheid te publiceren
    print("try to stop driving")
    #cancel_goal = GoalID()
    #cancel_goal_pub.publish(cancel_goal)
    #twist_msg = Twist()
    #cmd_vel_pub.publish(twist_msg)

    # Stop de lokale planner door nul-snelheid te publiceren
    global old_goal
    print("try to stop driving")
    # Opgeslagen het huidige doel voordat het wordt geannuleerd
    old_goal = rospy.wait_for_message('/move_base/current_goal', PoseStamped)
    cancel_goal = GoalID()
    cancel_goal_pub.publish(cancel_goal)
    print("stopped driving")

def resume_navigation():
    # Hervat de lokale planner door normale snelheid te publiceren
    # Pas dit aan op basis van je bewegingsvereisten
    #twist_msg = Twist()
    #twist_msg.linear.x = 0.22  # Voorbeeld: stel de lineaire snelheid in op 0.2 m/s
    #cmd_vel_pub.publish(twist_msg)
    #current_goal_msg = rospy.wait_for_message('/move_base/current_goal', PoseStamped)
    #goal_msg = MoveBaseGoal()
    #goal_msg.target_pose = current_goal_msg
    #move_base_client.send_goal(goal_msg)
    #move_base_client.wait_for_result()
    #print("navigation resumed")
    global old_goal
    if old_goal is not None:
        print("Resuming navigation with the old goal")
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose = old_goal
        move_base_client.send_goal(goal_msg)
        move_base_client.wait_for_result()
        old_goal = None  # Reset de oude doelvariabele na hervatten
    else:
        print("No old goal to resume")

def check_and_control_navigation():
    # Lees de afstand van de seriele poort
    distance = read_distance_from_serial()
    print(f"Distance = {distance}")
    if distance > THRESHOLD_DISTANCE:
        # Stop de navigatie als de afstand te groot is
        print(f"Warning: Distance bigger than {THRESHOLD_DISTANCE}")
        stop_navigation()
        while(distance > THRESHOLD_DISTANCE):
             distance = read_distance_from_serial()
             print(f"Distance still too big: {distance}")
        rospy.sleep(0.5)
        print(f"Distance is OK again: {distance}")
        resume_navigation()
    else:
        # Hervat de navigatie als de afstand weer acceptabel is
        #resume_navigation()
        print("Keep driving")

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

        #rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # Controleer de afstand en bestuur de navigatie
            check_and_control_navigation()

            #rate.sleep()

    except serial.SerialException as e:
        print(f"Fout bij verbinden met {serial_port}: {e}")
    finally:
        ser.close()
