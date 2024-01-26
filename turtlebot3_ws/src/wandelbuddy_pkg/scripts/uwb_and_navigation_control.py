#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import serial


THRESHOLD_DISTANCE = 2.0
RPRATE = 20

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

def check_and_control_navigation():
    # Lees de afstand van de seriele poort
    distance = read_distance_from_serial()

    print(f"Distance: {distance}")

    if distance > THRESHOLD_DISTANCE:
        print(f"Warning, distance bigger than {THRESHOLD_DISTANCE}: {distance}")
        # Stop de navigatie als de afstand te groot is
        rospy.wait_for_service('/move_base/pause')
        print("service move_base connected")
        try:
            pause_navigation = rospy.ServiceProxy('/move_base/pause', Empty)
            pause_navigation()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    else:
        # Hervat de navigatie als de afstand weer acceptabel is
        #rospy.wait_for_service('/move_base/resume')
        try:
            resume_navigation = rospy.ServiceProxy('/move_base/resume', Empty)
            resume_navigation()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    # Publiceer de uwb-data op het ROS-topic (als je het nog nodig hebt)
    pub.publish(str(distance))

if __name__ == '__main__':
    rospy.init_node("uwb_distance_control_node")
    pub = rospy.Publisher("uwb_data", String, queue_size=10)

    # Stel de juiste COM-poort en baudrate in
    serial_port = "/dev/ttyUSB1"  # Vervang dit door de juiste COM-poort
    baud_rate = 115200

    try:
        ser = serial.Serial(serial_port, baud_rate)
        print(f"Verbonden met {serial_port} op {baud_rate} bps")

        rate = rospy.Rate(RPRATE)
        while not rospy.is_shutdown():
            # Controleer de afstand en bestuur de navigatie
            check_and_control_navigation()

            rate.sleep()

    except serial.SerialException as e:
        print(f"Fout bij verbinden met {serial_port}: {e}")
    finally:
        ser.close()
