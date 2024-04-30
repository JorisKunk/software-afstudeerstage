#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import serial


THRESHOLD_DISTANCE = 2.0
RPRATE = 20

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

def check_and_control_navigation():
    # Read the distance from the serial port
    distance = read_distance_from_serial()

    print(f"Distance: {distance}")

    if distance > THRESHOLD_DISTANCE:
        print(f"Warning, distance bigger than {THRESHOLD_DISTANCE}: {distance}")
        # Stop navigation if the distance is too large
        rospy.wait_for_service('/move_base/pause')
        print("Service move_base connected")
        try:
            pause_navigation = rospy.ServiceProxy('/move_base/pause', Empty)
            pause_navigation()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    else:
        # Resume navigation if the distance becomes acceptable again
        # rospy.wait_for_service('/move_base/resume')
        try:
            resume_navigation = rospy.ServiceProxy('/move_base/resume', Empty)
            resume_navigation()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

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

        rate = rospy.Rate(RPRATE)
        while not rospy.is_shutdown():
            # Check the distance and control navigation
            check_and_control_navigation()

            rate.sleep()

    except serial.SerialException as e:
        print(f"Error connecting to {serial_port}: {e}")
    finally:
        ser.close()
