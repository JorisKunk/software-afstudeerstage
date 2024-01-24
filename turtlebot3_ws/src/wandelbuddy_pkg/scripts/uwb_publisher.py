import rospy
from std_msgs.msg import String
import serial
import time


def uwb_data_pub():
    rospy.init_node("uwb_data_pub_node")
    pub = rospy.Publisher("uwb_data", String, queue_size=10)

    # Stel de juiste COM-poort en baudrate in
    serial_port = "/dev/ttyUSB1"  # Vervang dit door de juiste COM-poort
    baud_rate = 115200

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"Verbonden met {serial_port} op {baud_rate} bps")

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # Lees uwb-data van de seriële poort
            uwb_data = ser.readline() #.decode().strip()

            # Publiceer de uwb-data op het ROS-topic
            pub.publish(uwb_data)

            rate.sleep()

    except serial.SerialException as e:
        print(f"Fout bij verbinden met {serial_port}: {e}")
    finally:
        ser.close()


if __name__ == '__main__':
    try:
        uwb_data_pub()
    except rospy.ROSInterruptException:
        pass
