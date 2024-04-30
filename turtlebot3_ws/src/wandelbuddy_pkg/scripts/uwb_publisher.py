import rospy
from std_msgs.msg import String
import serial
import time

def uwb_data_pub():
    # Initialize the ROS node for publishing UWB data
    rospy.init_node("uwb_data_pub_node")
    pub = rospy.Publisher("uwb_data", String, queue_size=10)

    # Set the correct COM port and baudrate
    serial_port = "/dev/ttyUSB1"  # Replace this with the correct COM port
    baud_rate = 115200

    try:
        # Connect to the serial port
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"Connected to {serial_port} at {baud_rate} bps")

        rate = rospy.Rate(5)  # Set the publish rate to 5 Hz
        while not rospy.is_shutdown():
            # Read UWB data from the serial port
            uwb_data = ser.readline()

            # Publish the UWB data on the ROS topic
            pub.publish(uwb_data)

            rate.sleep()  # Wait to maintain the publish rate

    except serial.SerialException as e:
        print(f"Error connecting to {serial_port}: {e}")
    finally:
        ser.close()  # Close the serial port

if __name__ == '__main__':
    try:
        uwb_data_pub()
    except rospy.ROSInterruptException:
        pass
