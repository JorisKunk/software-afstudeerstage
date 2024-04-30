import rospy
from std_msgs.msg import String

# Define a callback function to process incoming hello_world messages
def process_hello_world_msg(data):
    print("Message Received: " + str(data))

# Create a subscriber node to receive hello_world messages
def create_subscriber():
    rospy.init_node("hello_world_sub_node")
    rospy.Subscriber("hello_world", String, process_hello_world_msg)

if __name__ == '__main__':
    # Initialize the subscriber node and start spinning
    create_subscriber()
    rospy.spin()
