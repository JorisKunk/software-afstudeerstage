import rospy
from std_msgs.msg import String

#simple publisher to test the workings of it
def hello_world_pub():
    #initialize publisher with a name
    rospy.init_node("hello_world_pub_node")
    pub = rospy.Publisher("hello_world", String, queue_size = 10)

    rate = rospy.Rate(5)
    i = 0
    while not rospy.is_shutdown():
        pub.publish("Hello World" + str(i))
        i +=1
        rate.sleep()


if __name__ == '__main__':
    try:
        hello_world_pub()

    except rospy.ROSInterruptException:
        pass
