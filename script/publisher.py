import rospy
import random
from std_msgs.msg import Int32

def current_position_callback(msg):
    rospy.loginfo(f"Current_Position: {msg.data}")

def main():
    rospy.init_node('Position_ROS_example', anonymous=True)

    pub = rospy.Publisher('Request_Position', Int32, queue_size=10)

    rospy.Subscriber('/Current_Position', Int32, current_position_callback)

    rate = rospy.Rate(1) 

    while not rospy.is_shutdown():
        position_request = Int32()
        position_request.data = random.randint(-10000, 10000)

        rospy.loginfo(f"--------------Publishing Request_Position: {position_request.data}-------------")
        pub.publish(position_request)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass