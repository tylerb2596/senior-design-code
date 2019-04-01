# SBD Metallic Debris Collector Project
# Stanley Black and Decker
# Georgia Institute of Technology
# created by Tyler Brown

#This is a python file used to communicate with an external GUI to start and
#stop the robot

import rospy
import socket
from std_msgs.msg import String

control_socket = 4200

def user_control():

    #publisher to send messages
    pub = rospy.Publisher('/state_machine/user_control', String, queue_size=10)

    #initialize the rosnode
    rospy.init_node('user_control', anonymous=True)

    #slow down the loop
    rate = rospy.Rate(10) # 10hz

    #create a socket to use in recieving messages
    user_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    user_socket.bind(('', control_socket))

    #infinite loop used to recieve UDP packets and publish them to ROS
     while not rospy.is_shutdown():
        message, address = user_socket.recvfrom(1024)
        message = message.lower()
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        user_control()
    except rospy.ROSInterruptException:
        pass