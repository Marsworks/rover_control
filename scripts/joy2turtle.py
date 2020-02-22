#!/usr/bin/env python

"""
Author: Ahmed Abbas
Receives joystick messages (subscribed to Joy topic)
then converts the joysick inputs into ROS commands
axis 1 (left stick vertical) controls linear speed
axis 0 (left stick horizonal) controls angular speed
buttons 4 (LB) toggles the camera left
buttons 5 (RB) toggles the camera right
buttons 6 (Back/Select) changes the mode from locomotion to arm control
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8 


def callback(data):
    tog = UInt8()
    
    if data.buttons[6] == 1.0:
        mode = not mode
        reset()
   
    if mode:
        loco(data)
    elif not mode:
        arm(data)


    if data.buttons[4] == 1.0:
        tog.data = 1
        pub_cam.publish(tog)
    elif data.buttons[5] == 1.0:	
        tog.data = 2
        pub_cam.publish(tog)


def start():
# Intializes everything
    # publishing to "/cmd_vel" and "/toggle_cam"
    global pub_move
    global pub_cam
    
    global mode 
    
    mode = True

    pub_move = rospy.Publisher('/cmd_vel', Twist)
    pub_cam = rospy.Publisher('/toggle_cam', UInt8)
    
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    
    # starts the node
    rospy.init_node('Joy2Serial')
    rospy.spin()


def loco(mov_data,mov_twist):
# Locomotion Control

    twist = Twist()

    twist.linear.x = 4*mov_data.axes[1]
    twist.angular.z = 4*mov_data.axes[0]

    pub_move.publish(twist)


def arm(data):
# Arm Control
    pass


def reset():
# Reset Arm and Loco motors   
    twist = Twist()
    twist.linear.x=0
    twist.angular.z=0
    pub_move.publish(twist)
    # PUT ARM PUBLISHER HERE SET TO DEFAULT


if __name__ == "__main__":
    start()
