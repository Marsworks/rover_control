#!/usr/bin/env python

"""
Author: Ahmed Abbas
Receives joystick messages (subscribed to Joy topic)
then converts the joysick inputs into ROS commands

One-stick mode:
axis 1 (left stick vertical) controls linear speed
axis 0 (left stick horizonal) controls angular speed

Two-stick mode:
axis 1 (left stick vertical) controls L motor
axis 4 (right stick vertical) controls R motor

buttons 4 (LB) toggles the camera left
buttons 5 (RB) toggles the camera right
buttons 6 (Back/Select) changes the mode from locomotion to arm control and vice-versa

TODO: Set-up enums for the joystick positions

"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, Float32



class Joy2Rover:
    
    def __init__(self, name='Joy2Rover', stick_type=True):
        """
        Initialise the joy2rover node

        Parameters
        -----------
        name: str
                Name of the node
        
        stick_type: bool
                Set to True to use two sticks to control the rover or one
        """
        
        # initialise the node
        rospy.init_node(name)

        self.node_name = rospy.get_name()
        rospy.loginfo("Started node: %s", self.node_name)
        
        stick_type = rospy.get_param("~stick_type", True)
        
        self.mode = True # TODO: rename this
        self.two_stick = stick_type 
        
        
        # Only start the correct publishers
        if self.two_stick:
            rospy.loginfo("Node: %s running with 2 analog sticks!", self.node_name)
            self.pub_left_vel = rospy.Publisher('lwheel_vtarget', Float32, queue_size=10)
            self.pub_right_vel = rospy.Publisher('rwheel_vtarget', Float32, queue_size=10)
            self.left = 0
            self.right = 0
        else:
            rospy.loginfo("Node: %s running with 1 analog sticks, Twist message will be published!", self.node_name)
            self.pub_move = rospy.Publisher('/cmd_vel', Twist)
        
        # Start the cam publisher
        self.pub_cam = rospy.Publisher('/toggle_cam', UInt8, queue_size=10)

        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, self.callback)
        
        
    def spin(self):
        """
        Starts the main loop
        """
        rospy.spin()

    def callback(self, data):
        """
        Joy callback function

        Parameters
        -----------
        data: :class: Joy
                The joystick message to convert to twist
        """
        
        # Camera toggle
        tog = UInt8()
        
        # Select button -> change mode
        if data.buttons[6] == 1.0:
            self.mode = not self.mode # TODO: set roslog
            self.reset()
    
        if self.mode:
            self.loco(data)
        else:
            self.arm(data)


        if data.buttons[4] == 1.0:
            tog.data = 1
            self.pub_cam.publish(tog)
        elif data.buttons[5] == 1.0:	
            tog.data = 2
            self.pub_cam.publish(tog)
    
    
    def reset(self):
        """
        Resets the arm and locomotion motors to their default position
        """

        # Check analog stick mode
        if self.two_stick:
            self.right = 0
            self.left = 0
        else:
            twist = Twist()
            twist.linear.x=0
            twist.angular.z=0
            self.pub_move.publish(twist)
        
        # TODO: PUT ARM PUBLISHER HERE SET TO DEFAULT

    def loco(self, joy_data):
        """
        Parses the joy stick data and publishes either Twist or L/R motor velocities
        depending on the stick_type

        Parameters
        ----------
        joy_data: :class: Joy
                Joy msg to parse
        """
        
        if self.two_stick:
            # Two Stick Control
            self.left = joy_data.axes[1]
            self.right = joy_data.axes[4]

            self.pub_left_vel.publish(self.left)
            self.pub_right_vel.publish(self.right)
        else:
            # One Stick Control
            twist = Twist()

            # One Stick Control
            twist.linear.x = 4*joy_data.axes[1]
            twist.angular.z = 4*joy_data.axes[0]

            # Two Stick Control

            self.pub_move.publish(twist)


    def arm(self, joy_data):
        """
        Parses Joy msgs and publishes arm motor velocities/pos

        Parameters
        ----------
        joy_data: :class: Joy
                Joy msg to parse
        """
        # TODO: Fill in the arm stuff
        pass


if __name__ == "__main__":
    # starts the joy2rover node
    try:
        joy_node = Joy2Rover()
        joy_node.spin()      
    except rospy.ROSInterruptException:
        pass
