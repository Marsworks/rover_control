/*
This node allows to take manual control of the rover at any time.

When the robot is not controlled manually, the node simply receives commands over the cmd and toggleArm topic and passes them over to the cameleon/cmd_vel 

When the operator presses manualOverrideButton, the cmd and toggleArm messages are discarted and the operator can drive the robot manually. 

TODO:
- Add skid steering support (drive individual sides)
- Add camera toggle support
- Add Robot arm support (moveit_servo)

Author: Ahmed Abbas with adapted code from gestom's cameleon_teleop (https://github.com/gestom/cameleon_teleop)
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <string.h>


class RoverJoyTeleop {

    protected:
    ros::NodeHandle _n;

    private: 
    /*joystick input parameters - which button causes the user to take control, toggle to arm control mode and axes that correspond to forward*/ 
    int manualOverrideButton = 4;
    int linearAxis = 1;
    int angularAxis = 0;
    int armButton = 6;

    /*these constants determine how quickly the robot moves based on the joystick input*/ 
    double linearGain = 0.2;
    double angularGain = 0.2;

    /*Current drive mode*/
    std::string driveMode;
    bool armMode;

    /*listening to joystick, toggleArm and cmd topics and publishing commands to the cameleon ros driver*/
    ros::Publisher vel_pub_;
    ros::Subscriber arm_sub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber cmd_sub_;

    /*state variables - twist is the message that eventually gets to the ROS driver of the robot, other are obvious*/
    geometry_msgs::Twist twist;
    double forwardSpeed = 0;
    double forwardAcceleration= 0;


    public:
    bool teleoperated = false;

    RoverJoyTeleop(ros::NodeHandle nh) : _n(ros::NodeHandle()) {
        nh.param("axis_linear", linearAxis, 1);
        nh.param("axis_angular", angularAxis, 0);
        nh.param("manual_override_button", manualOverrideButton, 4);

        nh.param("scale_angular", angularGain, 0.2);
        nh.param("scale_linear", linearGain, 0.2);
        

        nh.param<std::string>("drive_mode", driveMode, "diff");
        nh.param<bool>("start_arm", armMode, 0);

        vel_pub_ = _n.advertise<geometry_msgs::Twist>("joy_teleop/cmd_vel", 1);
        joy_sub_ = _n.subscribe<sensor_msgs::Joy>("joy", 10, &RoverJoyTeleop::joyCallback, this);
        cmd_sub_ = _n.subscribe<geometry_msgs::Twist>("cmd", 10, &RoverJoyTeleop::cmdCallback, this);
        arm_sub_ = _n.subscribe("/toggleArm", 1, &RoverJoyTeleop::armCallback, this);
        
        printParams();

     }


    /*commands from higher-lever modules*/
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd)
    {
        /*if the robot is not teleoperated, form the twist command from the incoming cmd topic*/
        if (teleoperated == false){
            twist.linear.x  = cmd->linear.x;
            twist.angular.z = cmd->angular.z;    
        }
        
    }

    /*arm toggle can come from another component*/
    void armCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        //TODO: add something here   
    }

    /*receiving joystick data*/
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    {     
            /*if swiching modes from teleoperated to manual and vice versa, then clear velocities*/
        if (teleoperated != joy->buttons[manualOverrideButton]){
            twist.angular.z = 0.0;
            twist.linear.x = 0.0;
            twist.angular.y = 0.0;
            forwardAcceleration = 0;
            forwardSpeed = 0;
        }

        /*is it teleoperated or arm controlled?*/
        teleoperated = (joy->buttons[manualOverrideButton] == 1);
        armMode = joy->buttons[armButton] == 1;

        /*if yes, form the twist command from the joystick input*/
        if (teleoperated && !armMode)
        {
            twist.angular.z = angularGain*joy->axes[angularAxis];
            forwardAcceleration = 0.02*joy->axes[linearAxis];;
            ROS_INFO( "%i %i ",angularAxis,linearAxis);
            ROS_INFO( "Rychlost z= %f, Rychlost x= %f ",  twist.linear.z,twist.linear.x);
        }
    }
    
    void printParams(){
        std::cout 
        << "----------------------\n"
        << "Teleop Parameters:"
        << "\nLinear Gain: " << linearGain
        << "\nAngular Gain: " << angularGain
        << "\nDrive Mode: " << driveMode
        << "\nArm Mode: " << armMode
        << "----------------------" << std::endl;
    }

    /**
     * Spin and increase
     */ 
    void spinJoy(){
        ros::spinOnce();
		if (teleoperated){
			forwardSpeed += forwardAcceleration;
			forwardSpeed = fmin(fmax(forwardSpeed,-1.0),1.0);
			twist.linear.x =  forwardSpeed*linearGain;
            vel_pub_.publish(twist);
		}
    }

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "cameleon_teleop");
	ros::NodeHandle nh("~");

    RoverJoyTeleop rjt(nh);

	while (ros::ok()){
		rjt.spinJoy();
		usleep(50000);
	}

}
    