#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include "rosaria/BumperState.h"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71 ////////DEFINIR BARRA ESPACIADORA

using geometry_msgs::Twist;
using namespace std;

class TeleopPioneer
{
public:
	TeleopPioneer();
	void keyLoop();
	
	ros::NodeHandle n; 
		
	ros::Publisher chatter_pub;
	ros::Time t1;
	Twist vel;
	//int kfd = 0;
	//struct termios cooked, raw;
	unsigned int temp=0;
	
};

TeleopPioneer::TeleopPioneer()
{
	//nh_.param("scale_angular", a_scale_, a_scale_);
	//nh_.param("scale_linear", l_scale_, l_scale_);
	//twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
	chatter_pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_turtle");
	TeleopPioneer teleop_pioneer;
	signal(SIGINT,quit);
	
	ros::init(argc, argv, "main"); 
	signal(SIGINT,quit);
	ros::Rate r(5);
	teleop_pioneer.t1=ros::Time::now();	
	
	teleop_pioneer.keyLoop();
	return(0);
}

void TeleopPioneer::keyLoop()
{
	char c;
	bool dirty=false;
	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move Pioneer");


	while(ros::ok)
	{
		// get the next event from the keyboard

		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		ROS_DEBUG("value: 0x%02X\n", c);

		switch(c)
		{
			case KEYCODE_L:
				ROS_DEBUG("LEFT");
				vel.linear.x = 0;
				vel.linear.y=0;
				vel.linear.z=0;
				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = 0.5;
				ROS_INFO("Turnleft");
				dirty = true;
				break;

			case KEYCODE_R:
				ROS_DEBUG("RIGHT");
				vel.linear.x = 0;
				vel.linear.y=0;
				vel.linear.z=0;
				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = -0.5;
				ROS_INFO("Turnright");
				dirty = true;
				break;

			case KEYCODE_U:
				ROS_DEBUG("UP");
				vel.linear.x = 0.4;
				vel.linear.y=0;
				vel.linear.z=0;
				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = 0;
				ROS_INFO("forward");
				dirty = true;
				break;

			case KEYCODE_D:
				ROS_DEBUG("DOWN");
				vel.linear.x = -0.4;
				vel.linear.y=0;
				vel.linear.z=0;
				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = 0;
				ROS_INFO("Backward");
				dirty = true;
				break;
				
			case KEYCODE_Q: //REEEEEVIIIIIISAAAAAAAARRRRRR
				ROS_DEBUG("Q");
				vel.linear.x = 0;
				vel.linear.y=0;
				vel.linear.z=0;
				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = 0;
				ROS_INFO("stop");
				dirty= true;
				break;
		}


		if(dirty ==true)
		{
			chatter_pub.publish(vel);
			dirty=false;
		}
		
		ros::Duration(0.1).sleep(); // sleep for one tenth of a second
		ros::spinOnce();
		
	}
	
return;
}
	
