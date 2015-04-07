#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>

#include "rosaria/BumperState.h"

using geometry_msgs::Twist;
using namespace std;


class TeleopPioneer
{
public:
	TeleopPioneer();
	void keyLoop();
	
	ros::Time t1;
	
private:
	ros::NodeHandle n;	
	ros::Publisher publicadorVelocidad;
	Twist vel;
	unsigned int temp=0;
	
};

TeleopPioneer::TeleopPioneer()
{
	publicadorVelocidad = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
}

void TeleopPioneer::keyLoop()
{
	char c;
	bool dirty=false;
	
	while(ros::ok)
	{
		cin>>c;

		switch(c)
		{
			case 'a':
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

			case 'd':
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

			case 'w':
				ROS_DEBUG("UP");
				vel.linear.x = 0.3;
				vel.linear.y=0;
				vel.linear.z=0;
				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = 0;
				ROS_INFO("forward");
				dirty = true;
				break;

			case 's':
				ROS_DEBUG("DOWN");
				vel.linear.x = -0.3;
				vel.linear.y=0;
				vel.linear.z=0;
				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = 0;
				ROS_INFO("Backward");
				dirty = true;
				break;
				
			default: //STOP with 'q'
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

		publicadorVelocidad.publish(vel);
		
		ros::Duration(0.1).sleep(); // sleep for one tenth of a second
		ros::spinOnce();
		
	}
	
return;
}



void quit(int sig)
{
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_pioneer");
	TeleopPioneer teleop_pioneer;
	signal(SIGINT, quit);
	ros::Rate r(5);
	//teleop_pioneer.t1=ros::Time::now();	
	
	teleop_pioneer.keyLoop();
	return(0);
}
	
