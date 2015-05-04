#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>
#include <math.h>

#include "rosaria/BumperState.h"
#include <tf/transform_listener.h>

using geometry_msgs::Twist;
using namespace std;

void positionCallback(const nav_msgs::Odometry& odom)
{
  ROS_INFO("Looking for odometry");
  x=odom.pose.pose.position.x;
  y=odom.pose.pose.position.y;
  z=odom.pose.pose.orientation.z;
}

bool avanza(float distance, float speed)
{
	bool done=false;
	
	if (distance>0)
	{
		forward=true;
		backward=false;
	}
	else
	{
		forward=true;
		backward=false;
	}
	
	if (speed<0)
		speed=-speed;
		
	Twist vel;
	
	float x0 = x;
    float y0 = y;
    
    vel.linear.y = 0;
	vel.linear.z = 0;
	vel.angular.x = 0;
	vel.angular.y = 0;
	vel.angular.z = 0;
	
    if(forward)
		vel.linear.x = speed;
	else
		vel.linear.x = -speed;
		
	publicadorVelocidad.publish(vel);
    
    while(true)
    {
		float dx=x-x0;
		float dy=y-y0;
		float distanceMoved = sqrt(dx * dx + dy * dy);
		
		if(forward)
		{
			if (distanceMoved >= distance)
			{
				done=true;
				break;
			}
		}
		else
		{
			if (distanceMoved >= -distance)
			{
				done=true;
				break;
			}
		}
		
		ros::Duration(0.1).sleep(); // sleep for one tenth of a second
	}
	forward=false;
	backward=false;
	vel.linear.x = 0.0;
	publicadorVelocidad.publish(vel);	
	
return done;
}



void quit(int sig)
{
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "moving_alone");
	ros::Time t1;
	ros::NodeHandle n;	
	ros::Publisher publicadorVelocidad = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
	ros::Subscriber subscriptorPosicion = n.subscribe("RosAria/pose", 100, positionCallback);
	unsigned int temp=0;
	bool forward;
	bool backward;
	bool right;
	bool left;
	static float x;
	static float y;
	static float z;
	signal(SIGINT, quit);
	ros::Rate r(5);	
	
	avanza(1.0, 0.2);
	return(0);
}
	
