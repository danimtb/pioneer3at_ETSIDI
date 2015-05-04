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


class MoveAlone
{
public:
	MoveAlone();
	bool avanza(float distance, float speed);
	//gira(float grados, float speed);
	void positionCallback(const nav_msgs::Odometry::ConstPtr& odom);
	tf::TransformListener listener;
	ros::Time t1;
	
private:
	ros::NodeHandle n;	
	ros::Publisher publicadorVelocidad;
	ros::Subscriber subscriptorPosicion;
	unsigned int temp;
	bool forward;
	bool backward;
	bool right;
	bool left;
	float x;
	float y;
	float z;
};

void MoveAlone::positionCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	x=odom->pose.pose.position.x;
	y=odom->pose.pose.position.y;
	z=odom->pose.pose.orientation.z;
	//cout << *odom <<endl;
	ROS_INFO("New position:\n\tx = %f\n\ty = %f\n\tz = %f", x, y, z);
}

MoveAlone::MoveAlone()
{
	ROS_INFO("Initializing MoveAlone class");
	temp=0;
	publicadorVelocidad = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
	ROS_INFO("Publisher OK");
	subscriptorPosicion = n.subscribe("RosAria/pose", 100, &MoveAlone::positionCallback, this);
	ROS_INFO("Subscriber OK");
}

bool MoveAlone::avanza(float distance, float speed)
{
	ROS_INFO("Moving forward/backward");
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
	tf::StampedTransform startTraslation;
	listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
	listener.lookupTransform("/odom", "/base_link", ros::Time(0), startTraslation);
	
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
		
	//std_msgs::String msg;
	//std::stringstream ss;
	//ss << vel.linear.x;
	//msg.data = ss.str();
    
    float dx=0;
	float dy=0;
	float distanceMoved=0;
	
	tf::StampedTransform currentTraslation;
		
    while(true)
    {
		listener.lookupTransform("/odom", "/base_link", ros::Time(0), currentTraslation);
		dx=currentTraslation.getOrigin().getX()-startTraslation.getOrigin().getX();
		dy=currentTraslation.getOrigin().getY()-startTraslation.getOrigin().getY();
		
		publicadorVelocidad.publish(vel);
		ROS_INFO("Moving with speed: %f", vel.linear.x);
		
		distanceMoved = sqrt(dx * dx + dy * dy);
		ROS_INFO("Distance moved: %f", distanceMoved);
		 
		
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
		ros::spinOnce();
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
	ROS_INFO("Moving alone started");
	MoveAlone moving_alone;
	signal(SIGINT, quit);
	ros::Rate r(5);	
	while(ros::ok())
	{
		moving_alone.avanza(1.0, 0.2);
		
		while(1){}
	}
	return(0);
}
	
