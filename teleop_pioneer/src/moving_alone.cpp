#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "rosaria/BumperState.h"
#include <tf/transform_listener.h>

using geometry_msgs::Twist;
using namespace std;


class MoveAlone
{
public:
	MoveAlone(ros::NodeHandle &n);
	bool avanza(float distance, float speed);
	bool gira(bool clockwise, double degrees, double speed);
	void stop();
	tf::TransformListener listener;
	ros::Time t1;
	
private:
	ros::NodeHandle n;	
	ros::Publisher publicadorVelocidad;
	ros::Subscriber subscriptorVelocidad;
	nav_msgs::Odometry odometry;
	void getPose(const nav_msgs::Odometry::ConstPtr& odom);
	
	unsigned int temp;
	bool forward;
	bool backward;
	bool right;
	bool left;
};

void MoveAlone::getPose(const nav_msgs::Odometry::ConstPtr& odom)
{
	odometry=*odom;		
}

void MoveAlone::stop()
{
	ROS_INFO("Odometry -> x=%f", odometry.twist.twist.linear.x);
	ROS_INFO("Odometry -> z=%f", odometry.twist.twist.angular.z);
	ROS_INFO("Stopping robot...");
	while(odometry.twist.twist.linear.x!=0.0f || odometry.twist.twist.angular.z!=0.0f)
	{
		//ROS_INFO("Odometry -> x=%f", odometry.twist.twist.linear.x);
		//ROS_INFO("Odometry -> z=%f", odometry.twist.twist.angular.z);
		Twist vel_stop;
		vel_stop.linear.y = 0.0f;
		vel_stop.linear.z = 0.0f;
		vel_stop.angular.x = 0.0f;
		vel_stop.angular.y = 0.0f;	
		vel_stop.angular.z = 0.0f;
		publicadorVelocidad.publish(vel_stop);
		ros::spinOnce();
	}
	ROS_INFO("Robot Stopped");
	ROS_INFO("Odometry -> x=%f", odometry.twist.twist.linear.x);
	ROS_INFO("Odometry -> z=%f", odometry.twist.twist.angular.z);
	
}

MoveAlone::MoveAlone(ros::NodeHandle &n)
{
	ROS_INFO("Initializing MoveAlone class");
	temp=0;
	publicadorVelocidad = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 2);
	subscriptorVelocidad = n.subscribe("RosAria/pose", 10, &MoveAlone::getPose, this);
	ROS_INFO("Publisher OK");
}

bool MoveAlone::avanza(float distance, float speed)
{
	speed=fabs(speed);
	ROS_INFO("Moving forward/backward");
	
	if (distance>0)
	{
		forward=true;
		backward=false;
	}
	else
	{
		forward=true;
		backward=false;
		speed=-speed;
	}
		
	Twist vel;

	listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));
	tf::StampedTransform start_transform;
	tf::StampedTransform current_transform;
	listener.lookupTransform("base_link", "odom", ros::Time(0), start_transform);
    
    vel.linear.y = 0.0f;
	vel.linear.z = 0.0f;
	vel.angular.x = 0.0f;
	vel.angular.y = 0.0f;
	vel.angular.z = 0.0f;
	vel.linear.x = speed;
		
	//std_msgs::String msg;
	//std::stringstream ss;
	//ss << vel.linear.x;
	//msg.data = ss.str();
		
	ros::Rate rate(10.0);
    bool done = false;
    //send the drive command
	publicadorVelocidad.publish(vel);
	//rate.sleep();
    
    while(!done && n.ok())
    {
		//get the current transform
		try
		{
			listener.lookupTransform("base_link", "odom", ros::Time(0), current_transform);
		}
		catch (tf::TransformException ex)
        {
			ROS_ERROR("%s",ex.what());
			break;
		}
		
		//see how far we've traveled
		tf::Transform relative_transform = start_transform.inverse() * current_transform;
		double dist_moved = relative_transform.getOrigin().length();
		
		if(fabs(dist_moved) > pow(fabs(distance), 0.50f))
        {
			ROS_INFO("Decreasing speed");
			vel.linear.x=speed*0.1f;
			vel.linear.y = 0.0f;
			vel.linear.z = 0.0f;
			vel.angular.x = 0.0f;
			vel.angular.y = 0.0f;
			vel.angular.z = 0.0f;
			publicadorVelocidad.publish(vel);
			
		}
		
		if(fabs(dist_moved) > fabs(distance))
        {
            ROS_INFO("Distance moved: %f", dist_moved);
			done = true;
		}
		ros::spinOnce();
	}
	stop();
	
	if (done) return true;
    return false;
}

bool MoveAlone::gira(bool clockwise, double degrees, double speed)
{
	double radians=(degrees/360.0f)*2*M_PI;
	ROS_INFO("Rotating %f radians", radians);
	
	while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;

    //wait for the listener to get the first message
    listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener.lookupTransform("base_link", "odom", ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = speed;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    if (!clockwise) desired_turn_axis = -desired_turn_axis;
    
    bool done = false;
    //send the drive command
    publicadorVelocidad.publish(base_cmd);
    
    while (!done && n.ok())
    {
        //get the current transform
        try
        {
            listener.lookupTransform("base_link", "odom", ros::Time(0), current_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            break;
        }
        tf::Transform relative_transform = start_transform.inverse() * current_transform;
        tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
      
        double angle_turned = relative_transform.getRotation().getAngle();
      
        if(fabs(angle_turned) < 1.0e-2) continue;

        if(actual_turn_axis.dot( desired_turn_axis ) < 0)
		    angle_turned = 2 * M_PI - angle_turned;

        if(angle_turned > radians) {
            ROS_INFO("Angle turned: %f", angle_turned);
            done=true;
        }
		
		ros::spinOnce();
    }
    stop();
    if (done) return true;
    return false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "moving_alone");
	ROS_INFO("Moving alone started");
	ros::NodeHandle nh;
	MoveAlone moving_alone(nh);
	moving_alone.avanza(1.0f, 0.5f);
	moving_alone.gira(true, 90.0f, 0.2f);
	moving_alone.avanza(-0.5f, 0.1f);
	return 0;
}
	
