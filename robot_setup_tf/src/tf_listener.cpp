#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

	//we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
	sensor_msgs::LaserScan laser_scan;
	laser_scan.header.frame_id = "base_sick";

void transformLaser(const tf::TransformListener& listener)
{	
	//we'll just use the most recent transform available for our simple example
	laser_scan.header.stamp = ros::Time();
	
	try
	{
		sensor_msgs::LaserScan base_scan;
		listener.transformPoint("base_pioneer", laser_scan, base_scan);
		
		ROS_INFO("Transformada recibida");
	}
    
    catch(tf::TransformException& ex)
    {
		ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
	}
}

void callback(const sensor_msgs::LaserScan& datos)
{
	laser_scan=datos;		
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_tf_listener");
	ros::NodeHandle n;
	
	ros::Subscriber subscriptorLaser;
	subscriptorLaser = n.subscribe<sensor_msgs::LaserScan>("/scan", 100, callback);
	
	tf::TransformListener listener(ros::Duration(10));
	
	//we'll transform a point once every second
	ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformLaser, boost::ref(listener)));
	
	ros::spin();

}
