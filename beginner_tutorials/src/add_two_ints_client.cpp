#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
	int a, b;
  ros::init(argc, argv, "add_two_ints_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
 
 while(ros::ok())
  {
	  ROS_INFO("Introduce dos numeros:");
	  cin>>a;
	  cin>>b;
	  srv.request.a = a;
	  srv.request.b = b;
	  if (client.call(srv))
	  {
		ROS_INFO("Sum: %ld", srv.response.sum);
	  }
	  else
	  {
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	  }
}

  return 0;
}
