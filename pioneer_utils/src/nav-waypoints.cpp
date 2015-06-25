 
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool setLocalGoal(const float &x, const float &angle)
{
	 //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "/base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.orientation.w = angle;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
	  ROS_INFO("Hooray, the base moved 1 meter forward");
	  return true;
  }
  else
  {
	  ROS_INFO("The base failed to move forward 1 meter for some reason");
	  return false;
  }	
}


bool setGlobalGoal(const float &x, const float &y, const float &angle)
{
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();
	
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
	ROS_INFO("Sending GLOBAL goal");
	ac.sendGoal(goal);
	
	ac.waitForResult();
	
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Hooray, the base moved 1 meter forward");
		return true;
	}
    else
    {
		ROS_INFO("The base failed to move forward 1 meter for some reason");
		return false;
	}
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  setGlobalGoal(-0.671, 1.938, 1.0); //rosa
  setGlobalGoal(0.193, -1.520, 1.0); //invernadero
  setGlobalGoal(2.118, -8.223, 1.0); //comau
  setGlobalGoal(-1.073, -9.271, 1.0); //puerta principal
  
  return 0;
}
