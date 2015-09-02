#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using geometry_msgs::Twist;
using namespace std;

class RosariaSounds
{
public:
	RosariaSounds(ros::NodeHandle &n);
	ros::Time t1;
	
private:
	ros::NodeHandle n;	
	ros::Publisher publicador;
	ros::Subscriber subscriptor;
	void getVel(const geometry_msgs::Twist::ConstPtr& velocidad);
	Twist vel;

	
};

RosariaSounds::RosariaSounds(ros::NodeHandle &n)
{
	ROS_INFO("Initializing RosariaSounds class");
	publicador= n.advertise<geometry_msgs::Twist>("cmd_vel", 2);
	subscriptor = n.subscribe("cmd_vel", 10, &MoveAlone::getVel, this);
	ROS_INFO("Publisher OK");
}

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "RosariaSounds", ros::init_options::NoSigintHandler);
	signal(SIGINT, quit);

    ros::NodeHandle nh;

    RosariaSounds sounds(nh);
	sounds.play();
	
	ros::Rate r(5);
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
	puts("--------------Reading from keyboard-------------");
	puts("------------------------------------------------");
	puts("Use arrow keys to move Pioneer and SPACE to stop");


	while(!g_request_shutdown && ros::ok())
	{

		// get the next event from the keyboard

		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}
			
		switch(c)
		{
			case KEYCODE_L:
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
				vel.linear.x = -0.4;
				vel.linear.y=0;
				vel.linear.z=0;
				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = 0;
				ROS_INFO("Backward");
				dirty = true;
				break;
				
			case KEYCODE_SPACE:
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
	
	vel.linear.x = 0;
	vel.linear.y=0;
	vel.linear.z=0;
	vel.angular.x = 0;
	vel.angular.y = 0;
	vel.angular.z = 0;
	ROS_INFO("STOP AND EXITING");
	chatter_pub.publish(vel);
	ros::shutdown();
	tcsetattr(kfd, TCSANOW, &cooked);
	exit(0);
	return 0;
}

	
