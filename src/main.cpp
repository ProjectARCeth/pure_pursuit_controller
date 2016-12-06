#include"pure_pursuit_header.hpp"
#include"ackermann_msgs/AckermannDrive.h"
#include"iostream"

int main(int argc, char **argv)
	{
	ros::init(argc, argv, "regler_node");
	ros::NodeHandle n;
	float k1=1;	
	PurePursuit PP(k1,&n);

	std::cout<<PP.getU().speed;	
	ros::spin();
	return 0;
	}

//spin spinonce ? wie wäre ein bsp?
