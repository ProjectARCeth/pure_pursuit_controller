#include"pure_pursuit_header.hpp"
#include"ackermann_msgs/AckermannDrive.h"
#include <iostream>

int main(int argc, char **argv)
	{
	ros::init(argc, argv, "regler_node");
	ros::NodeHandle n;
	float k1=0.5;
	PurePursuit PP(k1,&n);
	//PP.setState(0,3);
		//float j1=PP.findReference(19);
		//PP.setState(0,5);
		//float j2=PP.findReference(19);
	//float b=fabs(-2324.3);
//	float a=atan2(5,-2);

	//std::cout<<a<<std::endl;
	ros::spin();
	return 0;
	}

//spin spinonce ? wie wäre ein bsp?
