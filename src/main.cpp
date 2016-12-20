#include"pure_pursuit_header.hpp"
#include"ackermann_msgs/AckermannDrive.h"
#include <iostream>

int main(int argc, char **argv)
	{
	ros::init(argc, argv, "regler_node");
	ros::NodeHandle n;
	float k1=0.5;	//je grösser k1 desto grösser der laterale fehler aber sanfter die fahrt
	PurePursuit PP(k1,&n);

	ros::spin();		
	return 0;
	}




