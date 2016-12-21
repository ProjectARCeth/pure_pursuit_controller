#include"../include/pure_pursuit_controller/pure_pursuit_controller.hpp"
#include"ackermann_msgs/AckermannDrive.h"
#include <iostream>

int main(int argc, char **argv)
	{
	ros::init(argc, argv, "regler_node");
	ros::NodeHandle n;
	//Je grösser k1 desto grösser der laterale Fehler aber sanfter die Fahrt
	float k1=0.5;	
	PurePursuit PP(k1,&n);

	ros::spin();		
	return 0;
	}




