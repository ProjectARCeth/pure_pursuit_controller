#include "../include/pure_pursuit_controller/pure_pursuit_controller.hpp"

#include <iostream>
#include <string> 

int main(int argc, char **argv)
	{
	// Create the high-level controller node.
	ros::init(argc, argv, "high_level_controller");
	ros::NodeHandle n;
	std::string PATH_NAME = *(argv + 1);
	// Create a PP-controller object.
	PurePursuit PP(&n,PATH_NAME);
	ros::Rate r(10); // 10 hz
	while (ros::ok())
	{
	ros::spinOnce();
	r.sleep();
	}
	// Start the callback functions.
	ros::spin();
	return 0;
	}
