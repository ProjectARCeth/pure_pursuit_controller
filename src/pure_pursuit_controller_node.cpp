#include "../include/pure_pursuit_controller/pure_pursuit_controller.hpp"
#include "ackermann_msgs/AckermannDrive.h"
#include <iostream>

int main(int argc, char **argv)
	{
	// Define the location where the teach path is stored.
	std::string file_location_path_txt = "trolololol";
	// Create the high-level controller node.
	ros::init(argc, argv, "high_level_controller");
	ros::NodeHandle n;

	// k_lad is the tuning knob for the look-ahead-distance of the pure_pursuit_controller.
	// The bigger k_lad, the bigger the lateral error but the smoother ride.
	float k_lad = 0.5;
	PurePursuit PP(k_lad, &n, file_location_path_txt);

	// Start the callback functions.
	ros::spin();
	return 0;
	}
