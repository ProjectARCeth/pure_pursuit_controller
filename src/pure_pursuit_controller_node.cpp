#include "../include/pure_pursuit_controller/pure_pursuit_controller.hpp"

int main(int argc, char **argv)
	{
	// Create the high-level controller node.
	ros::init(argc, argv, "high_level_controller");
	ros::NodeHandle n;

	// Create a PP-controller object.
	PurePursuit PP(&n);

	// Start the callback functions.
	ros::spin();
	return 0;
	}
