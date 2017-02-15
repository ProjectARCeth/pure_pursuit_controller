#pragma once

#include "arc_msgs/State.h"
#include "arc_tools/coordinate_transform.hpp"
#include "ackermann_msgs/AckermannDrive.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "math.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <fstream>
#include <sstream>

class PurePursuit{

public :
	// 1. Constructors and Destructors.
	// Default constructor.
	PurePursuit();
	// Individual constructor which takes the node handle and the LAD tuning-knob as arguments.
	PurePursuit(float k1_lad, float k2_lad, ros::NodeHandle* n, std::string file_location_path_txt);
	// Default destructor.
	~PurePursuit();

	// 2. Essential methods which are needed for the controller to work.
	// Callback functions which saves the incoming state and calls the essential methods to calculate the needed control commands.
	void sts(const arc_msgs::State::ConstPtr& incoming_state);
	// Method which calculates the required steering angle, using the pure-pursuit formula.
	void calculateSteer();
	// Method which calculates the ideal speed, using the self-derived empirical formula.
	void calculateVel();
	// Method which publishes the calculated control commands on a specified topic (TO Interface-Node).
	void publishU();

	// 3. Helper methods for the controller.
	// Method which will be called in the constructor. Reads in the text-file which contains the teach-path and saves it.
	void readPathFromTxt(std::string inFileName);
	// Method which finds the nearest point.
	float* nearestPoint();
	// Method which returns the current state.
	arc_msgs::State getState();
	// Method which returns the calculated control commands.
	ackermann_msgs::AckermannDrive getU();

private:
	// 1. ROS setup.
	ros::NodeHandle* n_;
	// Publishers.
	ros::Publisher pub_stellgroessen_;
	ros::Publisher path_publisher;
	ros::Publisher track_error_pub;
	// Subscribers.
	ros::Subscriber sub_state_;

	// 2. Parameters (are initialized at the beginning and (usually) not changed during the process).
	// Location where text-file with teach path is stored.
	std::string file_location_path_txt_;
	// Saves the path which will be read out from the text file.
	nav_msgs::Path path_;
	// Number of path points.
	int n_poses_path_;
	// Wheelbase.
	float wheel_base_;
	// Tuning knob for PurePursuit controller
	float k1_lad_;
	float k2_lad_;

	// 3. Variables (will be changed during process.)
	// Variable which saves the incoming state from L&M.
	arc_msgs::State state_;
	// Variable which saves the calculated commands. Can be used for publishing or logging.
	ackermann_msgs::AckermannDrive u_;
	// Variable which can be used for calculating the current lateral error. Can be used to display the controllers performance.
	float lateral_error_;
};
