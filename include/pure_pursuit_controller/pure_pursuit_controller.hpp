#pragma once

#include "arc_msgs/State.h"
#include "arc_tools/coordinate_transform.hpp"
#include "arc_tools/timing.hpp"
#include "ackermann_msgs/AckermannDrive.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "math.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <algorithm>

class PurePursuit{

public:
	// 1. Constructors and Destructors.
	// Default constructor.
	PurePursuit();
	// Individual constructor which takes the node handle and the LAD tuning-knob as arguments.
	PurePursuit(ros::NodeHandle* n);
	// Default destructor.
	~PurePursuit();

	// 2. Essential methods which are needed for the controller to work.
	// Callback functions which saves the incoming state and calls the essential methods to calculate the needed control commands.
	void stateCallback(const arc_msgs::State::ConstPtr& incoming_state);
	//Callback function for distance to obstacle
	void obstacleCallback(const std_msgs::Float64::ConstPtr& msg);
	//Save gui_stop bool;
	void guiStopCallback(const std_msgs::Bool::ConstPtr& msg);
	// Method which calculates the required steering angle, using the pure-pursuit formula.
	void calculateSteer();
	// Method which calculates the ideal speed, using the self-derived empirical formula.
	void calculateVel();
	// Method which publishes the calculated control commands on a specified topic (TO Interface-Node).
	void publishU();

	// 3. Helper methods for the controller.
	// Method which will be called in the constructor. Reads in the text-file which contains the teach-path and saves it.
	void readPathFromTxt(std::string inFileName);
	// Method which finds the nearest point and returns its index.
	int nearestPoint();
	// Method which returns the current state.
	arc_msgs::State getState();
	// Method which returns the calculated control commands.
	ackermann_msgs::AckermannDrive getU();
	//Reurns distance between index from_i to index to_i
	float distanceIJ(int from_i , int to_i );
	//Returns mean curvature at index i
	float curveRadius(int j);
	//Returns index at certain distance from input index
	int indexOfDistanceFront(int i, float d);
	int indexOfDistanceBack(int i, float d);

private:
	// 1. ROS setup.
	ros::NodeHandle* n_;
	// Publishers.
	ros::Publisher pub_stellgroessen_;
	ros::Publisher path_publisher_;
	ros::Publisher track_error_pub_;
	ros::Publisher gui_pub_;
	// Subscribers.
	ros::Subscriber sub_state_;
	ros::Subscriber distance_to_obstacle_sub_;
	ros::Subscriber gui_stop_sub_;
	// 2. Parameters (are initialized at the beginning and (usually) not changed during the process).
	// Saves the path which will be read out from the text file on path_diff the velocities.
	nav_msgs::Path path_;
	nav_msgs::Path path_diff_;
	std::vector<float> teach_vel_;
	// Number of path points.
	int n_poses_path_;
	int slow_down_index_;
	// 3. Variables (will be changed during process.)
	// Variable which saves the incoming state from L&M.
	arc_msgs::State state_;
	float obstacle_distance_;
	// Variable which saves the calculated commands. Can be used for publishing or logging.
	ackermann_msgs::AckermannDrive u_;
	// Variable which can be used for calculating the current lateral error. Can be used to display the controllers performance.
	float tracking_error_;
	// The current array position which won't be overwritten by the incoming state.
	int global_;
	//easy access to absolute velocity
	float v_abs_;
	//Shut down from graphical user interface
	bool gui_stop_;
	//Time
	arc_tools::Clock BigBen_;
	//
	std_msgs::Float32MultiArray pure_pursuit_gui_msg_;
};


