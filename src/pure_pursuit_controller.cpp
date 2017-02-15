#include "../include/pure_pursuit_controller/pure_pursuit_controller.hpp"
#include "../include/interpolation/bezier.hpp"

// Constructors and Destructors.
// Default Constructor.
PurePursuit::PurePursuit(){}
// Individual Constructor.
PurePursuit::PurePursuit(float k1_lad, float k2_lad, ros::NodeHandle* n, std::string file_location_path_txt)
{
	// 1. Save the arguments to member variables.
	// Set the nodehandle.
	n_ = n;
	// Set the LAD tuning-knob.
	k1_lad_ = k1_lad;
	k2_lad_ = k2_lad;
	// Save the location of the teach path.
	file_location_path_txt_ = file_location_path_txt;

	// 2. Initialize some member variables.
	// Read in the text file where the teach path is saved and store it to a member variable of type nav_msgs/Path.
	readPathFromTxt(file_location_path_txt_);
	// Not needed if L&M sends us the current_arrayposition
	state_.current_arrayposition = 0;
	// Set the Wheelbase.
	wheel_base_ = 3.0;
	// The current array position which won't be overwritten by the incoming state.
	global = 0;

	// 3. ROS specific setups.
	// Publisher.
	// Publishes the control commands to the interface node (TO VCU).
	pub_stellgroessen_ = n_->advertise<ackermann_msgs::AckermannDrive>("/stellgroessen", 10);
	// Publishes the track error. Can be used to test the accuracy of the controller.
	track_error_pub = n_->advertise<std_msgs::Float64>("/track_error", 10);
	// Subscriber.
	sub_state_ = n_->subscribe("/state", 10, &PurePursuit::sts,this);

	// Construction succesful.
	std::cout<<"PP Object created!"<<std::endl;
}
// Default destructor.
PurePursuit::~PurePursuit(){}

// Callback Function/Method which waits for new state from L&M and then calculates the ideal control commands.
void PurePursuit::sts(const arc_msgs::State::ConstPtr& incoming_state)
{
	// Save the incoming state (from subscriber) to a member variable.
	state_ = *incoming_state;
	// Calculate the steering angle using the PurePursuit Controller Formula.
	this -> calculateSteer();
	// Calculate the ideal velocity using the self-derived formula.
	this -> calculateVel();
	// Publish the calculated control commands.
	this -> publishU();
	// Display success.
	std::cout<<"SENT COMMAND"<<std::endl;
}

// Calculate the desired steering angle using the pure pursuit formula.
void PurePursuit::calculateSteer()
{
	// The deviation angle between direction of chassis to direction rearaxle<-->LAD_onPath.
	float theta1;
	// The current speed.
	float v_abs = sqrt(pow(state_.pose_diff.twist.linear.x,2) + pow(state_.pose_diff.twist.linear.x,2));
	// Empirical linear function to determine the look-ahead-distance.
	float lad = k2_lad_ + k1_lad_*v_abs;
	// Get the index of the nearest point.
  int j = nearestPoint();
	state_.current_arrayposition = j;
	float l = 0;
	int i = j;
	while(l<lad)
	{
		l += sqrt(pow(path_.poses[i+1].pose.position.x - path_.poses[i].pose.position.x,2)+pow(path_.poses[i+1].pose.position.y - path_.poses[i].pose.position.y,2));
		i = i+1;
	}
	std::cout<<"Nearest point index= "<<j<<std::endl<<"Reference index (LAD) = "<<i<<" "<<l<<std::endl;
	if(i<n_poses_path_-1)
	{
		float dy = path_.poses[i].pose.position.y - (state_.pose.pose.position.y);
		float dx = path_.poses[i].pose.position.x - (state_.pose.pose.position.x);
		theta1 = atan2(dy,dx);
	}
	else
	{
		float dy = path_.poses[n_poses_path_].pose.position.y - (state_.pose.pose.position.y);
		float dx = path_.poses[n_poses_path_].pose.position.x - (state_.pose.pose.position.x);
		theta1 = atan2(dy,dx);
	}

	// Transformation quaternion to euler angles.
	float ox = state_.pose.pose.orientation.x;
	float oy = state_.pose.pose.orientation.y;
	float oz = state_.pose.pose.orientation.z;
	float ow = state_.pose.pose.orientation.w;
	const Eigen::Vector4d quat(ox, oy, oz, ow);
	geometry_msgs::Vector3 eul;
	eul = arc_tools::transformEulerQuaternionMsg(quat);
	float theta2 = -eul.z;
	float alpha = theta1 - theta2;
	// Pure Pursuit Controller Formula.
	u_.steering_angle = atan2(2*wheel_base_*sin(alpha),l);
}
// Method which calculates the ideal speed, using the self-derived empirical formula.
void PurePursuit::calculateVel()
{
	// Do nothing.
}
// Method which publishes the calculated commands onto the topic to the system engineers interface node.
void PurePursuit::publishU()
{
	pub_stellgroessen_.publish(u_);
}

// HELPER METHODS.
// Method which reads in the text file and saves the path to the path_variable.
void PurePursuit::readPathFromTxt(std::string inFileName)
{
	// Create an ifstream object.
	std::fstream fin;
	fin.open(inFileName.c_str());

	// Check if stream is open.
	if (!fin.is_open())
	{
		std::cout << "Error with opening of  " <<inFileName << std::endl;
	}

	// Truncate two lines of the file to get rid of the last '|'.
	fin.seekg (-2, fin.end);
	int length = fin.tellg();
	fin.seekg (0, fin.beg);

	//Stream erstellen mit chars von fin.
	char *file = new char [length];
	fin.read(file, length);
	std::istringstream stream(file, std::ios::in);
	delete[] file;
	fin.close();

	int i = 0;
	int j;
	geometry_msgs::PoseStamped temp_pose;

	// Save to path_ variable.
	while(!stream.eof() && i<length)
	{
		geometry_msgs::PoseStamped temp_pose;
		path_.poses.push_back(temp_pose);
		stream>>j;
		stream>>path_.poses[j-1].pose.position.x;
		stream>>path_.poses[j-1].pose.position.y;
		stream>>path_.poses[j-1].pose.position.z;
		stream.ignore (300, '|');
		i++;
	}
	n_poses_path_ = i;
	std::cout<<"Created path of length: "<<n_poses_path_<<std::endl;
}
// Method which finds the nearest point and returns its index.
int PurePursuit::nearestPoint()
{
	// Array with 3 elements which will store the coordinates and the index of the nearest point.
	float x_projected[3];
	// Copy the current state to a local variable to avoid deleting or overwriting the current state.
	float x_now = state_.pose.pose.position.x;
	float y_now = state_.pose.pose.position.y;
	// Define a ridiculously large distance as a starting point.
	float d_old = 1000;
	// Variable which will be used for iteration and then used for setting global.
	float j = 0;
	// Variable used for publishing the cross-track error.
	std_msgs::Float64 err;

	// Iterate 200 point ahead of last current array position (global). Either 200 forward or the last path point.
	for(int i = global; i<global+200 && i<n_poses_path_; i++)
	{
		float x_path = path_.poses[i].pose.position.x;
		float y_path = path_.poses[i].pose.position.y;
		float d_new = sqrt(pow((x_now - x_path),2) + pow((y_now - y_path),2));
		// Update the current array position if a nearer point was found.
		if(d_new < d_old)
		{
			d_old = d_new;
			j = i;
			x_projected[0] = path_.poses[i].pose.position.x;
			x_projected[1] = path_.poses[i].pose.position.y;
			x_projected[2] = i;
		}
	}
	// Set a current array position which can't be overwritten from sts.
	global = j;
	// Display the current cross-track error.
	std::cout<<"Cross Track Error in cm = "<<d_old*100<<std::endl;
	// Save the cross-track error to a member variable.
	lateral_error_ = d_old;
	err.data = lateral_error_;
	// Publish the cross-track error.
	track_error_pub.publish(err);
	// Return the index of the nearest point.
	return x_projected[2];
}
// Method which returns the current state.
arc_msgs::State PurePursuit::getState()
{
	return state_;
}
// Method which returns the calculated control commands.
ackermann_msgs::AckermannDrive PurePursuit::getU()
{
	ackermann_msgs::AckermannDrive u1 = u_;
	return u1;
}
