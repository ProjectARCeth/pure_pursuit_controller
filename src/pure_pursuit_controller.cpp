#include "../include/pure_pursuit_controller/pure_pursuit_controller.hpp"
//Parameters
float K1_LAD_S=0.5;
float K2_LAD_S=3;
float K1_LAD_V=0.5;
float K2_LAD_V=3;
float MU_HAFT=0.8; // = 0.8 ungefähr
float G_EARTH=9.81;
float MAX_LATERAL_ACCELERATION_EROD=MU_HAFT*G_EARTH;
float MAX_ABSOLUTE_VELOCITY=30;
float WHEEL_BASE=3;
float C_VEL=1;  //[0,1]
float SLOW_DOWN_DISTANCE=10; 
float V_FREEDOM=20;
float SHUT_DOWN_TIME=5;
std::string FILE_LOCATION_PATH_TXT="/home/moritz/.ros/Paths/current_path_HG.txt";
float N_INTERPOLATION=100;
float D_INTERPOLATION=3;

// Constructors and Destructors.
// Default Constructor.
PurePursuit::PurePursuit(){}
// Individual Constructor.
PurePursuit::PurePursuit(ros::NodeHandle* n)
{	

	std::cout<<"Constructor"<<std::endl;
	// 1. Save the arguments to member variables.
	// Set the nodehandle.
	n_ = n;
	// 2. Initialize some member variables.
	// Read in the text file where the teach path is saved and store it to a member variable of type nav_msgs/Path.
	readPathFromTxt(FILE_LOCATION_PATH_TXT);
	// Not needed if L&M sends us the current_arrayposition
	state_.current_arrayposition = 0;
	// The current array position which won't be overwritten by the incoming state.
	global_ = 0;
	// Initialize gui_stop;
	gui_stop_=0;
	//Initialize obstacle distance in case it is not the first callback function running
	obstacle_distance_=100;
	pure_pursuit_gui_msg_.data.clear();
	for(int i=0;i<10;i++) pure_pursuit_gui_msg_.data.push_back(0);
	std::cout<<"Constructor"<<std::endl;
	// 3. ROS specific setups.
	// Publisher.
	// Publishes the control commands to the interface node (TO VCU).
	pub_stellgroessen_ = n_->advertise<ackermann_msgs::AckermannDrive>("/stellgroessen", 10);
	// Publishes the track error. Can be used to test the accuracy of the controller.
	track_error_pub_ = n_->advertise<std_msgs::Float64>("/track_error", 10);
	gui_pub_ = n_->advertise<std_msgs::Float32MultiArray>("/pure_pursuit_info", 10);
	// Subscriber.
	sub_state_ = n_->subscribe("/state", 10, &PurePursuit::stateCallback,this);
	distance_to_obstacle_sub_=n_->subscribe("/distance_to_obstacle",10,&PurePursuit::obstacleCallback,this);
	gui_stop_sub_=n_->subscribe("/shut_down",10,&PurePursuit::guiStopCallback,this);
	// Construction succesful.
	std::cout << std::endl << "PURE PURSUIT: Initialized with path lenght: " << std::endl;
	std::cout<<"Created path of length: "<<n_poses_path_<<std::endl;
	pure_pursuit_gui_msg_.data.push_back(1.2);
	std::cout<<"Slow down index= "<<slow_down_index_<<std::endl;

float R1=curveRadius(820);
std::cout<<"Radius: "<<R1<<std::endl;
/*
for(int i=0;i<n_poses_path_;i+=100)
{
R1=curveRadius(i);
std::cout<<"Radius: "<<R1<<std::endl<<std::endl;
}*/
}	
// Default destructor.
PurePursuit::~PurePursuit(){}

// Callback Function/Method which waits for new state from L&M and then calculates the ideal control commands.
void PurePursuit::stateCallback(const arc_msgs::State::ConstPtr& incoming_state)
{	
	std::cout<<"State CallBack"<<std::endl;
	// Save the incoming state (from subscriber) to a member variable.
	state_ = *incoming_state;
	//Save absolute velocity for simplicity
	v_abs_=sqrt(pow(state_.pose_diff.twist.linear.x,2) +
			pow(state_.pose_diff.twist.linear.y,2)
			+pow(state_.pose_diff.twist.linear.z,2));
	float x_path = path_.poses[state_.current_arrayposition].pose.position.x;
	float y_path = path_.poses[state_.current_arrayposition].pose.position.y;
	float z_path = path_.poses[state_.current_arrayposition].pose.position.z;
	float x_now = state_.pose.pose.position.x;
	float y_now = state_.pose.pose.position.y;
	float z_now = state_.pose.pose.position.z;
	tracking_error_ = sqrt(pow((x_now - x_path),2) + pow((y_now - y_path),2) + pow((z_now - z_path),2));
	std::cout<<"ciao"<<std::endl;
	pure_pursuit_gui_msg_.data[0]=distanceIJ(0,state_.current_arrayposition);
	std::cout<<"cia"<<std::endl;
	pure_pursuit_gui_msg_.data[1]=distanceIJ(state_.current_arrayposition,n_poses_path_-1);
	// Calculate the steering angle using the PurePursuit Controller Formula.
	this -> calculateSteer();
	// Calculate the ideal velocity using the self-derived formula.
	this -> calculateVel();
	// Publish the calculated control commands.
	this -> publishU();
	// Display success.
	std::cout<<"0:Abstand zu Start Pfad "<<pure_pursuit_gui_msg_.data[0]<<std::endl;
	std::cout<<"1:Abstand zu Ende Pfad "<<pure_pursuit_gui_msg_.data[1]<<std::endl;
	std::cout<<"2:Referenzindex für steuerung "<<pure_pursuit_gui_msg_.data[2]<<std::endl;
	std::cout<<"3:Solllenkwinkel "<<pure_pursuit_gui_msg_.data[3]<<std::endl;
	std::cout<<"4:Referenzindex für radius geschwindigkeit "<<pure_pursuit_gui_msg_.data[4]<<std::endl;
	std::cout<<"5:Radius Pfad "<<pure_pursuit_gui_msg_.data[5]<<std::endl;
	std::cout<<"6:Physikalische geschw. Grenze "<<pure_pursuit_gui_msg_.data[6]<<std::endl;
	std::cout<<"7:Bremsweg "<<pure_pursuit_gui_msg_.data[7]<<std::endl;
	std::cout<<"8:Upper bound given by teach velocity "<<pure_pursuit_gui_msg_.data[8]<<std::endl;
	std::cout<<"9:Sollgeschwindigkeit final "<<pure_pursuit_gui_msg_.data[9]<<std::endl;
	std::cout<<"10: "<<pure_pursuit_gui_msg_.data[10]<<std::endl;
	std::cout<<std::endl;
	std::cout<<"SENT COMMAND"<<std::endl;
}

void PurePursuit::guiStopCallback(const std_msgs::Bool::ConstPtr& msg)
{
	gui_stop_=msg->data;
	BigBen_.start();
}

void PurePursuit::obstacleCallback(const std_msgs::Float64::ConstPtr& msg)
{
	obstacle_distance_=msg->data;
}
// Calculate the desired steering angle using the pure pursuit formula.
void PurePursuit::calculateSteer()
{
	// The deviation angle between direction of chassis to direction rearaxle<-->LAD_onPath.
	float theta1;
	// The current speed.
	//float v_abs = sqrt(pow(state_.pose_diff.twist.linear.x,2) + pow(state_.pose_diff.twist.linear.y,2));
	// Empirical linear function to determine the look-ahead-distance.
	float lad = K2_LAD_S + K1_LAD_S*v_abs_;
	// Get the index of the nearest point.
/*
	state_.current_arrayposition = nearestPoint();
*/
	int i=indexOfDistanceFront(state_.current_arrayposition,lad);
	std::cout<<"Current Arrayposition= "<<state_.current_arrayposition<<std::endl<<"Reference index for steering = "<<i<<std::endl;
	float alpha=0;
	if(i<n_poses_path_-1)
	{	
//		float dy = path_.poses[i].pose.position.y - (state_.pose.pose.position.y);
//		float dx = path_.poses[i].pose.position.x - (state_.pose.pose.position.x);
//		theta1 = atan2(dy,dx);
	}
	else
	{
		std::cout<<"if else ist also NICHT unnötig!";

		i=n_poses_path_-1;
//		float dy = path_.poses[n_poses_path_-1].pose.position.y - (state_.pose.pose.position.y);
//		float dx = path_.poses[n_poses_path_-1].pose.position.x - (state_.pose.pose.position.x);
//		theta1 = atan2(dy,dx);

	}
	float l=distanceIJ(state_.current_arrayposition,i);
	pure_pursuit_gui_msg_.data[2]=i;
	geometry_msgs::Point referenz_local=arc_tools::globalToLocal(path_.poses[i].pose.position, state_);
	std::cout<<referenz_local<<std::endl;
	float dy = referenz_local.y;
	float dx = referenz_local.x;
	alpha = atan2(dy,dx);
	std::cout<<"alpha= "<<alpha<<std::endl;
	
	// Transformation quaternion to euler angles.
	float ox = state_.pose.pose.orientation.x;
	float oy = state_.pose.pose.orientation.y;
	float oz = state_.pose.pose.orientation.z;
	float ow = state_.pose.pose.orientation.w;
	const Eigen::Vector4d quat(ox, oy, oz, ow);
	geometry_msgs::Vector3 eul;
	eul = arc_tools::transformEulerQuaternionMsg(quat);

//	float theta2 = -eul.z;
//	float alpha = theta1 - theta2;
	// Pure Pursuit Controller Formula.
	u_.steering_angle = atan2(2*WHEEL_BASE*sin(alpha),l);
	std::cout<<u_.steering_angle<<std::endl;
	pure_pursuit_gui_msg_.data[3]=u_.steering_angle;
}
// Method which calculates the ideal speed, using the self-derived empirical formula.
void PurePursuit::calculateVel()	//""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
{	std::cout<<"Start Calc Vel"<<std::endl;
    //First calculate optimal velocity
	//for the moment take curvature at fix distance lad_v
	float lad_v= K2_LAD_V + K1_LAD_V*v_abs_;
	//find reference index for curvature
	int i=indexOfDistanceFront(state_.current_arrayposition, lad_v);
	if(i=n_poses_path_) i=n_poses_path_-1;

	pure_pursuit_gui_msg_.data[4]=i;
	float v_limit=sqrt(MAX_LATERAL_ACCELERATION_EROD*curveRadius(i));		//Physik stimmt?
	pure_pursuit_gui_msg_.data[6]=v_limit;
    //Penalisations
	float C=C_VEL;
	//penalize lateral error from paht, half for 1m error
	C=C/(1+abs(tracking_error_));
	std::cout<<"Penalisation after lateral error "<<C<<std::endl;
	//Obstacle distance
	float brake_dist=pow(v_abs_*3.6/10,2)/2;	//Physikalisch Sinn??
	pure_pursuit_gui_msg_.data[7]=brake_dist;
	if((obstacle_distance_>brake_dist)&&(obstacle_distance_<2*brake_dist))
	{	
		std::cout<<"Case1"<<std::endl;
		C=C*(obstacle_distance_/brake_dist)-1;
	}
	else if (obstacle_distance_>=2*brake_dist)
	{
		std::cout<<"Case2"<<std::endl;
		C=C;
	}
	else if (obstacle_distance_<=brake_dist)
	{
		std::cout<<"Case3"<<std::endl;
		C=0;
	}
	//Orientation error not yet implemented
	std::cout<<"Penalisation after obstacle distance "<<C<<std::endl;
    //Slow down
	//slow down gradually when arrive at SLOW_DOWN_DISTANCE from end of of path

	if (state_.current_arrayposition>=slow_down_index_)
		{
		std::cout<<"SLOW DOWN we reached slow_down_index "<<slow_down_index_<<std::endl<<"Distance to end: "<<distanceIJ(state_.current_arrayposition,n_poses_path_-1)<<std::endl;
		//Lineares herrunterschrauben
		C=C*(distanceIJ(state_.current_arrayposition,n_poses_path_-1))/(SLOW_DOWN_DISTANCE);
		}
	//If shut down action is running
	if(gui_stop_==1)//&& time zwischen 0 und pi/2
		{
		std::cout<<"Shutting down gradually"<<std::endl;
		C=C*cos(BigBen_.getTimeFromStart()*1.57079632679/SHUT_DOWN_TIME);	//Zähler ist PI/2.
		}
	float v_ref=v_limit*C;
//Upper buonds
	//upper limit, HERE 25 m/s;
	if(v_ref>MAX_ABSOLUTE_VELOCITY)
		{
		std::cout<<"Upper limit of 25 reached. "<<v_ref<<" is too fast"<<std::endl;
		v_ref=MAX_ABSOLUTE_VELOCITY;
		}
	//not too divergent from teach part
	float v_teach=sqrt(	pow(path_diff_.poses[state_.current_arrayposition].pose.position.x,2) +
					pow(path_diff_.poses[state_.current_arrayposition].pose.position.y,2) +
					pow(path_diff_.poses[state_.current_arrayposition].pose.position.z,2));
	pure_pursuit_gui_msg_.data[8]=v_teach+V_FREEDOM;
	if(v_ref>v_teach+V_FREEDOM)
		{
		std::cout<<"Too divergent from teach velocity" <<std::endl;
		v_ref=v_teach+V_FREEDOM;
		}

	//Speichern auf Stellgrössen
	u_.speed=v_ref;
	pure_pursuit_gui_msg_.data[9]=u_.speed;
	u_.acceleration=v_abs_;
	std::cout<<"lateral error= "<<tracking_error_<<std::endl<<"brake_dist= "<<brake_dist<<std::endl<<"v_limit= "<<v_limit<<std::endl<<"v_abs_= "<<v_abs_<<std::endl<<"C= "<<C<<std::endl<<"v_teach= "<<v_teach<<std::endl;
}//"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
// Method which publishes the calculated commands onto the topic to the system engineers interface node.
void PurePursuit::publishU()
{
	pub_stellgroessen_.publish(u_);
	gui_pub_.publish(pure_pursuit_gui_msg_);
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
		path_diff_.poses.push_back(temp_pose);
		stream>>j;
		stream>>path_.poses[j-1].pose.position.x;
		stream>>path_.poses[j-1].pose.position.y;
		stream>>path_.poses[j-1].pose.position.z;
		//Save orientation
		stream>>path_.poses[j-1].pose.orientation.x;
		stream>>path_.poses[j-1].pose.orientation.y;
		stream>>path_.poses[j-1].pose.orientation.z;
		stream>>path_.poses[j-1].pose.orientation.w;
		//Save teach_velocity
		stream>>path_diff_.poses[j-1].pose.position.x;
		stream>>path_diff_.poses[j-1].pose.position.y;
		stream>>path_diff_.poses[j-1].pose.position.z;

		stream.ignore (300, '|');
		i++;
		if(j-1>n_poses_path_-1){std::cout<<"!!!!!!!!LAUFZEITFEHLER::readPathFromTxt!!!!!!!!!!!"<<std::endl;}
	}
	n_poses_path_ = i;
	float l_dumb=0;
	i=n_poses_path_;
	while(l_dumb<SLOW_DOWN_DISTANCE)
		{
		l_dumb+=distanceIJ(i-1,i);//sqrt(pow(path_.poses[i-1].pose.position.x-path_.poses[i-2].pose.position.x,2)+pow(path_.poses[i-1].pose.position.y-path_.poses[i-2].pose.position.y,2)+pow(path_.poses[i-1].pose.position.z-path_.poses[i-2].pose.position.z,2));
		i--;
		}
	slow_down_index_=i;
}

//No need if in state msg current_arrajposition is given
/*
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
	// Variable which will be used for iteration and then used for setting global_.
	float j = 0;
	// Variable used for publishing the cross-track error.
	std_msgs::Float64 err;

	// Iterate 200 point ahead of last current array position (global_). Either 200 forward or the last path point.
	for(int i = global_; i<global_+200 && i<n_poses_path_; i++)
	{
		float x_path = path_.poses[i].pose.position.x;
		float y_path = path_.poses[i].pose.position.y;
		float d_new = sqrt(pow((x_now - x_path),2) + pow((y_now - y_path),2));
		if(i>n_poses_path_-1){std::cout<<"!!!!!!!!LAUFZEITFEHLER::nearestPoint!!!!!!!!!!!"<<std::endl;}
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
	// Set a current array position which can't be overwritten from stateCallback.
	global_ = j;
	// Display the current cross-track error.
	std::cout<<"Cross Track Error in cm = "<<d_old*100<<std::endl;
	// Save the cross-track error to a member variable.
	tracking_error_ = d_old;
	err.data = tracking_error_;
	// Publish the cross-track error.
	track_error_pub_.publish(err);
	// Return the index of the nearest point.
	return x_projected[2];
}
*/

float PurePursuit::distanceIJ(int from_i , int to_i )	//Achtung indices: es muss auch für to_i=from_i+1 gehen, sonst unendliche schleife bei readpathfrom txt;
{	
	float d=0;
	for (int i =from_i; i<to_i; i++)
	{
		d += sqrt(	pow(path_.poses[i].pose.position.x - path_.poses[i+1].pose.position.x,2)+
				pow(path_.poses[i].pose.position.y - path_.poses[i+1].pose.position.y,2)+
				pow(path_.poses[i].pose.position.z - path_.poses[i+1].pose.position.z,2));
		if((i+1)>n_poses_path_-1){std::cout<<"!!!!!!!!LAUFZEITFEHLER::distanceIJ!!!!!!!!!!!"<<std::endl;}
	}
	return d;

}

float PurePursuit::curveRadius(int j)
{	
	int count=0;
	float r_sum=0;
	for(int t=1;t<=3;t++)
	{	
		count++;
		float D=D_INTERPOLATION/t;
		int i=j;
		int n_front=indexOfDistanceFront(i-1,D);
		int n_back=indexOfDistanceBack(i-1,D);
		std::cout<<"Central Index: "<<i<<std::endl<<"Front Index: "<<n_front<<std::endl<<"Back Index: "<<n_back<<std::endl;
		if(n_back<=0)
		{
			i=indexOfDistanceFront(0,D);
			n_front=indexOfDistanceFront(i-1,D);
			n_back=indexOfDistanceBack(i-1,D);
			std::cout<<"Hintere Grenze"<<std::endl;
		std::cout<<"New Central Index: "<<i<<std::endl<<"New Front Index: "<<n_front<<std::endl<<"New Back Index: "<<n_back<<std::endl;
		}
		else if(n_front>=n_poses_path_-1)
		{
			i=indexOfDistanceBack(n_poses_path_-1,D);
			n_front=indexOfDistanceFront(i-1,D);
			n_back=indexOfDistanceBack(i-1,D);
			std::cout<<"Vordere Grenze"<<std::endl;
		std::cout<<"New Central Index: "<<i<<std::endl<<"New Front Index: "<<n_front<<std::endl<<"New Back Index: "<<n_back<<std::endl;
		}

		Eigen::Vector3d i_back(		path_.poses[n_back].pose.position.x-path_.poses[i].pose.position.x,
							path_.poses[n_back].pose.position.y-path_.poses[i].pose.position.y,
							path_.poses[n_back].pose.position.z-path_.poses[i].pose.position.z);
		Eigen::Vector3d i_front(	path_.poses[n_front].pose.position.x-path_.poses[i].pose.position.x,
							path_.poses[n_front].pose.position.y-path_.poses[i].pose.position.y,
							path_.poses[n_front].pose.position.z-path_.poses[i].pose.position.z);
		Eigen::Vector3d back_front(	path_.poses[n_front].pose.position.x-path_.poses[n_back].pose.position.x,
							path_.poses[n_front].pose.position.y-path_.poses[n_back].pose.position.y,
							path_.poses[n_front].pose.position.z-path_.poses[n_back].pose.position.z);
		if((n_back>n_poses_path_-1)&&(n_front>n_poses_path_-1)&&(i>n_poses_path_-1))
			{std::cout<<"!!!!!!!!LAUFZEITFEHLER::curveRadius!!!!!!!!!!!"<<std::endl;}
		float zaehler =i_back.dot(-i_front);
		float nenner = (i_back.norm()*i_front.norm());
		std::cout<<"argument: "<<zaehler/nenner<<std::endl;
		float gamma=acos(zaehler/nenner);//winkel zwischen Vektoren 
		std::cout<<"winkel: "<<gamma<<std::endl;
		if(sin(gamma)==0) 
		{
			std::cout<<"straight!"<<std::endl;
			r_sum+=9999999;		//irgendeine grosse zahl um nicht nan zu erzeugen in nächster zeile
		}
		else
		{
		r_sum+=back_front.norm()/(2*sin(gamma));	//Gleichung umkreis
		}
	}
	std::cout<<"Count= "<<count<<std::endl<<"r_sum= "<<r_sum<<std::endl;
	float r=r_sum/count;
	pure_pursuit_gui_msg_.data[5]=r;
	return r;
}

int PurePursuit::indexOfDistanceFront(int i, float d)
{
	int j=i;
	float l = 0;
	while(l<d &&j<n_poses_path_-1)
	{
		l += sqrt(	pow(path_.poses[j+1].pose.position.x - path_.poses[j].pose.position.x,2)+
				pow(path_.poses[j+1].pose.position.y - path_.poses[j].pose.position.y,2)+
				pow(path_.poses[j+1].pose.position.z - path_.poses[j].pose.position.z,2));
		if(j+1>n_poses_path_-1){std::cout<<"!!!!!!!!LAUFZEITFEHLER::indexOfDistanceFront!!!!!!!!!!!"<<std::endl;}
		j ++;
	}
	std::cout<<"effective front distance= "<<l<<std::endl;
	return j+1;
}

int PurePursuit::indexOfDistanceBack(int i, float d)
{
	int j=i;
	float l = 0;
	while(l<d && j>0)
	{
		l += sqrt(	pow(path_.poses[j-1].pose.position.x - path_.poses[j].pose.position.x,2)+
				pow(path_.poses[j-1].pose.position.y - path_.poses[j].pose.position.y,2)+
				pow(path_.poses[j-1].pose.position.z - path_.poses[j].pose.position.z,2));
		if(j>n_poses_path_-1){std::cout<<"!!!!!!!!LAUFZEITFEHLER::indexOfDistanceBack!!!!!!!!!!!"<<std::endl;}
		j --;
	}
	std::cout<<"effective back distance= "<<l<<std::endl;
	return j;
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


