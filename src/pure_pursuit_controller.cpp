#include "../include/pure_pursuit_controller/pure_pursuit_controller.hpp"
//Parameters
float K1_LAD_S;
float K2_LAD_S;
float K1_LAD_V;
float K2_LAD_V;
float MU_HAFT; // = 0.8 ungefähr
float G_EARTH;
float MAX_LATERAL_ACCELERATION;
float MAX_ABSOLUTE_VELOCITY;
float DISTANCE_WHEEL_AXIS;
float FOS_VELOCITY;  //[0,1]
float SLOW_DOWN_DISTANCE; 
float SLOW_DOWN_PUFFER;
float V_FREEDOM;
float SHUT_DOWN_TIME;
//std::string FILE_LOCATION_PATH_TXT="/home/moritz/.ros/Paths/Obstacles_Hoengg_teach2.txt";
float DISTANCE_INTERPOLATION;
float CRITICAL_OBSTACLE_DISTANCE;
float UPPERBOUND_LAD_S;
float LOWERBOUND_LAD_S;
int QUEUE_LENGTH;
std::string STELLGROESSEN_TOPIC;
std::string TRACKING_ERROR_TOPIC;
std::string NAVIGATION_INFO_TOPIC;
std::string STATE_TOPIC;
std::string OBSTACLE_DISTANCE_TOPIC;
std::string SHUTDOWN_TOPIC;
std::string PATH_NAME_EDITED;

// Constructors and Destructors.
// Default Constructor.
PurePursuit::PurePursuit(){}
// Individual Constructor.
PurePursuit::PurePursuit(ros::NodeHandle* n, std::string PATH_NAME )
{	

	
	n->getParam("/control/K1_LAD_S", K1_LAD_S);
	n->getParam("/control/K2_LAD_S", K2_LAD_S);
	n->getParam("/control/UPPERBOUND_LAD_S", UPPERBOUND_LAD_S);
	n->getParam("/control/LOWERBOUND_LAD_S", LOWERBOUND_LAD_S);
	n->getParam("/control/K1_LAD_V", K1_LAD_V);
	n->getParam("/control/K2_LAD_V", K2_LAD_V);
	n->getParam("/erod/MU_HAFT",MU_HAFT);
	n->getParam("/erod/G_EARTH",G_EARTH);
	n->getParam("/erod/MAX_LATERAL_ACCELERATION",MAX_LATERAL_ACCELERATION);
	n->getParam("/safety/MAX_ABSOLUTE_VELOCITY",MAX_ABSOLUTE_VELOCITY);
	n->getParam("/erod/DISTANCE_WHEEL_AXIS",DISTANCE_WHEEL_AXIS);
	n->getParam("/safety/FOS_VELOCITY",FOS_VELOCITY);
	n->getParam("/control/SLOW_DOWN_DISTANCE",SLOW_DOWN_DISTANCE);
	n->getParam("/control/SLOW_DOWN_PUFFER",SLOW_DOWN_PUFFER);
	n->getParam("/control/V_FREEDOM",V_FREEDOM);
	n->getParam("/control/SHUT_DOWN_TIME",SHUT_DOWN_TIME );
	n->getParam("/control/DISTANCE_INTERPOLATION",DISTANCE_INTERPOLATION );
	n->getParam("/safety/CRITICAL_OBSTACLE_DISTANCE",CRITICAL_OBSTACLE_DISTANCE );
	n->getParam("/general/QUEUE_LENGTH",QUEUE_LENGTH );
	n->getParam("/topic/STELLGROESSEN",STELLGROESSEN_TOPIC);
	n->getParam("/topic/TRACKING_ERROR",TRACKING_ERROR_TOPIC);
	n->getParam("/topic/NAVIGATION_INFO",NAVIGATION_INFO_TOPIC);
	n->getParam("/topic/STATE",STATE_TOPIC);
	n->getParam("/topic/OBSTACLE_DISTANCE",OBSTACLE_DISTANCE_TOPIC);
	n->getParam("/topic/SHUTDOWN",SHUTDOWN_TOPIC);
	PATH_NAME_EDITED = PATH_NAME + "_teach.txt";  //BSP:rosrun pure_pursuit_controller regler_node /home/moritz/.ros/Paths/test

	// 1. Save the arguments to member variables.
	// Set the nodehandle.
	n_ = n;
	// 2. Initialize some member variables.
	// Read in the text file where the teach path is saved and store it to a member variable of type nav_msgs/Path.
	readPathFromTxt(PATH_NAME_EDITED);
	// Initialize gui_stop;
	gui_stop_=0;
	//Initialize obstacle distance in case it is not the first callback function running
	obstacle_distance_=100;
	pure_pursuit_gui_msg_.data.clear();
	for(int i=0;i<10;i++) pure_pursuit_gui_msg_.data.push_back(0);
	// 3. ROS specific setups.
	// Publisher.
	// Publishes the control commands to the interface node (TO VCU).
	pub_stellgroessen_ = n_->advertise<ackermann_msgs::AckermannDrive>(STELLGROESSEN_TOPIC, QUEUE_LENGTH);
	// Publishes the track error. Can be used to test the accuracy of the controller.
	track_error_pub_ = n_->advertise<std_msgs::Float64>(TRACKING_ERROR_TOPIC, QUEUE_LENGTH);
	gui_pub_ = n_->advertise<std_msgs::Float32MultiArray>(NAVIGATION_INFO_TOPIC, QUEUE_LENGTH);
	// Subscriber.
	sub_state_ = n_->subscribe(STATE_TOPIC, QUEUE_LENGTH, &PurePursuit::stateCallback,this);
	distance_to_obstacle_sub_=n_->subscribe(OBSTACLE_DISTANCE_TOPIC, QUEUE_LENGTH ,&PurePursuit::obstacleCallback,this);
	gui_stop_sub_=n_->subscribe(SHUTDOWN_TOPIC, QUEUE_LENGTH ,&PurePursuit::guiStopCallback,this);
	// Construction succesful.
	std::cout << std::endl << "PURE PURSUIT: Consturctor with path lenght: " <<n_poses_path_<< " and slow_down_index: "<<slow_down_index_<<std::endl;
	
/*
fir (int i=0; i<n_poses_path_;i++) std::cout<<"Radius at "<<i<<" is "<<curveRadius(i)<<std::endl;
*/
}	
// Default destructor.
PurePursuit::~PurePursuit(){}

// Callback Function/Method which waits for new state from L&M and then calculates the ideal control commands.
void PurePursuit::stateCallback(const arc_msgs::State::ConstPtr& incoming_state)
{	
	// Save the incoming state (from subscriber) to a member variable.
	state_ = *incoming_state;
	v_abs_=incoming_state->pose_diff;
	float x_path = path_.poses[state_.current_arrayposition].pose.position.x;
	float y_path = path_.poses[state_.current_arrayposition].pose.position.y;
	float z_path = path_.poses[state_.current_arrayposition].pose.position.z;
	float x_now = state_.pose.pose.position.x;
	float y_now = state_.pose.pose.position.y;
	float z_now = state_.pose.pose.position.z;
	tracking_error_ =abs(arc_tools::globalToLocal(path_.poses[state_.current_arrayposition].pose.position, state_).y);
//	tracking_error_ = sqrt(pow((x_now - x_path),2) + pow((y_now - y_path),2) + pow((z_now - z_path),2));
	pure_pursuit_gui_msg_.data[0]=distanceIJ(0,state_.current_arrayposition);
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
	// Empirical linear function to determine the look-ahead-distance.
	float lad = K2_LAD_S + K1_LAD_S*v_abs_;
	lad=std::max(lad,LOWERBOUND_LAD_S);
	lad=std::min(lad,UPPERBOUND_LAD_S);
	int i=indexOfDistanceFront(state_.current_arrayposition,lad);
	float alpha=0;
	if(i>=n_poses_path_-1) i=n_poses_path_-1;
	float l=distanceIJ(state_.current_arrayposition,i);
	pure_pursuit_gui_msg_.data[2]=i;
	geometry_msgs::Point referenz_local=arc_tools::globalToLocal(path_.poses[i].pose.position, state_);
	float dy = referenz_local.y;
	float dx = referenz_local.x;
	std::cout<<"x-local "<<dx<<std::endl<<"y-local "<<dy<<std::endl;
	alpha = atan2(dy,dx);
	u_.steering_angle = atan2(2*DISTANCE_WHEEL_AXIS*sin(alpha),l);
	pure_pursuit_gui_msg_.data[3]=u_.steering_angle;
}
// Method which calculates the ideal speed, using the self-derived empirical formula.
void PurePursuit::calculateVel()
{
    //First calculate optimal velocity
	//for the moment take curvature at fix distance lad_v
	float lad_v= K2_LAD_V + K1_LAD_V*v_abs_;
	//find reference index for curvature
	int i=indexOfDistanceFront(state_.current_arrayposition, lad_v);
	if(i>=n_poses_path_) i=n_poses_path_-1;

	pure_pursuit_gui_msg_.data[4]=i;
	float v_limit=sqrt(MAX_LATERAL_ACCELERATION*curveRadius(i));		//Physik stimmt?
	pure_pursuit_gui_msg_.data[6]=v_limit;
    //Penalisations
	float C=FOS_VELOCITY;
	//penalize lateral error from paht, half for 1m error
	C=C/(1+abs(tracking_error_));
	//Obstacle distance
	float brake_dist=pow(v_abs_*3.6/10,2)/2;	//Physikalisch Sinn??
	pure_pursuit_gui_msg_.data[7]=brake_dist;
	if((obstacle_distance_>brake_dist)&&(obstacle_distance_<2*brake_dist))
	{	
		//std::cout<<"Case1"<<std::endl;	//Comments important at obstacle detection testing
		C=C*(obstacle_distance_/brake_dist)-1;
	}
	else if (obstacle_distance_>=2*brake_dist)
	{
		//std::cout<<"Case2"<<std::endl;
		C=C;
	}
	else if (obstacle_distance_<=brake_dist)
	{
		//std::cout<<"Case3"<<std::endl;
		C=0;
	}
	if(obstacle_distance_<CRITICAL_OBSTACLE_DISTANCE)
	{
		C=0;
	}

	//Orientation error not yet implemented
    //Slow down
	//slow down gradually when arrive at SLOW_DOWN_DISTANCE from end of of path

	if (state_.current_arrayposition>=slow_down_index_)
		{
		std::cout<<"PURE PURSUIT: We reached slow_down_index "<<slow_down_index_<<std::endl<<"Distance to end: "<<distanceIJ(state_.current_arrayposition,n_poses_path_-1)<<std::endl;
		//Lineares herrunterschrauben
		C=C*((distanceIJ(state_.current_arrayposition,n_poses_path_-1))-SLOW_DOWN_PUFFER)/(SLOW_DOWN_DISTANCE);
		}
	//If shut down action is running
	if(gui_stop_==1&&BigBen_.getTimeFromStart()<=SHUT_DOWN_TIME)//&& time zwischen 0 und pi/2
		{
		std::cout<<"PURE PURSUIT: Shutting down gradually"<<std::endl;
		C=C*cos(BigBen_.getTimeFromStart()*1.57079632679/(SHUT_DOWN_TIME));	//Zähler ist PI/2.
		}
	else if (gui_stop_==1 && BigBen_.getTimeFromStart()>SHUT_DOWN_TIME)
		{
		std::cout<<"PURE PURSUIT: Shutted Down"<<std::endl;
		C=0;
		}
	float v_ref=v_limit*C;
//Upper buonds
	//upper limit, HERE 25 m/s;
	if(v_ref>MAX_ABSOLUTE_VELOCITY)
		{
		std::cout<<"PURE PURSUIT: Upper limit of 25 reached. "<<v_ref<<" is too fast"<<std::endl;
		v_ref=MAX_ABSOLUTE_VELOCITY;
		}
	//not too divergent from teach part
	pure_pursuit_gui_msg_.data[8]=teach_vel_[state_.current_arrayposition-1]+V_FREEDOM;
	if(v_ref>teach_vel_[state_.current_arrayposition-1]+V_FREEDOM)
		{
		std::cout<<"PURE PURSUIT: Too divergent from teach velocity" <<std::endl;
		v_ref=teach_vel_[state_.current_arrayposition-1]+V_FREEDOM;
		}

	//Speichern auf Stellgrössen
	u_.speed=v_ref;
	pure_pursuit_gui_msg_.data[9]=u_.speed;
	u_.acceleration=v_abs_;
}
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
		std::cout << "PURE PURSUIT: Error with opening of  " <<inFileName << std::endl;
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
		float temp_diff;
		path_.poses.push_back(temp_pose);
		teach_vel_.push_back(temp_diff);
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
		stream>>teach_vel_[j-1];//path_diff_.poses[j-1].pose.position.x;
						//stream>>path_diff_.poses[j-1].pose.position.y;
						//stream>>path_diff_.poses[j-1].pose.position.z;

		stream.ignore (300, '|');
		i++;
	}
	n_poses_path_ = i;
	float l_dumb=0;
	i=n_poses_path_-1;
	while(l_dumb<SLOW_DOWN_DISTANCE)
		{
		l_dumb+=distanceIJ(i-1,i);
		i--;
		}
	slow_down_index_=i;
}


float PurePursuit::distanceIJ(int from_i , int to_i )	//Achtung indices: es muss auch für to_i=from_i+1 gehen, sonst unendliche schleife bei readpathfrom txt;
{	
	float d=0;
	for (int i =from_i; i<to_i; i++)
	{
		d += sqrt(	pow(path_.poses[i].pose.position.x - path_.poses[i+1].pose.position.x,2)+
				pow(path_.poses[i].pose.position.y - path_.poses[i+1].pose.position.y,2)+
				pow(path_.poses[i].pose.position.z - path_.poses[i+1].pose.position.z,2));
		if((i+1)>n_poses_path_-1){std::cout<<"PURE PURSUIT: LAUFZEITFEHLER distanceIJ"<<std::endl;}
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
		float D=DISTANCE_INTERPOLATION/t;
		int i=j;
		int n_front=indexOfDistanceFront(i-1,D);
		int n_back=indexOfDistanceBack(i-1,D);
		if(n_back<=0)
		{
			i=indexOfDistanceFront(0,D);
			n_front=indexOfDistanceFront(i-1,D);
			n_back=indexOfDistanceBack(i-1,D);
		}
		else if(n_front>=n_poses_path_-1)
		{
			i=indexOfDistanceBack(n_poses_path_-1,D);
			n_front=indexOfDistanceFront(i-1,D);
			n_back=indexOfDistanceBack(i-1,D);
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
			{std::cout<<"PURE PURSUIT: LAUFZEITFEHLER curve radius"<<std::endl;}
		float zaehler =i_back.dot(-i_front);
		float nenner = (i_back.norm()*i_front.norm());
		float gamma=acos(zaehler/nenner);//winkel zwischen Vektoren
		if(sin(gamma)==0) 
		{
			r_sum+=9999999;		//irgendeine grosse Zahl um nicht nan zu erzeugen in nächster zeile
		}
		else
		{
			r_sum+=back_front.norm()/(2*sin(gamma));	//Gleichung umkreis
		}
	}
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
		if(j+1>n_poses_path_-1){std::cout<<"PURE PURSUIT: LAUFZEITFEHLER::indexOfDistanceFront"<<std::endl;}
		j ++;
	}
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
		if(j>n_poses_path_-1){std::cout<<"PURE PURSUIT: LAUFZEITFEHLER indexOfDistanceBack"<<std::endl;}
		j --;
	}
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


