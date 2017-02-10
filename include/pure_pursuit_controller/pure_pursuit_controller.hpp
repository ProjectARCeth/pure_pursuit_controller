#pragma once

#include"arc_msgs/State.h"
#include"ackermann_msgs/AckermannDrive.h"
#include"nav_msgs/Path.h"
#include"math.h"
#include"ros/ros.h"
#include"geometry_msgs/PoseStamped.h"
#include"arc_tools/coordinate_transform.hpp"
#include"geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <fstream>
#include <sstream>

//void safeThePath(const nav_msgs::Path subscribed);
class PurePursuit{


public :
	PurePursuit();
	//Konstruktor mit parameter k einstellbar	
	PurePursuit(float k1,ros::NodeHandle* n);	
	~PurePursuit();
	//Ein Mal durchführt, liesst Pfad aus Text Datei
	void readPathFromTxt(std::string inFileName);		
	//Parametrisierte Funktion, Optional
	float* pathInfo(float where);	
	float findReference(float l);	
	ackermann_msgs::AckermannDrive getU();
	void safeTheState();			
	//manuelles überschreiben von u möglich
	void setU(ackermann_msgs::AckermannDrive u);	
	//Schritte um u zu berechnen (Reglerspezyfisch)
	void calculateSteer();			
	void calculateAccel();	
	void publishU();
	void sts(const arc_msgs::State::ConstPtr& subscribed);
	float* nearestPoint();
	void setState(float x, float y);
	arc_msgs::State getState();
	void setManual(bool b);
	float curvaturePath(float lad_v);

private :
	float k_;
	ackermann_msgs::AckermannDrive u_;
	nav_msgs::Path path_;
	arc_msgs::State state_;
	ros::NodeHandle* n_;
	bool manual_u_;
	ros::Subscriber sub_state_;
	ros::Subscriber sub_path_;
	ros::Publisher pub_stellgroessen_;
	ros::Publisher path_publisher;
	float global; 
	int n_poses_path_;
	ros::Publisher track_error_pub;
	float lateral_error_;
protected:

};
