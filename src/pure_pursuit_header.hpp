#pragma once

#include"arc_msgs/State.h"
#include"ackermann_msgs/AckermannDrive.h"
#include"nav_msgs/Path.h"
#include"math.h"
#include"ros/ros.h"
#include"geometry_msgs/PoseStamped.h"
#include"arc_tools/coordinate_transform.hpp"
#include"geometry_msgs/Vector3.h"
#include <iostream>

//void safeThePath(const nav_msgs::Path subscribed);
class PurePursuit{


public :
	PurePursuit();
	PurePursuit(float k1,ros::NodeHandle* n);			//Konstruktor mit parameter k einstellbar
	float* pathInfo(float where);		//hier interpoliert und info at "where" (derivative ecc) in ARRAY zurück
	ackermann_msgs::AckermannDrive getU();	//die im private gespeicherten aktuellen Stellgrössen u werden zurückgegeben
	void safeTheState();			//gibt State zurück
	void setU(ackermann_msgs::AckermannDrive u);	//manuelles überschreiben von u möglich
	void calculateU();				//Schritte um u zu berechnen (Reglerspezyfisch)
	float findReference(float l);	//findet "referenz" (reglerspezyfisch) hier parameter auf kurve
	void publishU();
	void safeThePath(const nav_msgs::Path::ConstPtr& subscribed);
	void sts(const arc_msgs::State::ConstPtr& subscribed);
	float* projectOnPath();		//Noch nicht gebraucht, gibt senkrechte Projection auf pfad, um von dort LAD zu berechnen
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
protected:

};
