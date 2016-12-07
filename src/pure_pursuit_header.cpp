#include"pure_pursuit_header.hpp"	//?muss ich sie auch hier includen?
PurePursuit::PurePursuit(){}
PurePursuit::PurePursuit(float k1,ros::NodeHandle* n)		//Konstruktor mit parameter k einstellbar und speichert den Pfad als "path"
	{
	k_=k1;
	manual_u_=false;
	n_=n;
	global=0;
	state_.current_arrayposition=0;
	sub_path_ = n_->subscribe("path", 5,&PurePursuit::safeThePath,this);
	sub_state_ = n_->subscribe("state", 10, &PurePursuit::sts,this);
	pub_stellgroessen_ = n_->advertise<ackermann_msgs::AckermannDrive>("stellgroessen", 10);
	}

void PurePursuit::safeThePath(const nav_msgs::Path::ConstPtr& subscribed)
	{
	path_=*subscribed;
	//this->calculateU();
	//this->publishU();

	}

void PurePursuit::sts(const arc_msgs::State::ConstPtr& subscribed)
	{
	state_=*subscribed;
	this->calculateU();
	this->publishU();
	}



float* PurePursuit::pathInfo(float where)			//hier interpoliert und info at "where" (derivative ecc) in ARRAY zurück
	{							//provisorisch, bevor mit Pfadpunkten gearbeiteet wird
	float R=30;						//array:{x_coordinate,y_coordinate,..}
	float x_pfad[2]={0,(where)};			
	float *pointer;
	pointer= x_pfad ;
	return pointer;
	}

ackermann_msgs::AckermannDrive PurePursuit::getU()		//u wird zurückgegeben
	{
	ackermann_msgs::AckermannDrive u1=u_;
	return u1;
	}



void PurePursuit::setU(ackermann_msgs::AckermannDrive u)	//manuelles überschreiben von u möglich
	{
	this->u_=u;
	manual_u_=true;
	}

void PurePursuit::calculateU()					//Schritte um u zu berechnen (Reglerspezyfisch)
	{
	if(manual_u_==false)
	{
	float v=10.0;
	float L=3.0;
	float v_abs=sqrt(pow(state_.pose_diff.twist.linear.x,2)+pow(state_.pose_diff.twist.linear.x,2));
	float l=10;
	float j=findReference(l);
	float theta1=atan2(pathInfo(j)[1]-(state_.pose.pose.position.y),pathInfo(j)[0]-(state_.pose.pose.position.x));
	//letzte zwei Zeilen oder
  //float j=projectOnPath()[2];
  //float theta1;
	//if(int(j+l)<(sizeof(path_.poses)/sizeof(path_.poses[0]))-3)
	//{
	//	theta1=1;
	//} //atan2(path_.poses[int(j+l)].pose.position.x-(state_.pose.pose.position.x),path_.poses[int(j+l)].pose.position.y-	(state_.pose.pose.position.y));}
	//else
	//{
	//  int ende=(sizeof(path_.poses)/sizeof(path_.poses[0]));
	//  theta1=atan2(path_.poses[ende].pose.position.x-(state_.pose.pose.position.x),path_.poses[ende].pose.position.y-	(state_.pose.pose.position.y));
	//}
	float ox=state_.pose.pose.orientation.x;		//Transformation von Quaternion zu Euler
	float oy=state_.pose.pose.orientation.y;
	float oz=state_.pose.pose.orientation.z;
	float ow=state_.pose.pose.orientation.w;
	const Eigen::Vector4d quat(ox, oy, oz, ow);
	geometry_msgs::Vector3 eul;
	eul=arc_tools::transformEulerQuaternion(quat);
	float theta2=-eul.z;

	float alpha=theta1-theta2;
		//float x_j=path.poses[10].pose.position.x;
  std::cout<<alpha<<std::endl<<theta1<<std::endl<<theta2;
	u_.speed=v;
	u_.steering_angle=atan2(2*L*sin(alpha),l);
	}
	}

float PurePursuit::findReference(float l)		//erste referenz wird zw pathInfo(0) und pathInfo(200) gesucht
	{
	float e=100;
	float j;
	for(float i=global; i<(global+200); i=i+0.1)	//je nach Pfad grenzen und schritte
												//einstellen
		{
		float x_pfad=pathInfo(i)[0];
		float y_pfad=pathInfo(i)[1];
		float x_now=state_.pose.pose.position.x;
		float y_now=state_.pose.pose.position.y;
		float d=fabs(l-sqrt(pow((x_now-x_pfad),2)+pow((y_now-y_pfad),2)));
		if(d<e)
			{
			e=d;
			j=i;
			}
		}
	global=j;
	return j;
	}

void PurePursuit::publishU()
	{
	pub_stellgroessen_.publish(u_);
	std::cout<<state_.current_arrayposition<< std::endl;
	}

float* PurePursuit::projectOnPath()					//gibt i.a. nicht Punkt
	{
								//aus path zurück, sondern zwischen zwei...
	float x_projected[3];						//Eintrag 1,2 koordinaten, eintrag 3 currentarrayposition
	float* x_projected_p=NULL;

	float ox=state_.pose.pose.orientation.x;		//Transformation von Quaternion zu Euler
	float oy=state_.pose.pose.orientation.y;
	float oz=state_.pose.pose.orientation.z;
	float ow=state_.pose.pose.orientation.w;
	const Eigen::Vector4d quat(ox, oy, oz, ow);
	geometry_msgs::Vector3 eul;
	eul=arc_tools::transformEulerQuaternion(quat);
	float theta=eul.z;

	float x_now=state_.pose.pose.position.x;
	float y_now=state_.pose.pose.position.y;
	for(int i=0;i<(sizeof(path_.poses)/sizeof(path_.poses[0]))-1;i++)		//schaut alle LINEAREN Interpolationen
										//zw punktepaaren durch
		{
		float x_j=path_.poses[i+1].pose.position.x;
		float x_i=path_.poses[i].pose.position.x;
		float y_j=path_.poses[i+1].pose.position.y;
		float y_i=path_.poses[i].pose.position.y;

		float lambda=(tan(theta)*(y_i-y_now)-(x_i-x_now))/((x_j-x_i)-tan(theta)*(y_j-y_i));	//analytisch
		if(0<=lambda&&lambda<=1)
			{
			x_projected[0]=x_i+lambda*(x_j-x_i);
			x_projected[1]=y_i+lambda*(y_j-y_i);
			x_projected[2]=i;
			x_projected_p=&x_projected[0];
			return x_projected_p;
			}
		}

	if(&x_projected_p==NULL){std::cout<<"Fehler, senkrecht zu Fahrzeug liegt kein Pfad";}
	return x_projected_p;						//Sollte eine fehlermeldung geben
	}

void PurePursuit::setState(float x, float y)
	{
	state_.pose.pose.position.x=x;
	state_.pose.pose.position.y=y;
	}

arc_msgs::State PurePursuit::getState() 
	{
	return state_;
	}
