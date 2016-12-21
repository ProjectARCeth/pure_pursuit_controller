#include"../include/pure_pursuit_controller/pure_pursuit_controller.hpp"
PurePursuit::PurePursuit(){}
PurePursuit::PurePursuit(float k1,ros::NodeHandle* n)
	{
	std::cout<<"Constructor runs"<<std::endl;
	k_=k1;
	manual_u_=false;
	n_=n;
	global=0;
	//Argument ist Pfad von dem .txt file, User-spezyfisch.
	readPathFromTxt("/home/moritz/.ros/Paths/pathRov_openLoop.txt");
	state_.current_arrayposition=0;
	sub_state_ = n_->subscribe("state", 10, &PurePursuit::sts,this);
	pub_stellgroessen_ = n_->advertise<ackermann_msgs::AckermannDrive>("stellgroessen", 10);
	track_error_pub = n_->advertise<std_msgs::Float64>("track_error", 10);
	//Für RVIZ Darstellung.
	path_publisher=n_->advertise<nav_msgs::Path>("/path",10);
	int s=sizeof(path_.poses)/sizeof(path_.poses[0]);
	}
void PurePursuit::sts(const arc_msgs::State::ConstPtr& subscribed)
	{
	
	state_=*subscribed;
	this->calculateU();
	this->publishU();
	this->path_publisher.publish(path_);
	std::cout<<"END OF LOOP"<<std::endl;
	}
void PurePursuit::publishU()
	{
	pub_stellgroessen_.publish(u_);
	}

void PurePursuit::calculateU()					
	{
	if(manual_u_==false)
	{
	float v=10.0;
	float L=3.0;
	float v_abs=sqrt(pow(state_.pose_diff.twist.linear.x,2)+pow(state_.pose_diff.twist.linear.x,2));
	//empirische lineare funktion
	float lad=3+k_*v_abs;	
	float theta1;
//Entweder.
	
	//float j=findReference(lad);		//Pfad unten parametrisiert 
	//float dy=pathInfo(j)[1]-(state_.pose.pose.position.y);
	//float dx=pathInfo(j)[0]-(state_.pose.pose.position.x);
	//float theta1=atan2(dy,dx);
//Oder.
  	float j=nearestPoint()[2];
	float l=0;
	int i=j;
	//Länge skalieren.
	while(l<lad)	  	
		{
		l+=sqrt(pow(path_.poses[i+1].pose.position.x-path_.poses[i].pose.position.x,2)+pow(path_.poses[i+1].pose.position.y-path_.poses[i].pose.position.y,2));
		i=i+1;
		}
	std::cout<<"Nearest index= "<<j<<std::endl<<"Reference index= "<<i<<" "<<l<<std::endl;
	if(i<n_poses_path_-1)
		{	
		float dy=path_.poses[i].pose.position.y-(state_.pose.pose.position.y);
		float dx=path_.poses[i].pose.position.x-(state_.pose.pose.position.x);
		theta1=atan2(dy,dx);
		}
	else
		{
		float dy=path_.poses[n_poses_path_].pose.position.y-(state_.pose.pose.position.y);
		float dx=path_.poses[n_poses_path_].pose.position.x-(state_.pose.pose.position.x);
		theta1=atan2(dy,dx);
		}

//Ende entweder oder.
	//Transformation von Quaternion zu Euler.
	float ox=state_.pose.pose.orientation.x;	
	float oy=state_.pose.pose.orientation.y;
	float oz=state_.pose.pose.orientation.z;
	float ow=state_.pose.pose.orientation.w;
	const Eigen::Vector4d quat(ox, oy, oz, ow);
	geometry_msgs::Vector3 eul;
	eul=arc_tools::transformEulerQuaternionMsg(quat);
	float theta2=-eul.z;
	float alpha=theta1-theta2;
	u_.speed=v;
	u_.steering_angle=atan2(2*L*sin(alpha),l);
	}
	}
//welcher punkt auf Pfad hat gewissen Abstand l von mir.
float PurePursuit::findReference(float l)		
	{
	float e=1000;
	float j;
	for(float i=global; i<(global+200); i=i+0.1)
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
float* PurePursuit::nearestPoint()	
	{
	float x_projected[3];
	float* x_projected_p=&x_projected[0];
	float x_now=state_.pose.pose.position.x;
	float y_now=state_.pose.pose.position.y;
	float d_old=1000;
	float j=0;
	//Wenn am Ende der Strecke bleibt der nearest point der letzte Pfadpunkt.
	for(int i=global;i<global+200&&i<n_poses_path_;i++)
		{
			float x_path=path_.poses[i].pose.position.x;
			float y_path=path_.poses[i].pose.position.y;
			float d_new=sqrt(pow((x_now-x_path),2)+pow((y_now-y_path),2));
			if(d_new<d_old)
				{
				d_old=d_new;
				j=i;
				x_projected[0]=path_.poses[i].pose.position.x;
				x_projected[1]=path_.poses[i].pose.position.y;
				x_projected[2]=i;
				}
		}
	global=j;
	std::cout<<"Cross Track Error in cm = "<<d_old*100<<std::endl;
	std_msgs::Float64 err;
	err.data = d_old;
	track_error_pub.publish(err);

	/*Alternativ:Senkrecht zum auto den punkt auf dem Pfad nehmen... Fehlerheft...
	bool b=0;
	float ox=state_.pose.pose.orientation.x;		//Transformation von Quaternion zu Euler
	float oy=state_.pose.pose.orientation.y;
	float oz=state_.pose.pose.orientation.z;
	float ow=state_.pose.pose.orientation.w;
	const Eigen::Vector4d quat(ox, oy, oz, ow);
	geometry_msgs::Vector3 eul;
	eul=arc_tools::transformEulerQuaternionMsg(quat);
	float theta=eul.z;
	for(int i=0;i<n_poses_path_-10;i++)		//schaut ALLE linearen Interpolationen
										//zw punktepaaren durch
		{
		float x_j=path_.poses[i+1].pose.position.x;
		float x_i=path_.poses[i].pose.position.x;
		float y_j=path_.poses[i+1].pose.position.y;
		float y_i=path_.poses[i].pose.position.y;
		float lambda=(tan(theta)*(y_i-y_now)-(x_i-x_now))/((x_j-x_i)-tan(theta)*(y_j-y_i));	//analytisch
		float dist_to_path=sqrt(pow((x_now-x_i),2)+pow((y_now-y_i),2));
		float dist_to_path_old=100000;
		if(0<=lambda&&lambda<=1&&dist_to_path<dist_to_path_old)
			{
			x_projected[0]=x_i+lambda*(x_j-x_i);
			x_projected[1]=y_i+lambda*(y_j-y_i);
			x_projected[2]=i;
			dist_to_path_old=dist_to_path;
			std::cout<<"ciao"<<std::endl;
			b=1;
			}
		}
	if(b==0)
		{
		std::cout<<"Fehler, senkrecht zu Fahrzeug liegt kein Pfad, es bewegt sicj nun zum Pfadanfang";
		x_projected[0]=0;
		x_projected[1]=0;
		x_projected[2]=5;		
		}*/
	return x_projected_p;
	}
void PurePursuit::readPathFromTxt(std::string inFileName)
	{
	
	std::fstream fin; 	
	fin . open ( inFileName. c_str ()) ;
	if (! fin . is_open () )
		{std::cout << "Error with opening of  " <<inFileName << std::endl ;}
	//Länge des files -2 um letztes '|' wegzuschneiden.
	fin.seekg (-2, fin.end); 
	int length = fin.tellg();
	fin.seekg (0, fin.beg);
	//Stream erstellen mit chars von fin.
	char * file = new char [length];
	fin.read (file,length);
	std::istringstream stream(file,std::ios::in);
	delete[] file;	
	fin . close () ;
	int i=0;
	int j;	
	geometry_msgs::PoseStamped temp_pose;
	//Schreiben des Pfades.
	while(!stream.eof()&& i<length)
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
	n_poses_path_=i;
	std::cout<<"Created path of lenght= "<<n_poses_path_<<std::endl;
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
//Falls parametrisierte Kurve gebraucht wird.
float* PurePursuit::pathInfo(float where)			
	{							
	float R=20;						
	float x_pfad[2]={where,(R*sin(where/15))};
	float *pointer;
	pointer= x_pfad ;
	return pointer;
	}
ackermann_msgs::AckermannDrive PurePursuit::getU()
	{
	ackermann_msgs::AckermannDrive u1=u_;
	return u1;
	}
void PurePursuit::setU(ackermann_msgs::AckermannDrive u)
	{
	this->u_=u;
	manual_u_=true;
	}
void PurePursuit::setManual(bool b)
	{
	manual_u_=b;
	}

PurePursuit::~PurePursuit(){}

