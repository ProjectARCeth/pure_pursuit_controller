#include"arc_msgs/State.h"
#include"ros/ros.h"
#include"nav_msgs/Path.h"
#include"std_msgs/Float64.h"
#include"math.h"     
#include"std_msgs/Float64MultiArray.h"

arc_msgs::State status;
nav_msgs::Path pathus;
std::vector<float>x_curr;
std::vector<float>v_curr;
float R=100;
float k=1;
float L=3;

void safe_the_state(const arc_msgs::State subscribed)
{
status=subscribed;
x_curr[0]=subscribed.pose.pose.position.x;
x_curr[1]=subscribed.pose.pose.position.y;
v_curr[0]=subscribed.pose_diff.pose.position.x;
v_curr[1]=subscribed.pose_diff.pose.position.y;

}
void safe_the_path(const nav_msgs::Path subscribed)
{pathus=subscribed;}

std::vector<float> pfad(float s)
{
std::vector<float> x_pfad;	//provisorisch, bevor mit Pfadpunkten gearbeiteet wird
x_pfad[0]=R*cos(s);
x_pfad[1]=R*sin(s);
return x_pfad;
}

float find_j(float l)
{
float e=100;
int j=0;
	for(float i=status.current_arrayposition; i<(status.current_arrayposition+3); i=i+0.1)	//je nach Pfad grenzen und schritte
												//einstellen
	{
	float x_pfad=pfad(i)[0];
	float y_pfad=pfad(i)[1];
	float x_now=x_curr[0];
	float y_now=x_curr[1];
	float d=abs(l-sqrt(pow((x_now-x_pfad),2)+pow((y_now-y_pfad),2)));
	if(d<e)
		{
		e=d;
		j=i;
		}	
	}

return j;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_pursuit");
  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("state", 1000, safe_the_state);
  ros::Subscriber sub2 = n.subscribe("path", 1000, safe_the_path);
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray >("stellgroessen", 1000);

float v_abs=sqrt(v_curr[1]*v_curr[1]+v_curr[0]*v_curr[0]);
float l=k*v_abs;
float j=find_j(l);
float theta1=atan2(pfad(j)[1],pfad(j)[0]);
float theta2=status.pose.pose.orientation.z;
float alpha=theta1-theta2;

std_msgs::Float64MultiArray msg;		//Float64 ist erstellte message...Float64.data ist der Herz an den wir ran wollen
msg.data[0]=atan2(2*L*sin(alpha),l);
msg.data[1]=v_abs;				

pub.publish(msg);		//Publizieren auf Topic stellgroessen floatarray mit steering angle und velocity (nicht accelleration)

ros::spinOnce();		//Wozu??

return 0;
}				//run... Segmentation fault (core dumped)

