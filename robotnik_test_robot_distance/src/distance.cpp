#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <math.h>

using namespace std;

ros::Subscriber odom_sub;
double distance_travelled, lastx, lasty;
int meters;
bool bverbose, bprinted;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_value){
	double x = odom_value->pose.pose.position.x;
	double y = odom_value->pose.pose.position.y;
	
	distance_travelled += sqrt(((x-lastx)*(x-lastx))+((y-lasty)*(y-lasty)));
	
	if(bverbose){
		ROS_INFO("The robot travelled %f meters", distance_travelled);
	}
	else{
		if(!bprinted && ((int)distance_travelled % meters) == 0){
			ROS_INFO("The robot travelled %f meters", distance_travelled);
			bprinted = true;
		}
		else if(((int)distance_travelled % meters) != 0){
			bprinted = false;
		}
	}
	lastx = x;
	lasty = y;
}
	

int main(int argc, char** argv){
  ros::init(argc, argv, "distance");
  std::string odom_topic;
  ros::NodeHandle nh("~");
  distance_travelled = 0.0;
  lastx = 0.0;
  lasty = 0.0;
  bprinted = false;
  nh.param<std::string>("odom_topic", odom_topic, "odom");
  nh.param<int>("meters", meters, 10);
  nh.param<bool>("verbose", bverbose, false);
  
  odom_sub = nh.subscribe(odom_topic, 1, odomCallback);
  
  ros::spin();
  
}
