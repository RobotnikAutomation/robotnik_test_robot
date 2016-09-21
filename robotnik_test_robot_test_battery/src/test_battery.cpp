#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h> 

using namespace std;

ros::Subscriber battery_sub;
float fmin_volt, fmax_volt;
bool bverbose;

void batteryCallback(const std_msgs::Float32::ConstPtr& msg){
	float volt = msg->data;
	int percent = ((volt-fmin_volt)*100)/(fmax_volt-fmin_volt);
	
	if(percent < 25){
		ROS_WARN("Battery at %d percent, please connect the charger", percent);
	}
	if(percent < 5){
		ROS_ERROR("Shutting down is inminent, connect the charger now");
	}
	if(bverbose){
		ROS_INFO("Battery at %d percent", percent);
	}
}
	

int main(int argc, char** argv){
  ros::init(argc, argv, "test_battery");
  std::string battery_topic;
  ros::NodeHandle nh("~");
  nh.param<std::string>("battery_topic", battery_topic, "battery");
  nh.param<float>("min_volt", fmin_volt, 23.0);
  nh.param<float>("max_volt", fmax_volt, 26.0);
  nh.param<bool>("verbose", bverbose, false);
  
  battery_sub = nh.subscribe(battery_topic, 1, batteryCallback);
  
  ros::spin();
  
}
