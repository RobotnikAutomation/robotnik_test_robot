#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdlib.h>
#include <string>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void wait_unitl_succeded_or_watchdog(int secs, MoveBaseClient* ac){
	ros::Time t1, t2;
	t1 = ros::Time::now();
	t2 = ros::Time::now();
	while(ac->getState() != actionlib::SimpleClientGoalState::SUCCEEDED && (t2.sec-t1.sec)<secs){
		t2 = ros::Time::now();
	}
}

int main(int argc, char** argv){
  ros::init(argc, argv, "test_square");
  int i, numLaps, watchdog_secs;
  std::string target_frame_id;
  float side_size, max_vel_x, max_vel_theta, acc_lim_x, acc_lim_theta;
  ros::NodeHandle nh("~");
  
  nh.param<int>("numLaps", numLaps, 2);
  nh.param<int>("watchdog_secs", watchdog_secs, 20);
  nh.param<float>("side_size", side_size, 2.0);
  nh.param<float>("max_vel_x", max_vel_x, 2.0);
  nh.param<float>("max_vel_theta", max_vel_theta, 1.0);
  nh.param<float>("acc_lim_x", acc_lim_x, 1.0);
  nh.param<float>("acc_lim_theta", acc_lim_theta, 1.0);
  nh.param<std::string>("target_frame_id", target_frame_id, "odom");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  std::ostringstream ss;
  ss << max_vel_x;
  std::string aux(ss.str());
  system(("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS max_vel_x "+aux).c_str());
  ss.str("");
  ss.clear();
  ss << max_vel_theta;
  aux = ss.str();
  system(("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS max_vel_theta "+aux).c_str());
  ss.str("");
  ss.clear();
  ss << acc_lim_x;
  aux = ss.str();
  system(("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS acc_lim_x "+aux).c_str());
  ss.str("");
  ss.clear();
  ss << acc_lim_theta;
  aux = ss.str();
  system(("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS acc_lim_theta "+aux).c_str());
  
  move_base_msgs::MoveBaseGoal goal;

  for(i=0;i<numLaps;i++){
	  goal.target_pose.header.frame_id = target_frame_id;
	  
	  
	  goal.target_pose.header.stamp = ros::Time::now();

	  goal.target_pose.pose.position.x = side_size/2;
	  goal.target_pose.pose.position.y = 0.0;
	  goal.target_pose.pose.orientation.z = 0;
	  goal.target_pose.pose.orientation.w = 1.0;

	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

	  wait_unitl_succeded_or_watchdog(watchdog_secs, &ac);
	  ac.cancelGoal();
	  
	  goal.target_pose.header.stamp = ros::Time::now();

	  goal.target_pose.pose.position.x = side_size;
	  goal.target_pose.pose.position.y = 0.0;
	  goal.target_pose.pose.orientation.z = 0;
	  goal.target_pose.pose.orientation.w = 1.0;

	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

	  wait_unitl_succeded_or_watchdog(watchdog_secs, &ac);
	  ac.cancelGoal();
	  
	  goal.target_pose.header.stamp = ros::Time::now();

	  goal.target_pose.pose.position.x = side_size;
	  goal.target_pose.pose.position.y = side_size/2;
	  goal.target_pose.pose.orientation.z = 0.71;
	  goal.target_pose.pose.orientation.w = 0.71;

	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

	  wait_unitl_succeded_or_watchdog(watchdog_secs, &ac);
      ac.cancelGoal();
      
	  goal.target_pose.header.stamp = ros::Time::now();

	  goal.target_pose.pose.position.x = side_size;
	  goal.target_pose.pose.position.y = side_size;
	  goal.target_pose.pose.orientation.z = 1;
	  goal.target_pose.pose.orientation.w = 0.0;

	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

	  wait_unitl_succeded_or_watchdog(watchdog_secs, &ac);
      ac.cancelGoal();
      
	  goal.target_pose.header.stamp = ros::Time::now();

	  goal.target_pose.pose.position.x = side_size/2;
	  goal.target_pose.pose.position.y = side_size;
	  goal.target_pose.pose.orientation.z = 1;
	  goal.target_pose.pose.orientation.w = 0.0;

	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

	  wait_unitl_succeded_or_watchdog(watchdog_secs, &ac);
      ac.cancelGoal();
      
	  goal.target_pose.header.stamp = ros::Time::now();

	  goal.target_pose.pose.position.x = 0.0;
	  goal.target_pose.pose.position.y = side_size;
	  goal.target_pose.pose.orientation.z = 1;
	  goal.target_pose.pose.orientation.w = 0.0;

	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

	  wait_unitl_succeded_or_watchdog(watchdog_secs, &ac);
	  ac.cancelGoal();
	  
	  goal.target_pose.header.stamp = ros::Time::now();

	  goal.target_pose.pose.position.x = 0.0;
	  goal.target_pose.pose.position.y = side_size/2;
	  goal.target_pose.pose.orientation.z = 0.71;
	  goal.target_pose.pose.orientation.w = -0.71;

	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

	  wait_unitl_succeded_or_watchdog(watchdog_secs, &ac);
      ac.cancelGoal();
      
	  goal.target_pose.header.stamp = ros::Time::now();

	  goal.target_pose.pose.position.x = 0.0;
	  goal.target_pose.pose.position.y = 0.0;
	  goal.target_pose.pose.orientation.z = 0;
	  goal.target_pose.pose.orientation.w = 1.0;

	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

	  wait_unitl_succeded_or_watchdog(watchdog_secs, &ac);
	  ac.cancelGoal();
  }

  return 0;
}
