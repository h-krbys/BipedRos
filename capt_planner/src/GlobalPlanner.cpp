#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

ros::Subscriber subStart;
ros::Publisher  pubStart;
ros::Publisher  pubGoal;

void startCallback( const std_msgs::Empty::ConstPtr &empty){
  geometry_msgs::PoseWithCovarianceStamped startPose;
  startPose.header.frame_id         = "map";
  startPose.header.stamp            = ros::Time::now();
  startPose.pose.pose.position.x    = 0;
  startPose.pose.pose.position.y    = 0;
  startPose.pose.pose.position.z    = 0;
  startPose.pose.pose.orientation.x = 0;
  startPose.pose.pose.orientation.y = 0;
  startPose.pose.pose.orientation.z = 0;
  startPose.pose.pose.orientation.w = 1;
  pubStart.publish(startPose);

  geometry_msgs::PoseStamped goalPose;
  goalPose.header.frame_id    = "map";
  goalPose.header.stamp       = ros::Time::now();
  goalPose.pose.position.x    = 2;
  goalPose.pose.position.y    = 0;
  goalPose.pose.position.z    = 0;
  goalPose.pose.orientation.x = 0;
  goalPose.pose.orientation.y = 0;
  goalPose.pose.orientation.z = 0;
  goalPose.pose.orientation.w = 1;
  pubGoal.publish(goalPose);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle nh;

  subStart = nh.subscribe<std_msgs::Empty>("/global/start", 10, &startCallback);
  pubStart = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
  pubGoal  = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);

  // pub = nh.advertise<std_msgs::Int64>("/number_count", 10);
  ros::spin();
}