#include "Capt.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <vector>

ros::Subscriber subFootstep;
Capt::Footstep  footstep;

void footstepCallback( const nav_msgs::Path::ConstPtr &path){
  footstep.clear();
  for(size_t i = 0; i < path->poses.size(); i++) {
    geometry_msgs::Pose pose = path->poses[i].pose;

    Capt::Step step;
    if( ( (int)i % 2 ) == 0) {
      step.suf = Capt::Foot::FOOT_L;
    }else{
      step.suf = Capt::Foot::FOOT_R;
    }
    step.pos.x() = pose.position.x;
    step.pos.y() = pose.position.y;
    step.pos.z() = pose.position.z;

    footstep.push_back(step);
  }
}

void plan(double t){

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_planner");
  ros::NodeHandle nh;
  subFootstep = nh.subscribe<nav_msgs::Path>("/footstep_planner/path", 10, &footstepCallback);

  double    t = 0;
  ros::Rate loop_rate(10);
  while (ros::ok() ) {
    ros::spinOnce();
    plan(t);
    loop_rate.sleep();
    t += 0.01;
  }

  // ros::spin();

  return 0;
}