#include "Capt.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <vector>

ros::Subscriber             subFootstep;
std::vector<Capt::Footstep> footstep;

void footstepCallback( const nav_msgs::Path::ConstPtr &path){
  footstep.clear();
  for(size_t i = 0; i < path->poses.size(); i++) {
    geometry_msgs::Pose pose = path->poses[i].pose;

    Capt::Footstep step;
    if( ( i % 2 ) == 0) {
      step.suf = Capt::Foot::FOOT_L;
    }else{
      step.suf = Capt::Foot::FOOT_R;
    }
    step.pos.x() = pose.position.x;
    step.pos.y() = pose.position.y;
    step.pos.z() = pose.position.z;

    footstep.push_back(step);
    // printf("%d\n", (int)i);
    // printf(" %1.4lf, %1.4lf, %1.4lf\n", pose.position.x, pose.position.y, pose.position.z);
  }
}

// void plan(){
//
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_planner");
  ros::NodeHandle nh;

  subFootstep = nh.subscribe<nav_msgs::Path>("/footstep_planner/path", 10, &footstepCallback);

  ros::spin();
}