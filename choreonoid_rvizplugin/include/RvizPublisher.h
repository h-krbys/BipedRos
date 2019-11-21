#ifndef CNOID_RVIZ_PLUGIN_RVIZ_PUBLISHER_H
#define CNOID_RVIZ_PLUGIN_RVIZ_PUBLISHER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <cnoid/SimpleController>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>

struct PlotData {
  geometry_msgs::Pose pose;
  Eigen::Vector3f     cop, com, icp;
  Eigen::Vector3f     foot_r, foot_l;
};

class RvizPublisher
{
public:
  RvizPublisher();
  RvizPublisher(const RvizPublisher& org);
  ~RvizPublisher();

  void setTimeStep(double timestep);

  void setPose(cnoid::BodyPtr body);
  void setCop(Eigen::Vector3f cop);

  void simulation(double time);
  void playback(double time);

  void publishPose();
  void publishCop();

private:
  ros::NodeHandle *nh;

  ros::Publisher pubCop;

  Eigen::Vector3f cop, com, icp;
  Eigen::Vector3f foot_r, foot_l;

  double dt;

  std::vector<PlotData> data;
};

#endif