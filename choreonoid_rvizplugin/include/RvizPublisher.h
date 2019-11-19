#ifndef CNOID_RVIZ_PLUGIN_RVIZ_PUBLISHER_H
#define CNOID_RVIZ_PLUGIN_RVIZ_PUBLISHER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>

struct PlotData {
  float           joint[43];
  Eigen::Vector3f cop, com, icp;
  Eigen::Vector3f foot_r, foot_l;
};

class RvizPublisher
{
public:
  RvizPublisher();
  RvizPublisher(const RvizPublisher& org);
  ~RvizPublisher();

  void callbackTimeStep(const std_msgs::Float64::ConstPtr &dt);
  void callbackSimulation(const std_msgs::Float64::ConstPtr &t);
  void callbackPlayback(const std_msgs::Float64::ConstPtr &t);

  void callbackBody(const geometry_msgs::Pose::ConstPtr &body);
  void callbackJoint(const std_msgs::Float64MultiArray::ConstPtr &joint);
  void callbackCop(const geometry_msgs::Point::ConstPtr &cop);

  void publishJoint();
  void publishBody();
  void publishCop();

private:
  ros::NodeHandle nh;

  ros::Subscriber subTimeStep, subSimulation, subPlayback;
  ros::Subscriber subBody, subJoint;
  ros::Subscriber subCop;

  ros::Publisher pubBody, pubJoint;
  ros::Publisher pubCop;

  float           joint[43];
  Eigen::Vector3f cop, com, icp;
  Eigen::Vector3f foot_r, foot_l;

  double dt;

  std::vector<PlotData> data;

  // publishする型
  sensor_msgs::JointState js;
};

#endif