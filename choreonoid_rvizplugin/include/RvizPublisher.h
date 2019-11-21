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
  void setComRef(Eigen::Vector3f comRef);
  void setCopRef(Eigen::Vector3f copRef);
  void setIcpRef(Eigen::Vector3f icpRef);
  void setCom(Eigen::Vector3f com);
  void setCop(Eigen::Vector3f cop);
  void setIcp(Eigen::Vector3f icp);

  void simulation(double time);
  void playback(double time);

  void publishPose();
  void publishComRef();
  void publishCopRef();
  void publishIcpRef();
  void publishCom();
  void publishCop();
  void publishIcp();

private:
  ros::NodeHandle *nh;

  ros::Publisher pubComRef, pubCopRef, pubIcpRef;
  ros::Publisher pubCom, pubCop, pubIcp;

  Eigen::Vector3f copRef, comRef, icpRef;
  Eigen::Vector3f cop, com, icp;
  Eigen::Vector3f foot_r, foot_l;

  double dt;

  std::vector<PlotData> data;

  const double markerSize;
  const double lineWidth;
};

#endif