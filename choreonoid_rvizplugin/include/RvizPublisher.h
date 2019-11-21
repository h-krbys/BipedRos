#ifndef CNOID_RVIZ_PLUGIN_RVIZ_PUBLISHER_H
#define CNOID_RVIZ_PLUGIN_RVIZ_PUBLISHER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <cnoid/SimpleController>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>

struct PlotData {
  Eigen::Vector3f              copRef, comRef, icpRef;
  Eigen::Vector3f              cop, com, icp;
  Eigen::Vector3f              footR, footL;
  std::vector<Eigen::Vector3f> footstepR, footstepL;
};

class RvizPublisher
{
public:
  RvizPublisher();
  RvizPublisher(const RvizPublisher& org);
  ~RvizPublisher();

  void initialize();

  void setTimeStep(double timestep);

  void setPose(cnoid::BodyPtr body);
  void setComRef(Eigen::Vector3f comRef);
  void setCopRef(Eigen::Vector3f copRef);
  void setIcpRef(Eigen::Vector3f icpRef);
  void setCom(Eigen::Vector3f com);
  void setCop(Eigen::Vector3f cop);
  void setIcp(Eigen::Vector3f icp);

  void setFootstepR(std::vector<Eigen::Vector3f> footstepR);
  void setFootstepL(std::vector<Eigen::Vector3f> footstepL);

  void simulation(double time);
  void playback(double time);

  // void publishPose();
  void publishComRef();
  void publishCopRef();
  void publishIcpRef();
  void publishCom();
  void publishCop();
  void publishIcp();
  void publishFootstepR();
  void publishFootstepL();

  void publishComRefTraj(double time);
  void publishCopRefTraj(double time);
  void publishIcpRefTraj(double time);
  void publishComTraj(double time);
  void publishCopTraj(double time);
  void publishIcpTraj(double time);
  void publishFootRTraj(double time);
  void publishFootLTraj(double time);

private:
  ros::NodeHandle *nh;

  ros::Publisher pubComRef, pubCopRef, pubIcpRef;
  ros::Publisher pubCom, pubCop, pubIcp;
  ros::Publisher pubFootstepR, pubFootstepL;
  ros::Publisher pubComRefTraj, pubCopRefTraj, pubIcpRefTraj;
  ros::Publisher pubComTraj, pubCopTraj, pubIcpTraj;
  ros::Publisher pubFootRTraj, pubFootLTraj;

  Eigen::Vector3f              copRef, comRef, icpRef;
  Eigen::Vector3f              cop, com, icp;
  Eigen::Vector3f              footR, footL;
  std::vector<Eigen::Vector3f> footstepR, footstepL;

  double dt;

  std::vector<PlotData> data;

  const double markerSize;
  const double lineWidth;
};

#endif