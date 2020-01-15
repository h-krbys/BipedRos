#ifndef CNOID_RVIZ_PLUGIN_RVIZ_PUBLISHER_H
#define CNOID_RVIZ_PLUGIN_RVIZ_PUBLISHER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <cnoid/SimpleController>
#include <cnoid/TimeBar>
#include <cnoid/ToolBar>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include <cnoid/SimulatorItem>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>

namespace cnoid {

struct LinkData {
  std::string name;
  double      p_x, p_y, p_z;      // position
  double      q_x, q_y, q_z, q_w; // quaternion
};

struct CaptData {
  Vector3 pos;
  int     nstep;

  void operator=(const CaptData &data) {
    this->pos   = data.pos;
    this->nstep = data.nstep;
  }
};

struct PlotData {
  PlotData(){
    e_copRef      = false; e_comRef = false; e_icpRef = false;
    e_cop         = false; e_com = false; e_icp = false;
    e_footRRef    = false; e_footLRef = false;
    e_footR       = false; e_footL = false;
    e_footstepRef = false; e_footstepR = false; e_footstepL = false;
    e_gridMap     = false;
    e_force       = false;
  }

  std::vector<LinkData> link;
  Vector3               copRef, comRef, icpRef;
  Vector3               cop, com, icp;
  Vector3               footRRef, footLRef;
  Vector3               footR, footL;
  std::vector<Vector3>  footstepRef, footstepR, footstepL;
  std::vector<CaptData> gridMap;
  Vector3               force;

  // e_* means enabled_*
  bool e_copRef, e_comRef, e_icpRef;
  bool e_cop, e_com, e_icp;
  bool e_footRRef, e_footLRef;
  bool e_footR, e_footL;
  bool e_footstepRef, e_footstepR, e_footstepL;
  bool e_gridMap;
  bool e_force;
};

class RvizPublisher
{
public:
  RvizPublisher();
  RvizPublisher(const RvizPublisher& org);
  ~RvizPublisher();

  void initialize();

  void save();

  void setTimeStep(double timestep);

  void setPose(BodyPtr body);
  void setCopRef(Vector3 copRef);
  void setComRef(Vector3 comRef);
  void setIcpRef(Vector3 icpRef);
  void setFootRRef(Vector3 footRRef);
  void setFootLRef(Vector3 footLRef);
  void setCom(Vector3 com);
  void setCop(Vector3 cop);
  void setIcp(Vector3 icp);
  void setFootstepRef(std::vector<Vector3> footstepRef);
  void setFootstepR(std::vector<Vector3> footstepR);
  void setFootstepL(std::vector<Vector3> footstepL);
  void setGridMap(std::vector<CaptData> gridMap);
  void setForce(Vector3 force);

  bool timeChanged(double time);
  void simulation(double time);
  void playback(double time);

  void publishAll(double time);

  void publishPose();
  void publishCopRef();
  void publishComRef();
  void publishIcpRef();
  void publishFootRRef();
  void publishFootLRef();
  void publishCom();
  void publishCop();
  void publishIcp();
  void publishFootstepRef();
  void publishFootstepR();
  void publishFootstepL();
  void publishGridMap();
  void publishForce();
  void publishPendulumRef();
  void publishPendulum();

  void publishComRefTraj(double time);
  void publishCopRefTraj(double time);
  void publishIcpRefTraj(double time);
  void publishFootRRefTraj(double time);
  void publishFootLRefTraj(double time);
  void publishComTraj(double time);
  void publishCopTraj(double time);
  void publishIcpTraj(double time);
  void publishFootRTraj(double time);
  void publishFootLTraj(double time);

private:
  ros::NodeHandle *nh;

  ros::Publisher pubComRef, pubCopRef, pubIcpRef;
  ros::Publisher pubCom, pubCop, pubIcp;
  ros::Publisher pubFootRRef, pubFootLRef;
  ros::Publisher pubFootstepRef, pubFootstepR, pubFootstepL;
  ros::Publisher pubComRefTraj, pubCopRefTraj, pubIcpRefTraj;
  ros::Publisher pubComTraj, pubCopTraj, pubIcpTraj;
  ros::Publisher pubFootRRefTraj, pubFootLRefTraj;
  ros::Publisher pubFootRTraj, pubFootLTraj;
  ros::Publisher pubGridMap;
  ros::Publisher pubForce;
  ros::Publisher pubPendulumRef, pubPendulum;

  std::vector<LinkData> link;
  Vector3               copRef, comRef, icpRef;
  Vector3               cop, com, icp;
  Vector3               footRRef, footLRef;
  Vector3               footR, footL;
  std::vector<Vector3>  footstepRef, footstepR, footstepL;
  std::vector<CaptData> gridMap;
  Vector3               force;

  bool e_copRef, e_comRef, e_icpRef;
  bool e_cop, e_com, e_icp;
  bool e_footRRef, e_footLRef;
  bool e_footR, e_footL;
  bool e_footstepRef, e_footstepR, e_footstepL;
  bool e_gridMap;
  bool e_force;

  double dt, maxTime;

  PlotData              data_; // temporary
  std::vector<PlotData> data;

  const double markerSize;
  const double lineWidth;
  const double forceScale;

  // connect signal & slot
  TimeBar   *timeBar;
  Connection timeBarConnection;
  bool       isSimulation;
  bool       isPlayback;
};

}

#endif