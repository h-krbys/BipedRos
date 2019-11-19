#ifndef CNOID_RVIZ_PLUGIN_STATE_PUBLISHER_ITEM_H
#define CNOID_RVIZ_PLUGIN_STATE_PUBLISHER_ITEM_H

#include <cnoid/ControllerItem>
#include <cnoid/BodyItem>
#include <cnoid/ItemManager>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <memory>
// biped messages
#include <biped_msgs/RvPoint.h>
#include <biped_msgs/RvLine.h>
#include <biped_msgs/RvCircle.h>
#include <biped_msgs/RvSquare.h>
#include <biped_msgs/RvArrow.h>
#include <biped_msgs/RvStl.h>

namespace cnoid {

struct StateData {
  Vector3f cop, com, icp;
  Vector3f foot_r, foot_l;
};

class CNOID_EXPORT StatePublisherItem : public Item
{
public:
  static void initialize(ExtensionManager* ext);

  StatePublisherItem();
  StatePublisherItem(const StatePublisherItem& org);
  ~StatePublisherItem();

  void setTimeStep(double dt);

  void setBody(const BodyPtr ioBody);

  void setCop(Vector3f cop);
  void setCom(Vector3f com);
  void setIcp(Vector3f icp);
  void setFootR(Vector3f pos);
  void setFootL(Vector3f pos);
  void setFootstepR(std::vector<Vector3f> pos);
  void setFootstepL(std::vector<Vector3f> pos);

  void enableCopTraj(bool flag);
  void enableComTraj(bool flag);
  void enableIcpTraj(bool flag);
  void enableFootTraj(bool flag);

  void update();
  void playback(double t);

private:
  ros::NodeHandle *nh;
  ros::Publisher   pub_joints, pub_body;
  ros::Publisher   pub_point, pub_line, pub_circle;
  ros::Publisher   pub_square, pub_arrow, pub_stl;

  double dt;

  Vector3f              cop, com, icp;
  Vector3f              foot_r, foot_l;
  std::vector<Vector3f> footstep_r, footstep_l;

  bool copTraj, comTraj, icpTraj, footTraj;

  std::vector<StateData> data;
};

}

#endif