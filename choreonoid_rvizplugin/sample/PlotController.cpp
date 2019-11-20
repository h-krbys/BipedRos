#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include "../include/RvizPublisher.h"
#include <mutex>

using namespace std;
using namespace cnoid;

const double pgain = 10;
const double dgain = 0.0;

class PlotController : public SimpleController
{
  BodyPtr             ioBody;
  double              t, dt;
  std::vector<double> qref;
  std::vector<double> qold;
  int                 phase;

  // virtual body
  BodyPtr                    virtualBody;
  Link                      *virtualBase, *virtualLLeg, *virtualRLeg;
  std::shared_ptr<JointPath> baseToLLeg, baseToRLeg;

  // ros setting
  ros::NodeHandle     *nh;
  ros::Publisher       pubJoint, pubPose, pubCop, pubSim;
  geometry_msgs::Point point;
  std_msgs::Float64    data;

public:
  virtual bool initialize(SimpleControllerIO* io) override
  {
    ioBody = io->body();
    dt     = io->timeStep();

    for(int i = 0; i < ioBody->numJoints(); ++i) {
      Link* joint = ioBody->joint(i);
      joint->setActuationMode(Link::JOINT_VELOCITY);
      io->enableIO(joint);
      qref.push_back(joint->q() );
    }
    qold = qref;

    for (int i = 0; i < ioBody->numLinks(); ++i) {
      io->enableInput(io->body()->link(i), LINK_POSITION);
    }

    // set virtual model
    virtualBody = ioBody->clone();
    virtualLLeg = virtualBody->link("leftFootSole");
    virtualRLeg = virtualBody->link("rightFootSole");
    virtualBase = virtualBody->link("pelvis");
    baseToRLeg  = getCustomJointPath(virtualBody, virtualBase, virtualRLeg);
    baseToLLeg  = getCustomJointPath(virtualBody, virtualBase, virtualLLeg);
    for (int i = 0; i < ioBody->numJoints(); ++i) {
      virtualBody->joint(i)->q() = ioBody->joint(i)->q();
    }
    baseToLLeg->calcForwardKinematics();
    baseToRLeg->calcForwardKinematics();

    phase = 0;

    int    argc = 0;
    char** argv = 0;
    ros::init(argc, argv, "simplecontroller");
    nh = new ros::NodeHandle("");

    pubPose  = nh->advertise<geometry_msgs::Pose>("/simulation/pose", 1000);
    pubJoint = nh->advertise<std_msgs::Float64MultiArray>("/simulation/joint", 1000);
    pubCop   = nh->advertise<geometry_msgs::Point>("/simulation/cop", 1000);
    pubSim   = nh->advertise<std_msgs::Float64>("/simulation/time", 1000);

    t = 0.0;

    return true;
  }

  virtual bool control() override
  {
    point.x = 1.0;
    point.y = 1.0;
    point.z = 1.0;
    pubCop.publish(point);

    Position posLLeg = baseToLLeg->endLink()->position();
    Position posRLeg = baseToRLeg->endLink()->position();

    switch(phase) {
    case 0:
      posLLeg.translation().z() += 0.1 * dt;
      posRLeg.translation().z() += 0.1 * dt;
      break;
    case 1:
      posLLeg.translation().x() += 0.05 * dt;
      posRLeg.translation().x() += 0.05 * dt;
      break;
    case 2:
      posLLeg.translation().x() -= 0.05 * dt;
      posRLeg.translation().x() -= 0.05 * dt;
      break;
    case 3:
      posLLeg.translation().y() += 0.05 * dt;
      posRLeg.translation().y() += 0.05 * dt;
      break;
    case 4:
      posLLeg.translation().y() -= 0.05 * dt;
      posRLeg.translation().y() -= 0.05 * dt;
      break;
    case 5:
      posLLeg.translation().z() -= 0.09 * dt;
      posRLeg.translation().z() -= 0.09 * dt;
      break;
    case 6:
      phase = 0;
      break;
    default:
      break;
    }

    if(t > 2) {
      t = 0.0;
      phase++;
    }

    baseToLLeg->calcInverseKinematics(posLLeg);
    baseToRLeg->calcInverseKinematics(posRLeg);

    std_msgs::Float64MultiArray arr;
    for(int i = 0; i < ioBody->numJoints(); ++i) {
      qref[i] = virtualBody->joint(i)->q();
      Link * joint = ioBody->joint(i);
      double q     = joint->q();
      double dq    = ( q - qold[i] ) / dt;
      joint->dq_target() = ( qref[i] - q ) * pgain + ( 0.0 - dq ) * dgain;
      qold[i]            = q;

      arr.data.push_back(q);
    }
    pubJoint.publish(arr);

    geometry_msgs::Pose pose;
    pose.position.x = ioBody->rootLink()->translation().x();
    pose.position.y = ioBody->rootLink()->translation().y();
    pose.position.z = ioBody->rootLink()->translation().z();
    Quaternion q(ioBody->rootLink()->rotation() );
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pubPose.publish(pose);

    data.data = t;
    pubSim.publish(data);

    t += dt;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PlotController)