#include <cnoid/BipedControl>
#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include "RvizPublisher.h"
#include "Capt.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace cnoid;

const double pgain = 10;
const double dgain = 0.0;

namespace {
enum Phase {
  INIT, WAIT, SSP_R, SSP_L, STOP
};
}

class CaptWalk : public SimpleController
{
  BodyPtr             ioBody;
  double              t, dt, elapsed;
  std::vector<double> qref;
  std::vector<double> qold;
  Phase               phase;

  // ros setting
  ros::NodeHandle nh;
  ros::Subscriber footstepSubscriber;

  // rviz
  RvizPublisher        publisher;
  std::vector<Vector3> footstepRef;

  // parameters
  // global planner
  Capt::Footstep footstep;

public:
  void footstepCallback( const nav_msgs::Path::ConstPtr &path){
    footstep.clear();
    footstepRef.clear();
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
      footstepRef.push_back(step.pos);
    }
  }

  virtual bool start() override
  {
    footstepSubscriber = nh.subscribe<nav_msgs::Path>("/footstep_planner/path", 10, &CaptWalk::footstepCallback, this);
    return true;
  }

  virtual bool initialize(SimpleControllerIO* io) override
  {
    ioBody = io->body();
    dt     = io->timeStep();

    // set input/output
    // set output to robot
    for(int i = 0; i < ioBody->numJoints(); ++i) {
      Link* joint = ioBody->joint(i);
      joint->setActuationMode(Link::JOINT_VELOCITY);
      io->enableIO(joint);
      qref.push_back(joint->q() );
    }
    qold = qref;
    // set input from robot
    for (int i = 0; i < ioBody->numLinks(); ++i) {
      io->enableInput(io->body()->link(i), LINK_POSITION);
    }

    t       = 0.0;
    elapsed = 0.0;

    publisher.setTimeStep(dt);

    return true;
  }

  virtual bool control() override
  {
    publisher.setPose(ioBody);
    publisher.setFootstepRef(footstepRef);
    publisher.simulation(t);

    t       += dt;
    elapsed += dt;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CaptWalk)