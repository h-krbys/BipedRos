#include <cnoid/BipedControl>
#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include "RvizPublisher.h"
#include "Capt.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
  ros::Publisher  startPublisher, goalPublisher;
  ros::Subscriber footstepSubscriber;

  // rviz
  RvizPublisher        publisher;
  std::vector<Vector3> footstepRef;

  // capturability
  Capt::Model         *model;
  Capt::Param         *param;
  Capt::Config        *config;
  Capt::Grid          *grid;
  Capt::Capturability *capturability;
  double               omega;

  // biped control
  ComTracker *comTracker;
  Kinematics *kinematics;

  // parameters
  // global planner
  Capt::Footstep footstep;
  // local planner
  Vector3 comRef, copRef, icpRef;
  Vector3 com, com_, comVel, cop, icp;
  Vector3 footRRef, footLRef;
  Vector3 footR, footL;

public:
  void sync(){
    com     = ioBody->calcCenterOfMass();
    comVel  = ( com - com_ ) / dt;
    com_    = com;
    icp     = com + comVel / omega;
    icp.z() = 0.0;
    footR   = ioBody->link("rightFootSole")->position().translation();
    footL   = ioBody->link("leftFootSole")->position().translation();
  }

  void startPublish(Vector3 pos){
    geometry_msgs::PoseWithCovarianceStamped startPose;
    startPose.header.frame_id         = "map";
    startPose.header.stamp            = ros::Time::now();
    startPose.pose.pose.position.x    = pos.x();
    startPose.pose.pose.position.y    = pos.y();
    startPose.pose.pose.position.z    = pos.z();
    startPose.pose.pose.orientation.x = 0;
    startPose.pose.pose.orientation.y = 0;
    startPose.pose.pose.orientation.z = 0;
    startPose.pose.pose.orientation.w = 1;
    startPublisher.publish(startPose);
  }

  void goalPublish(Vector3 pos){
    geometry_msgs::PoseStamped goalPose;
    goalPose.header.frame_id    = "map";
    goalPose.header.stamp       = ros::Time::now();
    goalPose.pose.position.x    = pos.x();
    goalPose.pose.position.y    = pos.y();
    goalPose.pose.position.z    = pos.z();
    goalPose.pose.orientation.x = 0;
    goalPose.pose.orientation.y = 0;
    goalPose.pose.orientation.z = 0;
    goalPose.pose.orientation.w = 1;
    goalPublisher.publish(goalPose);
  }

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
    startPublisher     = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    goalPublisher      = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);
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

    // set capturability parameters
    model         = new Capt::Model("/home/dl-box/study/capturability/data/valkyrie.xml");
    param         = new Capt::Param("/home/dl-box/study/capturability/data/footstep.xml");
    config        = new Capt::Config("/home/dl-box/study/capturability/data/valkyrie_config.xml");
    grid          = new Capt::Grid(param);
    capturability = new Capt::Capturability(grid);
    // capturability->load("/home/dl-box/study/capturability/build/bin/gpu/Basin.csv", Capt::DataType::BASIN);
    // capturability->load("/home/dl-box/study/capturability/build/bin/gpu/Nstep.csv", Capt::DataType::NSTEP);

    model->read(&omega, "omega");

    comTracker = new ComTracker(config);
    comTracker->setBody(ioBody);
    kinematics = new Kinematics();
    kinematics->setBody(ioBody);

    phase   = INIT;
    t       = 0.0;
    elapsed = 0.0;

    publisher.setTimeStep(dt);

    return true;
  }

  virtual bool control() override
  {
    sync();

    switch( phase ) {
    case INIT:
      comRef     = com;
      comRef.z() = 1.0;
      comTracker->set(comRef, footR, footL);

      // posLLeg.translation().z() += 0.1 * dt;
      // posRLeg.translation().z() += 0.1 * dt;
      // if( com.z() <= 1.0 ) {
      //   phase   = WAIT;
      //   elapsed = 0.0;
      // }

      // if(t > 1.0) {
      //   startPublish(Vector3(0, 0, 0) );
      //   goalPublish(Vector3(2, 0, 0) );
      //   phase = WAIT;
      // }
      break;
    case WAIT:
      break;
    }

    publisher.setPose(comTracker->getBody() );
    publisher.setFootstepRef(footstepRef);
    publisher.simulation(t);

    t       += dt;
    elapsed += dt;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CaptWalk)