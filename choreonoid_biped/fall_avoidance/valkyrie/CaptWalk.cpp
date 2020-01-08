#include <cnoid/BipedControl>
#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include "RvizPublisher.h"
#include "Capt.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;
using namespace cnoid;

const double pgain = 10;
const double dgain = 0.0;

namespace {

enum SufPhase {
  INIT, WAIT, DSP_LR, SSP_R, DSP_RL, SSP_L, STOP
};

}

class CaptWalk : public SimpleController
{
  BodyPtr             ioBody;
  double              t, dt, elapsed;
  std::vector<double> qref;
  std::vector<double> qold;
  SufPhase            phase;
  // SwfPhase            phase;

  // ros setting
  ros::NodeHandle nh;
  ros::Publisher  startPublisher, goalPublisher;
  ros::Subscriber joySubscriber, footstepSubscriber;

  // rviz
  RvizPublisher        publisher;
  std::vector<Vector3> footstepRef;

  // capturability
  Capt::Model         *model;
  Capt::Param         *param;
  Capt::Config        *config;
  Capt::Grid          *grid;
  Capt::Capturability *capturability;
  Capt::Monitor       *monitor;
  Capt::Planner       *planner;
  Capt::Trajectory    *trajectory;
  double               omega, h;

  // biped control
  IcpTracker *icpTracker;
  ComTracker *comTracker;
  Kinematics *kinematics;

  // parameters
  // global planner
  Capt::Footstep footstep;
  // local planner
  Vector3             comRef, copRef, icpRef;
  Vector3             com, comVel, comAcc, cop, icp;
  Vector3             footRRef, footLRef;
  Vector3             footR, footL;
  Vector3             force;
  Capt::EnhancedState state;
  Capt::EnhancedInput input;

public:
  void init(){
    cop       = Vector3::Zero();
    com       = Vector3::Zero();
    com.z()   = h;
    comVel    = Vector3::Zero();
    comAcc    = Vector3::Zero();
    icp       = Vector3::Zero();
    footR     = Vector3::Zero();
    footR.y() = -0.2;
    footL     = Vector3::Zero();
    footL.y() = +0.2;
    force     = Vector3::Zero();
  }

  void step(){
    // cop     = copMod;
    com    += comVel * dt;
    com.z() = h;
    comVel += comAcc * dt;
    // comVel     = ( icp - com ) * omega;
    comVel.z() = 0.0;
    comAcc     = omega * omega * ( com - cop ) + force / ioBody->mass();
    comAcc.z() = 0.0;
    icp        = com + comVel / omega;
    icp.z()    = 0.0;
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

  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy){
    // force.x() = 100 * ( joy->buttons[13] - joy->buttons[14] );
    // force.y() = 100 * ( joy->buttons[15] - joy->buttons[16] );
    force.x() = 100 * joy->axes[7];
    force.y() = 100 * joy->axes[6];
    force.z() = 0.0;
  }

  void footstepCallback( const nav_msgs::Path::ConstPtr &path){
    footstep.clear();
    footstepRef.clear();

    // current support foot
    Capt::Step step;
    step.suf = Capt::Foot::FOOT_R;
    step.pos = footR;
    footstep.push_back(step);
    footstepRef.push_back(step.pos);

    // footstep
    for(size_t i = 0; i < path->poses.size(); i++) {
      geometry_msgs::Pose pose = path->poses[i].pose;
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
    joySubscriber      = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &CaptWalk::joyCallback, this);
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
    param         = new Capt::Param("/home/dl-box/study/capturability/data/valkyrie_xy.xml");
    config        = new Capt::Config("/home/dl-box/study/capturability/data/valkyrie_config.xml");
    grid          = new Capt::Grid(param);
    capturability = new Capt::Capturability(grid);
    capturability->loadBasin("/home/dl-box/study/capturability/build/bin/cpu/Basin.csv");
    capturability->loadNstep("/home/dl-box/study/capturability/build/bin/cpu/Nstep.csv");
    monitor    = new Capt::Monitor(model, grid, capturability);
    planner    = new Capt::Planner(model, param, config, grid, capturability);
    trajectory = new Capt::Trajectory(model);

    // biped control
    icpTracker = new IcpTracker(model, config);

    model->read(&omega, "omega");
    model->read(&h, "com_height");

    phase   = INIT;
    t       = 0.0;
    elapsed = 0.0;

    publisher.setTimeStep(dt);

    return true;
  }

  virtual bool control() override
  {

    switch( phase ) {
    case INIT:
      init();
      if(t > 1.0) {
        startPublish(Vector3(0, 0, 0) );
        goalPublish(Vector3(2, 0, 0) );
        phase = WAIT;
      }
      break;
    case WAIT:
      if(t > 2.0) {
        phase = DSP_LR;
      }
      break;
    case DSP_LR:
      printf("DSP_LR\n");
      elapsed        = 0.0;
      state.footstep = footstep;
      state.icp      = icp;
      state.rfoot    = footR;
      state.lfoot    = footL;
      state.s_suf    = Capt::Foot::FOOT_R;
      state.elapsed  = 0.0;
      printf("icp      %1.3lf, %1.3lf, %1.3lf\n", icp.x(), icp.y(), icp.z() );
      printf("footR    %1.3lf, %1.3lf, %1.3lf\n", footR.x(), footR.y(), footR.z() );
      printf("footL    %1.3lf, %1.3lf, %1.3lf\n", footL.x(), footL.y(), footL.z() );
      printf("elapsed  %1.3lf\n", elapsed );

      if(monitor->check(state, footstep) ) {
        input = monitor->get();
        printf("safe\n");
      }else{
        // planner->set(state);
        // planner->plan();
        // input = planner->get();
        printf("unsafe\n");
      }

      trajectory->set(input, Capt::Foot::FOOT_R);
      copRef = trajectory->getCop(elapsed);
      icpRef = trajectory->getIcp(elapsed);

      phase   = SSP_R;
      elapsed = 0.0;
      break;
    case SSP_R:
      copRef = trajectory->getCop(elapsed);
      icpRef = trajectory->getIcp(elapsed);
      footR  = trajectory->getFootR(elapsed);
      footL  = trajectory->getFootL(elapsed);
      if(elapsed > input.duration) {
        phase = DSP_RL;
      }
      break;
    case DSP_RL:
      printf("DSP_RL\n");
      elapsed        = 0.0;
      state.footstep = footstep;
      state.icp      = icp;
      state.rfoot    = footR;
      state.lfoot    = footL;
      state.s_suf    = Capt::Foot::FOOT_L;
      state.elapsed  = 0.0;
      printf("icp      %1.3lf, %1.3lf, %1.3lf\n", icp.x(), icp.y(), icp.z() );
      printf("footR    %1.3lf, %1.3lf, %1.3lf\n", footR.x(), footR.y(), footR.z() );
      printf("footL    %1.3lf, %1.3lf, %1.3lf\n", footL.x(), footL.y(), footL.z() );
      printf("elapsed  %1.3lf\n", elapsed );

      if(monitor->check(state, footstep) ) {
        input = monitor->get();
        printf("safe\n");
      }else{
        // planner->set(state);
        // planner->plan();
        // input = planner->get();
        printf("unsafe\n");
      }

      trajectory->set(input, Capt::Foot::FOOT_L);
      copRef = trajectory->getCop(elapsed);
      icpRef = trajectory->getIcp(elapsed);

      elapsed = 0.0;
      phase   = SSP_L;
      break;
    case SSP_L:
      copRef = trajectory->getCop(elapsed);
      icpRef = trajectory->getIcp(elapsed);
      footR  = trajectory->getFootR(elapsed);
      footL  = trajectory->getFootL(elapsed);
      if(elapsed > input.duration) {
        phase = DSP_LR;
      }
      break;
    default:
      break;
    }

    icpTracker->set(copRef, icpRef, icp);
    cop = icpTracker->getCopMod();

    step();

    publisher.setFootstepRef(footstepRef);
    publisher.setFootstepR(planner->getFootstepR() );
    publisher.setFootstepL(planner->getFootstepL() );
    publisher.setFootRRef(footR);
    publisher.setFootLRef(footL);

    publisher.setCopRef(copRef);
    publisher.setCop(cop);
    publisher.setCom(com);
    publisher.setIcpRef(icpRef);
    publisher.setIcp(icp);
    publisher.setForce(force);
    publisher.simulation(t);

    // if( ( (int)( t * 1000 ) ) % ( 1000 )  == 0) {
    //   Vector3 center = ( footR + footL ) / 2;
    //   startPublish(Vector3(center.x(), center.y(), 0.0 ) );
    //   goalPublish(Vector3(2, 0, 0) );
    // }

    t       += dt;
    elapsed += dt;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CaptWalk)