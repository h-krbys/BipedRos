#define ENABLE_JOY  true
#define FORCE_SCALE 300

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
  INIT, DSP, SSP, STOP, FAIL
};

}

class CaptStand : public SimpleController
{
  BodyPtr             ioBody;
  double              t, dt, elapsed;
  std::vector<double> qref;
  std::vector<double> qold;
  SufPhase            phase;

  // rviz
  RvizPublisher        publisher;
  std::vector<Vector3> footstepR, footstepL, footstepRef;

  // capturability
  Capt::Model         *model;
  Capt::Param         *param;
  Capt::Config        *config;
  Capt::Grid          *grid;
  Capt::Capturability *capturability;
  Capt::Generator     *generator;
  Capt::Monitor       *monitor;
  Capt::Planner       *planner;
  Capt::Trajectory    *trajectory;
  double               omega, h;
  Capt::Foot           supportFoot;

  // biped control
  IcpTracker *icpTracker;
  ComTracker *comTracker;
  Kinematics *kinematics;

  // capture region
  std::vector<CaptData> gridMap;

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

  // loop counter
  int count;

  Capt::Status statusMonitor, statusPlanner;

  // timer
  Capt::Timer *timer;

  FILE *fpTime;

public:
  void init(){
    footR << 0, -0.2, 0;
    footL << 0, +0.2, 0;

    cop = footR;
    com = footR;
    icp = footR;

    comVel.x() = -0.0;
    comVel.y() = +0.5;

    copRef = cop;
    icpRef = icp;
  }

  void step(){
    // cop     = copMod;
    com       += comVel * dt;
    com.z()    = h;
    comVel    += comAcc * dt;
    comVel.z() = 0.0;
    comAcc     = omega * omega * ( com - cop ) + force / ioBody->mass();
    comAcc.z() = 0.0;
    icp        = com + comVel / omega;
    icp.z()    = 0.0;
  }

  void substitute(std::vector<Capt::CaptData> from, std::vector<CaptData> *to){
    to->clear();
    for(size_t i = 0; i < from.size(); i++) {
      CaptData data;
      data.pos   = from[i].pos;
      data.nstep = from[i].nstep;
      to->push_back(data);
    }
  }

  void footstepCallback( const nav_msgs::Path::ConstPtr &path){
    footstep.clear();
    footstepRef.clear();

    Capt::Step step;

    // initial stance foot
    step.suf = Capt::Foot::FOOT_L;
    step.pos = footL;
    footstep.push_back(step);
    footstepRef.push_back(step.pos);

    step.suf = Capt::Foot::FOOT_R;
    step.pos = footR;
    footstep.push_back(step);
    footstepRef.push_back(step.pos);

    // footstep for walk
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

    // footstep for stamp
    // for(size_t i = 0; i < path->poses.size(); i++) {
    //   if( ( (int)i % 2 ) == 0) {
    //     step.suf     = Capt::Foot::FOOT_L;
    //     step.pos.y() = +0.2;
    //   }else{
    //     step.suf     = Capt::Foot::FOOT_R;
    //     step.pos.y() = -0.2;
    //   }
    //   step.pos.x() = 0;
    //   step.pos.z() = 0;
    //
    //   footstep.push_back(step);
    //   footstepRef.push_back(step.pos);
    // }

    generator->calc(&footstep);
    // footstep.pop_back();
    // footstepRef.pop_back();
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
    if(!model) {
      model         = new Capt::Model("/home/kuribayashi/Capturability/cartesian/data/valkyrie.xml");
      param         = new Capt::Param("/home/kuribayashi/Capturability/cartesian/data/valkyrie_xy.xml");
      config        = new Capt::Config("/home/kuribayashi/Capturability/cartesian/data/valkyrie_config.xml");
      grid          = new Capt::Grid(param);
      capturability = new Capt::Capturability(grid);
      capturability->loadBasin("/home/kuribayashi/Capturability/cartesian/build/bin/cpu/Basin.csv");
      capturability->loadNstep("/home/kuribayashi/Capturability/cartesian/build/bin/cpu/1step.csv", 1);
      capturability->loadNstep("/home/kuribayashi/Capturability/cartesian/build/bin/cpu/2step.csv", 2);
      capturability->loadNstep("/home/kuribayashi/Capturability/cartesian/build/bin/cpu/3step.csv", 3);
      generator  = new Capt::Generator(model, param);
      monitor    = new Capt::Monitor(model, param, grid, capturability);
      planner    = new Capt::Planner(model, param, config, grid, capturability);
      trajectory = new Capt::Trajectory(model, param);
    }

    // biped control
    icpTracker = new IcpTracker(model, config);

    // timer
    timer  = new Capt::Timer();
    fpTime = fopen("/home/kuribayashi/computation.csv", "w");
    fprintf(fpTime, "time, computation\n");

    model->read(&omega, "omega");
    model->read(&h, "com_height");

    phase   = INIT;
    t       = 0.0;
    elapsed = 0.0;

    publisher.initialize();
    publisher.setTimeStep(dt);

    footstepR.clear();
    footstepL.clear();
    planner->clear();
    gridMap.clear();
    supportFoot = Capt::Foot::FOOT_NONE;

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

    return true;
  }

  virtual bool control() override
  {

    switch( phase ) {
    case INIT:
      init();
      phase = DSP;
      break;
    case DSP:
      // support foot exchange
      if(supportFoot == Capt::Foot::FOOT_R) {
        supportFoot = Capt::Foot::FOOT_L;
        printf("------ DSP (RL) ------\n");
      }else{
        supportFoot = Capt::Foot::FOOT_R;
        printf("------ DSP (LR) ------\n");
      }

      elapsed        = 0.0;
      state.footstep = footstep;
      state.icp      = icp;
      state.rfoot    = footR;
      state.lfoot    = footL;
      state.s_suf    = supportFoot;
      // printf("icp      %1.3lf, %1.3lf, %1.3lf\n", icp.x(), icp.y(), icp.z() );
      // printf("footR    %1.3lf, %1.3lf, %1.3lf\n", footR.x(), footR.y(), footR.z() );
      // printf("footL    %1.3lf, %1.3lf, %1.3lf\n", footL.x(), footL.y(), footL.z() );
      // printf("elapsed  %1.3lf\n", elapsed );

      // planner->set(state);
      // if(planner->plan() ) {
      //   input = planner->get();
      // }else{
      //   phase = STOP;
      //   break;
      // }

      timer->start();
      // monitor
      statusMonitor = monitor->check(state, footstep);
      if(statusMonitor == Capt::Status::SUCCESS) {
        planner->clear();
        input = monitor->get();
        substitute(monitor->getCaptureRegion(), &gridMap);
        footstepR = monitor->getFootstepR();
        footstepL = monitor->getFootstepL();
        printf("monitor: success\n");
      }else if(statusMonitor == Capt::Status::FAIL) {
        // printf("monitor: fail\n");
      }else{
        printf("monitor: finish\n");
        phase = STOP;
        break;
      }

      // planner
      if(statusMonitor == Capt::Status::FAIL) {
        planner->set(state);
        statusPlanner = planner->plan();
        if(statusPlanner == Capt::Status::SUCCESS) {
          printf("planner: success\n");
          input = planner->get();
          substitute(planner->getCaptureRegion(), &gridMap);
          footstepR = planner->getFootstepR();
          footstepL = planner->getFootstepL();
        }else if(statusPlanner == Capt::Status::FAIL) {
          printf("planner: fail\n");
          phase = FAIL;
          break;
        }else{
          printf("planner: finish\n");
          phase = STOP;
          break;
        }
      }
      timer->end();
      timer->print();
      fprintf(fpTime, "%lf, %lf\n", t, timer->get() );

      trajectory->set(input, supportFoot);
      copRef = trajectory->getCop(elapsed);
      icpRef = trajectory->getIcp(elapsed);

      phase   = SSP;
      elapsed = 0.0;
      count   = 0;

      break;
    case SSP:
      count++;
      if(count % 10 == 0 && input.duration - elapsed > 0.10) {
        // support foot
        if(supportFoot == Capt::Foot::FOOT_R) {
          printf("------ SSP (R) ------\n");
        }else{
          printf("------ SSP (L) ------\n");
        }

        state.icp   = icp;
        state.rfoot = footR;
        state.lfoot = footL;
        state.s_suf = supportFoot;

        timer->start();
        statusMonitor = monitor->check(state, footstep);
        if(statusMonitor == Capt::Status::SUCCESS) {
          planner->clear();
          input = monitor->get();
          substitute(monitor->getCaptureRegion(), &gridMap);
          footstepR = monitor->getFootstepR();
          footstepL = monitor->getFootstepL();
          printf("monitor: success\n");
        }else if(statusMonitor == Capt::Status::FAIL) {
          // printf("monitor: fail\n");
        }else{
          printf("monitor: finish\n");
          phase = STOP;
          break;
        }

        // planner
        if(statusMonitor == Capt::Status::FAIL) {
          planner->set(state);
          statusPlanner = planner->plan();
          if(statusPlanner == Capt::Status::SUCCESS) {
            input = planner->get();
            substitute(planner->getCaptureRegion(), &gridMap);
            footstepR = planner->getFootstepR();
            footstepL = planner->getFootstepL();
            printf("planner: success\n");
          }else if(statusPlanner == Capt::Status::FAIL) {
            printf("planner: fail\n");
            phase = FAIL;
            break;
          }else{
            printf("planner: finish\n");
            phase = STOP;
            break;
          }
        }
        timer->end();
        timer->print();
        fprintf(fpTime, "%lf, %lf\n", t, timer->get() );

        trajectory->set(input, supportFoot);
      }

      copRef = trajectory->getCop(elapsed);
      icpRef = trajectory->getIcp(elapsed);
      footR  = trajectory->getFootR(elapsed);
      footL  = trajectory->getFootL(elapsed);
      if(elapsed > input.duration) {
        phase = DSP;
        // phase = STOP;
      }
      break;
    case STOP:
      supportFoot = Capt::Foot::FOOT_NONE;
      footstepR.clear();
      footstepL.clear();
      planner->clear();
      gridMap.clear();
      if(fpTime) {
        fclose(fpTime);
        fpTime = NULL;
      }
      copRef = icp;
      break;
    case FAIL:
      footstepR.clear();
      footstepL.clear();
      planner->clear();
      gridMap.clear();
      if(fpTime) {
        fclose(fpTime);
        fpTime = NULL;
      }
      copRef = icp;
      break;
    default:
      break;
    }

    icpTracker->set(copRef, icpRef, icp);
    cop = icpTracker->getCopMod();
    if(supportFoot == Capt::Foot::FOOT_R) {
      if(cop.x() > 0.125 + footR.x() ) {
        cop.x() = 0.125 + footR.x();
      }
      if(cop.x() < -0.125 + footR.x() ) {
        cop.x() = -0.125 + footR.x();
      }
      if(cop.y() > 0.075 + footR.y() ) {
        cop.y() = 0.075 + footR.y();
      }
      if(cop.y() < -0.075 + footR.y() ) {
        cop.y() = -0.075 + footR.y();
      }
    }
    if(supportFoot == Capt::Foot::FOOT_L) {
      if(cop.x() > 0.125 + footL.x() ) {
        cop.x() = 0.125 + footL.x();
      }
      if(cop.x() < -0.125 + footL.x() ) {
        cop.x() = -0.125 + footL.x();
      }
      if(cop.y() > 0.075 + footL.y() ) {
        cop.y() = 0.075 + footL.y();
      }
      if(cop.y() < -0.075 + footL.y() ) {
        cop.y() = -0.075 + footL.y();
      }
    }

    if(!ENABLE_JOY) {
      // if(5.5 <= t && t <= 5.5 + duration) {
      //   // simulation 1
      //   // force.x() = -2000;
      //   // simulation 2
      //   // force.y() = 5000;
      // }else{
      //   force.x() = 0;
      //   force.y() = 0;
      // }
      if(5.5 <= t && t <= 6) {
        // simulation 3
        force.x() = -300 * sin( ( t - 5.5 ) * 3.14159 / 0.5);
        force.y() = +200 * sin( ( t - 5.5 ) * 3.14159 / 0.5);
      }else{
        force.x() = 0;
        force.y() = 0;
      }
    }

    // simulation step
    switch( phase ) {
    case INIT:
      break;
    case DSP:
    case SSP:
    case STOP:
    case FAIL:
      step();
      break;
    }

    publisher.setFootstepRef(footstepRef);
    publisher.setFootstepR(footstepR);
    publisher.setFootstepL(footstepL);
    publisher.setFootRRef(footR);
    publisher.setFootLRef(footL);

    publisher.setCopRef(copRef);
    publisher.setCop(cop);
    publisher.setCom(com);
    publisher.setIcpRef(icpRef);
    publisher.setIcp(icp);
    publisher.setForce(force);
    publisher.setGridMap(gridMap);
    publisher.simulation(t);

    // global replanning
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

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CaptStand)