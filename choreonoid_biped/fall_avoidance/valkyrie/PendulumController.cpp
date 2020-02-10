#include <cnoid/SimpleController>
#include "Capt.h"
#include "RvizPublisher.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;
using namespace cnoid;

class PendulumController : public SimpleController
{
  BodyPtr             ioBody;
  double              t, dt, elapsed;
  std::vector<double> qref;
  std::vector<double> qold;
  int                 phase;

  Capt::Cycloid cycloid;

  // rviz
  RvizPublisher publisher;

  // local planner
  Vector3 comRef, copRef, icpRef;
  Vector3 com, comVel, comAcc, cop, icp;
  Vector3 footR, footL;
  Vector3 force;

  double h, omega;
  double stepDuration;
  double stepWidth;

public:
  void init(){
    cop     = Vector3::Zero();
    com     = Vector3::Zero();
    com.z() = h;
    comVel  = Vector3::Zero();
    comAcc  = Vector3::Zero();
    icp     = Vector3::Zero();
    footR   = Vector3::Zero();
    footL   = Vector3::Zero();
    force   = Vector3::Zero();

    footR.y() -= 0.1;
    footL.y() += 0.1;

    cop.x()    = 0;
    com.x()    = 0;
    comVel.x() = 0.05;

    com.y() = -0.2;
    cop.y() = -0.2;

    copRef = cop;
    icpRef = icp;
  }

  void step(){
    com       += comVel * dt;
    com.z()    = h;
    comVel    += comAcc * dt;
    comVel.z() = 0.0;
    comAcc     = omega * omega * ( com - cop ) + force / ioBody->mass();
    comAcc.z() = 0.0;
    icp        = com + comVel / omega;
    icp.z()    = 0.0;
  }

  virtual bool initialize(SimpleControllerIO* io) override
  {
    init();

    ioBody = io->body();
    dt     = io->timeStep();

    t       = 0.0;
    elapsed = 0.0;
    phase   = 0;

    publisher.setTimeStep(dt);

    h     = 1.0;
    omega = sqrt(9.80665 / h);

    stepDuration = 1;
    // stepWidth    = 0.2;
    // stepWidth = 0.364;
    stepWidth = 0.5;

    Vector3 footR_ = footR;
    footR_.x() += stepWidth;
    cycloid.set(footR, footR_, stepDuration);

    return true;
  }

  virtual bool control() override
  {
    switch( phase ) {
    case 0:
      init();
      if(elapsed > 1.0) {
        elapsed = 0.0;
        phase   = 1;
      }
      break;
    case 1:
      footR = cycloid.get(elapsed);
      if(elapsed > 1.0) {
        cop.x() = footR.x();
        elapsed = 0.0;
        phase   = 2;
      }
      break;
    case 2:
      // cop.x() = icp.x();
      break;
    }

    if(phase > 0) {
      step();
    }

    publisher.setFootRRef(footR);
    publisher.setFootLRef(footL);

    publisher.setCop(cop);
    publisher.setCom(com);
    publisher.setIcp(icp);
    publisher.setForce(force);
    publisher.simulation(t);

    t       += dt;
    elapsed += dt;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PendulumController)