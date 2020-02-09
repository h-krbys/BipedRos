#include <cnoid/SimpleController>
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

  // rviz
  RvizPublisher publisher;

  // local planner
  Vector3 comRef, copRef, icpRef;
  Vector3 com, comVel, comAcc, cop, icp;
  Vector3 footR, footL;
  Vector3 force;

  double h, omega;

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

    cop.x()    = 0;
    com.x()    = 0;
    comVel.x() = 0.1;

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
    ioBody = io->body();
    dt     = io->timeStep();

    t     = 0.0;
    phase = 0;

    publisher.setTimeStep(dt);

    h     = 1.0;
    omega = sqrt(9.80665 / h);

    init();

    return true;
  }

  virtual bool control() override
  {

    switch( phase ) {
    case 0:
      if(t > 1.0) {
        phase = 1;
      }
      break;
    case 1:
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

    t += dt;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PendulumController)