#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include "RvizPublisher.h"
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

  // rviz
  RvizPublisher publisher;

  // control
  Vector3f cop;

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

    t = 0.0;

    publisher.setTimeStep(dt);

    return true;
  }

  virtual bool control() override
  {
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
    default:
      break;
    }

    if(t > 2 * ( phase + 1 ) ) {
      phase++;
    }

    baseToLLeg->calcInverseKinematics(posLLeg);
    baseToRLeg->calcInverseKinematics(posRLeg);

    for(int i = 0; i < ioBody->numJoints(); ++i) {
      qref[i] = virtualBody->joint(i)->q();
      Link * joint = ioBody->joint(i);
      double q     = joint->q();
      double dq    = ( q - qold[i] ) / dt;
      joint->dq_target() = ( qref[i] - q ) * pgain + ( 0.0 - dq ) * dgain;
      qold[i]            = q;
    }

    t += dt;

    cop.x() = t;
    cop.y() = 0;
    cop.z() = 0;

    publisher.setPose(ioBody);
    publisher.setCop(cop);
    publisher.simulation(t);

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PlotController)