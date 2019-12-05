#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include "RvizPublisher.h"

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
  Vector3               comRef, copRef, icpRef;
  Vector3               com, cop, icp;
  Vector3               footRRef, footLRef;
  std::vector<Vector3>  footstepR, footstepL;
  std::vector<CaptData> gridMap;

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

    footRRef.x() = ioBody->link("rightFootSole")->position().translation().x();
    footRRef.y() = ioBody->link("rightFootSole")->position().translation().y();
    footRRef.z() = ioBody->link("rightFootSole")->position().translation().z();
    footLRef.x() = ioBody->link("leftFootSole")->position().translation().x();
    footLRef.y() = ioBody->link("leftFootSole")->position().translation().y();
    footLRef.z() = ioBody->link("leftFootSole")->position().translation().z();

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
      posLLeg.translation().y() -= 0.1 * dt;
      posRLeg.translation().y() -= 0.1 * dt;
      break;
    case 3:
      posRLeg.translation().z() += 0.1 * dt;
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

    comRef.x() = t;
    comRef.y() = 1;
    comRef.z() = 0;

    copRef.x() = t;
    copRef.y() = 0;
    copRef.z() = 0;

    icpRef.x() = t;
    icpRef.y() = -1;
    icpRef.z() = 0;

    com.x() = t / 2;
    com.y() = 1;
    com.z() = 0;

    cop.x() = t / 2;
    cop.y() = 0;
    cop.z() = 0;

    icp.x() = t / 2;
    icp.y() = -1;
    icp.z() = 0;

    if(t < 0.5) {
      footRRef.z() += 0.2 * dt;
    }else if(t < 1.5) {
      footRRef.x() += 0.4 * dt;
      footRRef.y() += 0.0 * dt;
    }else if(t < 2) {
      footRRef.z() -= 0.2 * dt;
    }else if(t < 2.5) {
      footLRef.z() += 0.2 * dt;
    }else if(t < 3.5) {
      footLRef.x() += 0.4 * dt;
      footLRef.y() += 0.0 * dt;
    }else if(t < 4) {
      footLRef.z() -= 0.2 * dt;
    }

    footstepR.clear();
    footstepR.push_back(Vector3(0.5 * t + 0.5, -0.5, 0.0) );
    footstepR.push_back(Vector3(0.5 * t + 1.0, -0.5, 0.0) );
    footstepR.push_back(Vector3(0.5 * t + 1.5, -0.5, 0.0) );
    footstepR.push_back(Vector3(0.5 * t + 2.0, -0.5, 0.0) );

    footstepL.clear();
    footstepL.push_back(Vector3(0.5 * t + 0.5, +0.5, 0.0) );
    footstepL.push_back(Vector3(0.5 * t + 1.0, +0.5, 0.0) );
    footstepL.push_back(Vector3(0.5 * t + 1.5, +0.5, 0.0) );
    footstepL.push_back(Vector3(0.5 * t + 2.0, +0.5, 0.0) );

    int size = 5;
    gridMap.clear();
    for(int i = 0; i < size; i++) {
      for(int j = 0; j < size; j++) {
        CaptData data_;
        data_.pos   = Vector3(0.1 * t + 0.05 * i, 0.1 * t + 0.05 * j, 0.0);
        data_.nstep = i;
        gridMap.push_back(data_);
      }
    }

    publisher.setPose(ioBody);
    publisher.setComRef(comRef);
    publisher.setCopRef(copRef);
    publisher.setIcpRef(icpRef);
    publisher.setFootRRef(footRRef);
    publisher.setFootLRef(footLRef);
    publisher.setCom(com);
    publisher.setCop(cop);
    publisher.setIcp(icp);
    publisher.setFootstepR(footstepR);
    publisher.setFootstepL(footstepL);
    publisher.setGridMap(gridMap);
    publisher.simulation(t);

    t += dt;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PlotController)