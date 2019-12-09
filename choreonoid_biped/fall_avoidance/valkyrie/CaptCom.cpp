#include "Capt.h"
#include "RvizPublisher.h"
#include <cnoid/BipedControl>
#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include <vector>
#include <iostream>

using namespace cnoid;

const double pgain = 10;
const double dgain = 0.0;

class ValkyrieCom : public SimpleController
{
  BodyPtr             ioBody;
  double              dt, t;
  std::vector<double> qref;
  std::vector<double> qold;

  Vector3 comRef, pelvisRef, footRRef, footLRef;
  Vector3 com, torso, pelvis, footR, footL;

  Kinematics kinematics;

  // rviz
  RvizPublisher publisher;

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
      io->setLinkInput(ioBody->link(i), LINK_POSITION);
    }

    t = 0.0;

    kinematics.setBody(ioBody);

    publisher.setTimeStep(dt);

    torso  = ioBody->link("torso")->position().translation();
    pelvis = ioBody->link("pelvis")->position().translation();
    footR  = ioBody->link("rightFootSole")->position().translation();
    footL  = ioBody->link("leftFootSole")->position().translation();

    return true;
  }

  virtual bool control() override
  {
    Vector3 rpy(-t * 0.1, 0, 0);
    kinematics.setTorso(rpy);
    rpy << t * 0.1, 0, 0;

    qref = kinematics.getJoints();
    for(int i = 0; i < ioBody->numJoints(); ++i) {
      Link  *joint = ioBody->joint(i);
      double q     = joint->q();
      double dq    = ( q - qold[i] ) / dt;
      joint->dq_target() = ( qref[i] - q ) * pgain + ( 0.0 - dq ) * dgain;
      qold[i]            = q;
    }

    if(t < 2) {
      pelvis.z() -= 0.0001;
    }else{
      footL.y() -= 0.0001;
    }
    // kinematics.setTorso(torso);
    kinematics.setPelvis(pelvis, rpy);
    kinematics.setFootR(footR);
    kinematics.setFootL(footL);
    kinematics.inverse();
    kinematics.forward();

    publisher.setPose(kinematics.getBody() );
    publisher.simulation(t);

    // show current link & com position
    // printf("Left Foot\n");
    // printf("\tx: %1.4lf\n", ioBody->link("leftFoot")->position().translation().x() );
    // printf("\ty: %1.4lf\n", ioBody->link("leftFoot")->position().translation().y() );
    // printf("\tz: %1.4lf\n", ioBody->link("leftFoot")->position().translation().z() );

    t += dt;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ValkyrieCom)