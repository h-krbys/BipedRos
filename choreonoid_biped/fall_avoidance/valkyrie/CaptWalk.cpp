#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include "RvizPublisher.h"
#include "Capt.h"

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

  // virtual body
  BodyPtr                    virtualBody;
  Link                      *virtualBase, *virtualLLeg, *virtualRLeg;
  std::shared_ptr<JointPath> baseToLLeg, baseToRLeg;

  // rviz
  RvizPublisher publisher;

  // capturability parameters
  Capt::Model         *model;
  Capt::Param         *param;
  Capt::Config        *config;
  Capt::Grid          *grid;
  Capt::Capturability *capturability;
  Capt::Planner       *planner;
  double               omega;
  planner::Input       input;
  planner::Output      output;

  // control
  Vector3f              comRef, copRef, icpRef;
  Vector3f              com, com_, comVel, cop, icp;
  Vector3f              footRRef, footLRef;
  Vector3f              footR, footL;
  std::vector<Vector3f> footstepR, footstepL;
  std::vector<CaptData> gridMap;

public:
  void substitute(Vector3 from, Vector3f *to){
    to->x() = from.x();
    to->y() = from.y();
    to->z() = from.z();
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

  void sync(){
    substitute(ioBody->calcCenterOfMass(), &com);
    comVel = ( com - com_ ) / dt;
    com_   = com;
    icp    = com + comVel / omega;
    substitute(ioBody->link("rightFootSole")->position().translation(), &footR);
    substitute(ioBody->link("leftFootSole")->position().translation(),  &footL);
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

    // set capturability parameters
    model         = new Capt::Model("/home/dl-box/study/capturability/data/valkyrie.xml");
    param         = new Capt::Param("/home/dl-box/study/capturability/data/footstep.xml");
    config        = new Capt::Config("/home/dl-box/study/capturability/data/valkyrie_config.xml");
    grid          = new Capt::Grid(param);
    capturability = new Capt::Capturability(grid);
    capturability->load("/home/dl-box/study/capturability/build/bin/gpu/Basin.csv", Capt::DataType::BASIN);
    capturability->load("/home/dl-box/study/capturability/build/bin/gpu/Nstep.csv", Capt::DataType::NSTEP);
    planner = new Capt::Planner(model, param, config, grid, capturability);

    model->read(&omega, "omega");

    phase   = INIT;
    t       = 0.0;
    elapsed = 0.0;

    publisher.setTimeStep(dt);

    return true;
  }

  virtual bool control() override
  {
    // get current values from robot
    sync();

    Position posLLeg = baseToLLeg->endLink()->position();
    Position posRLeg = baseToRLeg->endLink()->position();

    switch( phase ) {
    case INIT:
      posLLeg.translation().z() += 0.1 * dt;
      posRLeg.translation().z() += 0.1 * dt;
      if( elapsed > 2 ) {
        phase   = WAIT;
        elapsed = 0.0;
      }
      break;
    case WAIT:
      if( elapsed > 1 ) {
        elapsed = 0.0;

        input.elapsed = elapsed;
        input.suf     = Capt::Foot::FOOT_R;
        input.rfoot   = footR;
        input.lfoot   = footL;
        input.icp     = icp;
        input.goal    = Capt::vec3_t(1.0, 0.0, 0.0);
        input.stance  = 0.4;

        planner->set(input);

        footstepR = planner->getFootstepR();
        footstepL = planner->getFootstepL();

        substitute(planner->getCaptureRegion(), &gridMap);

        phase = SSP_R;
      }
      break;
    case SSP_R:
      output = planner->get(elapsed);
      if(elapsed < output.duration) {
        footRRef = output.rfoot;
        footLRef = output.lfoot;
        icpRef   = output.icp;
        copRef   = output.cop;
      }else{
        elapsed = 0.0;

        input.elapsed = elapsed;
        input.suf     = Capt::Foot::FOOT_L;
        input.rfoot   = footRRef;
        input.lfoot   = footLRef;
        input.icp     = icpRef;
        input.goal    = Capt::vec3_t(1.0, 0.0, 0.0);
        input.stance  = 0.4;

        planner->set(input);

        footstepR = planner->getFootstepR();
        footstepL = planner->getFootstepL();

        substitute(planner->getCaptureRegion(), &gridMap);

        phase = SSP_L;
      }
      break;
    case SSP_L:
      output = planner->get(elapsed);
      if(elapsed < output.duration) {
        footRRef = output.rfoot;
        footLRef = output.lfoot;
        icpRef   = output.icp;
        copRef   = output.cop;
      }else{
        elapsed = 0.0;

        input.elapsed = elapsed;
        input.suf     = Capt::Foot::FOOT_R;
        input.rfoot   = footRRef;
        input.lfoot   = footLRef;
        input.icp     = icpRef;
        input.goal    = Capt::vec3_t(1.0, 0.0, 0.0);
        input.stance  = 0.4;

        planner->set(input);

        footstepR = planner->getFootstepR();
        footstepL = planner->getFootstepL();

        substitute(planner->getCaptureRegion(), &gridMap);

        phase = SSP_R;
      }
      break;
    default:
      break;
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

    publisher.setPose(ioBody);
    // publisher.setComRef(comRef);
    publisher.setCopRef(copRef);
    publisher.setIcpRef(icpRef);
    publisher.setFootRRef(footRRef);
    publisher.setFootLRef(footLRef);
    // publisher.setCom(com);
    // publisher.setCop(cop);
    // publisher.setIcp(icp);
    publisher.setFootstepR(footstepR);
    publisher.setFootstepL(footstepL);
    publisher.setGridMap(gridMap);
    // publisher.setFootLRef(footLRef);
    publisher.simulation(t);

    t       += dt;
    elapsed += dt;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CaptWalk)