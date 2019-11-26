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
  Capt::Grid          *grid;
  Capt::Capturability *capturability;
  Capt::GridMap       *gridmap;
  Capt::Search        *search;
  double               g, h, omega;

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
    param         = new Capt::Param("/home/dl-box/study/capturability/data/valkyrie_xy.xml");
    grid          = new Capt::Grid(param);
    capturability = new Capt::Capturability(grid);
    gridmap       = new Capt::GridMap(param);
    model->read(&g, "gravity");
    model->read(&h, "com_height");
    omega = sqrt(g / h);
    capturability->load("/home/dl-box/study/capturability/build/bin/gpu/Basin.csv", Capt::DataType::BASIN);
    capturability->load("/home/dl-box/study/capturability/build/bin/gpu/Nstep.csv", Capt::DataType::NSTEP);
    search = new Capt::Search(gridmap, grid, capturability);

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
      if( elapsed > 2 ) {
        Capt::vec2_t s_rfoot(footR.x(), footR.y() );
        Capt::vec2_t s_lfoot(footL.x(), footL.y() );
        Capt::vec2_t s_icp(icp.x(), icp.y() );
        Capt::vec2_t g_foot(1.0, 0.0);
        double       stance = 0.4;

        printf("footR %1.6lf, %1.6lf\n", footR.x(), footR.y() );
        printf("footL %1.6lf, %1.6lf\n", footL.x(), footL.y() );
        printf("ICP   %1.6lf, %1.6lf\n", icp.x(), icp.y() );

        search->setStanceWidth(stance);
        search->setStart(s_rfoot, s_lfoot, s_icp, Capt::Foot::FOOT_R);
        search->setGoal(g_foot);

        search->exe();

        footstepR = search->getFootstepR();
        footstepL = search->getFootstepL();
        footstepR.erase(footstepR.begin() ); // 現在の支持足位置と同じなので削除

        phase   = SSP_R;
        elapsed = 0.0;
      }
      break;
    case SSP_R:
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

    int size = 5;
    gridMap.clear();
    for(int i = 0; i < size; i++) {
      for(int j = 0; j < size; j++) {
        CaptData data_;
        data_.pos   = Vector3f(t + 0.05 * i, t + 0.05 * j, 0.0);
        data_.nstep = i;
        gridMap.push_back(data_);
      }
    }

    publisher.setPose(ioBody);
    // publisher.setComRef(comRef);
    // publisher.setCopRef(copRef);
    // publisher.setIcpRef(icpRef);
    // publisher.setCom(com);
    // publisher.setCop(cop);
    // publisher.setIcp(icp);
    publisher.setFootstepR(footstepR);
    publisher.setFootstepL(footstepL);
    // publisher.setGridMap(gridMap);
    // publisher.setFootLRef(footLRef);
    publisher.simulation(t);

    t       += dt;
    elapsed += dt;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CaptWalk)