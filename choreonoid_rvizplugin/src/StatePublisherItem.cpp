#include "StatePublisherItem.h"

using namespace std;
using namespace cnoid;

void StatePublisherItem::initialize(ExtensionManager* ext){
  ext->itemManager().registerClass<StatePublisherItem>("StatePublisherItem");
  ext->itemManager().addCreationPanel<StatePublisherItem>();
}

StatePublisherItem::StatePublisherItem(){
  int    argc = 0;
  char** argv = 0;
  ros::init(argc, argv, "state_publisher");
  nh = new ros::NodeHandle("");

  pub_cop  = nh->advertise<biped_msgs::RvPoint>("rviz_plot/point", 1);
  pub_line = nh->advertise<biped_msgs::RvLine>("rviz_plot/line", 1);
}

StatePublisherItem::~StatePublisherItem(){
}

void StatePublisherItem::setTimeStep(double dt){
  this->dt = dt;
}

void StatePublisherItem::setBody(const BodyPtr ioBody){
}

void StatePublisherItem::setCop(Vector3f cop){
  this->cop = cop;
}

void StatePublisherItem::setCom(Vector3f com){
  this->com = com;
}

void StatePublisherItem::setIcp(Vector3f icp){
  this->icp = icp;
}

void StatePublisherItem::setFootR(Vector3f pos){
  this->foot_r = pos;
}

void StatePublisherItem::setFootL(Vector3f pos){
  this->foot_l = pos;
}

void StatePublisherItem::setFootstepR(std::vector<Vector3f> pos){
  this->footstep_r = pos;
}

void StatePublisherItem::setFootstepL(std::vector<Vector3f> pos){
  this->footstep_l = pos;
}

void StatePublisherItem::enableCopTraj(bool flag){
  this->copTraj = flag;
}

void StatePublisherItem::enableComTraj(bool flag){
  this->comTraj = flag;
}

void StatePublisherItem::enableIcpTraj(bool flag){
  this->icpTraj = flag;
}

void StatePublisherItem::enableFootTraj(bool flag){
  this->footTraj = flag;
}

void StatePublisherItem::update(){
}

void StatePublisherItem::playback(double t){
}