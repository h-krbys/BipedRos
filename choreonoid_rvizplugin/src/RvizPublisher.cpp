#include "RvizPublisher.h"

using namespace std;
using namespace Eigen;

RvizPublisher::RvizPublisher(){
  // subscriber
  subTimeStep = nh.subscribe<std_msgs::Float64>(
    "/simulation/timestep", 1000, &RvizPublisher::callbackTimeStep, this);
  subSimulation = nh.subscribe<std_msgs::Float64>(
    "/simulation/time", 1000, &RvizPublisher::callbackSimulation, this);
  // subPlayback = nh.subscribe<std_msgs::Float64>(
  //   "/playback/time", 10, &RvizPublisher::callbackPlayback, this);
  subPose = nh.subscribe<geometry_msgs::Pose>(
    "/simulation/pose", 1000, &RvizPublisher::callbackPose, this);
  subJoint = nh.subscribe<std_msgs::Float64MultiArray>(
    "/simulation/joint", 1000, &RvizPublisher::callbackJoint, this);
  subCop = nh.subscribe<geometry_msgs::Point>(
    "/simulation/cop", 1000, &RvizPublisher::callbackCop, this);

  // publisher
  pubJoint = nh.advertise<sensor_msgs::JointState>("/marker/joint", 1);
  pubPose  = nh.advertise<visualization_msgs::Marker>("/marker/body", 1);
  pubCop   = nh.advertise<visualization_msgs::Marker>("/marker/cop", 1);

  // initialize
  dt = 0.001;
  js.position.resize(43);
  js.name.resize(43);
  js.name[0]  = "hokuyo_joint";
  js.name[1]  = "torsoYaw";
  js.name[2]  = "torsoPitch";
  js.name[3]  = "torsoRoll";
  js.name[4]  = "lowerNeckPitch";
  js.name[5]  = "neckYaw";
  js.name[6]  = "upperNeckPitch";
  js.name[7]  = "rightShoulderPitch";
  js.name[8]  = "rightShoulderRoll";
  js.name[9]  = "rightShoulderYaw";
  js.name[10] = "rightElbowPitch";
  js.name[11] = "rightForearmYaw";
  js.name[12] = "rightWristRoll";
  js.name[13] = "rightWristPitch";
  js.name[14] = "rightThumbRoll";
  js.name[15] = "rightThumbPitch1";
  js.name[16] = "rightIndexFingerPitch1";
  js.name[17] = "rightMiddleFingerPitch1";
  js.name[18] = "rightPinkyPitch1";
  js.name[19] = "leftShoulderPitch";
  js.name[20] = "leftShoulderRoll";
  js.name[21] = "leftShoulderYaw";
  js.name[22] = "leftElbowPitch";
  js.name[23] = "leftForearmYaw";
  js.name[24] = "leftWristRoll";
  js.name[25] = "leftWristPitch";
  js.name[26] = "leftThumbRoll";
  js.name[27] = "leftThumbPitch1";
  js.name[28] = "leftIndexFingerPitch1";
  js.name[29] = "leftMiddleFingerPitch1";
  js.name[30] = "leftPinkyPitch1";
  js.name[31] = "rightHipYaw";
  js.name[32] = "rightHipRoll";
  js.name[33] = "rightHipPitch";
  js.name[34] = "rightKneePitch";
  js.name[35] = "rightAnklePitch";
  js.name[36] = "rightAnkleRoll";
  js.name[37] = "leftHipYaw";
  js.name[38] = "leftHipRoll";
  js.name[39] = "leftHipPitch";
  js.name[40] = "leftKneePitch";
  js.name[41] = "leftAnklePitch";
  js.name[42] = "leftAnkleRoll";

  for(int i = 0; i < 43; i++) {
    joint[i] = 0.0;
  }
}

RvizPublisher::~RvizPublisher(){
}

void RvizPublisher::callbackTimeStep(const std_msgs::Float64::ConstPtr &dt){
  this->dt = dt->data;
}

void RvizPublisher::callbackSimulation(const std_msgs::Float64::ConstPtr &t){
  PlotData data_;
  for(int i = 0; i < 43; i++) {
    data_.joint[i] = joint[i];
  }
  data_.cop    = cop;
  data_.com    = com;
  data_.icp    = icp;
  data_.foot_r = foot_r;
  data_.foot_l = foot_l;
  printf("time %lf\n", (double)t->data);
  data.push_back(data_);

  publishCop();
  publishJoint();
}

void RvizPublisher::callbackPose(const geometry_msgs::Pose::ConstPtr &body){
  tf::Vector3 p(body->position.x,
                body->position.y,
                body->position.z);
  tf::Quaternion q(body->orientation.x,
                   body->orientation.y,
                   body->orientation.z,
                   body->orientation.w);
  tf::Transform transform;
  transform.setOrigin(p);
  transform.setRotation(q);

  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "pelvis") );
}

void RvizPublisher::callbackJoint(const std_msgs::Float64MultiArray::ConstPtr &joint){
  this->joint[0]  =  joint->data[18]; // hokuyo_joint
  this->joint[1]  =  joint->data[12]; // torsoYaw
  this->joint[2]  =  joint->data[13]; // torsoPitch
  this->joint[3]  =  joint->data[14]; // torsoRoll
  this->joint[4]  =  joint->data[15]; // lowerNeckPitch
  this->joint[5]  =  joint->data[16]; // neckYaw
  this->joint[6]  =  joint->data[17]; // upperNeckPitch
  this->joint[7]  =  joint->data[39]; // rightShoulderPitch
  this->joint[8]  =  joint->data[40]; // rightShoulderRoll
  this->joint[9]  =  joint->data[41]; // rightShoulderYaw
  this->joint[10] =  joint->data[42]; // rightElbowPitch
  this->joint[11] =  joint->data[43]; // rightForearmYaw
  this->joint[12] =  joint->data[44]; // rightWristRoll
  this->joint[13] =  joint->data[45]; // rightWristPitch
  this->joint[14] =  joint->data[55]; // rightThumbRoll
  this->joint[15] =  joint->data[56]; // rightThumbPitch1
  this->joint[16] =  joint->data[46]; // rightIndexFingerPitch1
  this->joint[17] =  joint->data[49]; // rightMiddleFingerPitch1
  this->joint[18] =  joint->data[52]; // rightPinkyPitch1
  this->joint[19] =  joint->data[19]; // leftShoulderPitch
  this->joint[20] =  joint->data[20]; // leftShoulderRoll
  this->joint[21] =  joint->data[21]; // leftShoulderYaw
  this->joint[22] =  joint->data[22]; // leftElbowPitch
  this->joint[23] =  joint->data[23]; // leftForearmYaw
  this->joint[24] =  joint->data[24]; // leftWristRoll
  this->joint[25] =  joint->data[25]; // leftWristPitch
  this->joint[26] =  joint->data[35]; // leftThumbRoll
  this->joint[27] =  joint->data[36]; // leftThumbPitch1
  this->joint[28] =  joint->data[26]; // leftIndexFingerPitch1
  this->joint[29] =  joint->data[29]; // leftMiddleFingerPitch1
  this->joint[30] =  joint->data[32]; // leftPinkyPitch1
  this->joint[31] =  joint->data[6];  // rightHipYaw
  this->joint[32] =  joint->data[7];  // rightHipRoll
  this->joint[33] =  joint->data[8];  // rightHipPitch
  this->joint[34] =  joint->data[9];  // rightKneePitch
  this->joint[35] =  joint->data[10]; // rightAnklePitch
  this->joint[36] =  joint->data[11]; // rightAnkleRoll
  this->joint[37] =  joint->data[0];  // leftHipYaw
  this->joint[38] =  joint->data[1];  // leftHipRoll
  this->joint[39] =  joint->data[2];  // leftHipPitch
  this->joint[40] =  joint->data[3];  // leftKneePitch
  this->joint[41] =  joint->data[4];  // leftAnklePitch
  this->joint[42] =  joint->data[5];  // leftAnkleRoll
}

void RvizPublisher::callbackCop(const geometry_msgs::Point::ConstPtr &cop){
  this->cop.x() = cop->x;
  this->cop.y() = cop->y;
  this->cop.z() = cop->z;
}

void RvizPublisher::publishJoint(){
  js.header.stamp = ros::Time::now();
  for(int i = 0; i < 43; i++) {
    js.position[i] = joint[i];
  }
  pubJoint.publish(js);
}

void RvizPublisher::publishCop(){
  // printf("publish %d\n", (int)data.size() );

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x    = cop.x();
  marker.pose.position.y    = cop.y();
  marker.pose.position.z    = cop.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  pubCop.publish(marker);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_piblisher");
  RvizPublisher rviz_piblisher;
  ros::spin();
}