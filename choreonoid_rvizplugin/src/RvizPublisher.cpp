#include "RvizPublisher.h"

using namespace std;
using namespace Eigen;

RvizPublisher::RvizPublisher() : markerSize(0.1), lineWidth(0.5){
  // node
  int    argc = 0;
  char** argv = 0;
  ros::init(argc, argv, "simplecontroller");
  nh = new ros::NodeHandle("");

  // publisher
  pubComRef = nh->advertise<visualization_msgs::Marker>("/marker/com_ref", 1);
  pubCopRef = nh->advertise<visualization_msgs::Marker>("/marker/cop_ref", 1);
  pubIcpRef = nh->advertise<visualization_msgs::Marker>("/marker/icp_ref", 1);
  pubCom    = nh->advertise<visualization_msgs::Marker>("/marker/com", 1);
  pubCop    = nh->advertise<visualization_msgs::Marker>("/marker/cop", 1);
  pubIcp    = nh->advertise<visualization_msgs::Marker>("/marker/icp", 1);

  // initialize
  dt = 0.001;
}

RvizPublisher::~RvizPublisher(){
}

void RvizPublisher::setTimeStep(double timestep){
  this->dt = timestep;
}

void RvizPublisher::simulation(double time){
  PlotData data_;
  data_.cop    = cop;
  data_.com    = com;
  data_.icp    = icp;
  data_.foot_r = foot_r;
  data_.foot_l = foot_l;
  data.push_back(data_);

  publishComRef();
  publishCopRef();
  publishIcpRef();
  publishCom();
  publishCop();
  publishIcp();
}

void RvizPublisher::setPose(cnoid::BodyPtr body){
  for (int i = 0; i < body->numLinks(); ++i) {
    auto        pos = body->link(i)->translation();
    tf::Vector3 p(pos.x(),
                  pos.y(),
                  pos.z() );
    cnoid::Quaternion quat(body->link(i)->rotation() );
    tf::Quaternion    q(quat.x(),
                        quat.y(),
                        quat.z(),
                        quat.w() );
    tf::Transform transform;
    transform.setOrigin(p);
    transform.setRotation(q);

    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", body->link(i)->name() ) );
  }
}

void RvizPublisher::setComRef(Eigen::Vector3f comRef){
  this->comRef = comRef;
}

void RvizPublisher::setCopRef(Eigen::Vector3f copRef){
  this->copRef = copRef;
}

void RvizPublisher::setIcpRef(Eigen::Vector3f icpRef){
  this->icpRef = icpRef;
}

void RvizPublisher::setCom(Eigen::Vector3f com){
  this->com = com;
}

void RvizPublisher::setCop(Eigen::Vector3f cop){
  this->cop = cop;
}

void RvizPublisher::setIcp(Eigen::Vector3f icp){
  this->icp = icp;
}

void RvizPublisher::publishComRef(){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x    = comRef.x();
  marker.pose.position.y    = comRef.y();
  marker.pose.position.z    = comRef.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = markerSize;
  marker.scale.y = markerSize;
  marker.scale.z = markerSize;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration();

  pubComRef.publish(marker);
}

void RvizPublisher::publishCopRef(){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x    = copRef.x();
  marker.pose.position.y    = copRef.y();
  marker.pose.position.z    = copRef.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = markerSize;
  marker.scale.y = markerSize;
  marker.scale.z = markerSize;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration();

  pubCopRef.publish(marker);
}

void RvizPublisher::publishIcpRef(){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x    = icpRef.x();
  marker.pose.position.y    = icpRef.y();
  marker.pose.position.z    = icpRef.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = markerSize;
  marker.scale.y = markerSize;
  marker.scale.z = markerSize;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration();

  pubIcpRef.publish(marker);
}

void RvizPublisher::publishCom(){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x    = com.x();
  marker.pose.position.y    = com.y();
  marker.pose.position.z    = com.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = markerSize;
  marker.scale.y = markerSize;
  marker.scale.z = markerSize;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  pubCom.publish(marker);
}

void RvizPublisher::publishCop(){
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

  marker.scale.x = markerSize;
  marker.scale.y = markerSize;
  marker.scale.z = markerSize;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  pubCop.publish(marker);
}

void RvizPublisher::publishIcp(){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x    = icp.x();
  marker.pose.position.y    = icp.y();
  marker.pose.position.z    = icp.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = markerSize;
  marker.scale.y = markerSize;
  marker.scale.z = markerSize;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  pubIcp.publish(marker);
}