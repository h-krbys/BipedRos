#include "RvizPublisher.h"

using namespace std;
using namespace Eigen;

RvizPublisher::RvizPublisher(){
  // node
  int    argc = 0;
  char** argv = 0;
  ros::init(argc, argv, "simplecontroller");
  nh = new ros::NodeHandle("");

  // publisher
  pubCop = nh->advertise<visualization_msgs::Marker>("/marker/cop", 1);

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

  publishCop();
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

void RvizPublisher::setCop(Eigen::Vector3f cop){
  this->cop = cop;
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