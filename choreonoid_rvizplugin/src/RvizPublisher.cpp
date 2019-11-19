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
  subBody = nh.subscribe<geometry_msgs::Pose>(
    "/simulation/body", 1000, &RvizPublisher::callbackBody, this);
  subCop = nh.subscribe<geometry_msgs::Point>(
    "/simulation/cop", 1000, &RvizPublisher::callbackCop, this);

  // publisher
  pubBody = nh.advertise<visualization_msgs::Marker>("/marker/body", 1);
  pubCop  = nh.advertise<visualization_msgs::Marker>("/marker/cop", 1);

  // initialize
  dt = 0.001;
}

RvizPublisher::~RvizPublisher(){
}

void RvizPublisher::callbackTimeStep(const std_msgs::Float64::ConstPtr &dt){
  this->dt = dt->data;
}

void RvizPublisher::callbackSimulation(const std_msgs::Float64::ConstPtr &t){
  PlotData data_;
  data_.cop    = cop;
  data_.com    = com;
  data_.icp    = icp;
  data_.foot_r = foot_r;
  data_.foot_l = foot_l;
  printf("time %lf\n", (double)t->data);
  data.push_back(data_);

  publishCop();
}

void RvizPublisher::callbackBody(const geometry_msgs::Pose::ConstPtr &body){
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

void RvizPublisher::callbackCop(const geometry_msgs::Point::ConstPtr &cop){
  this->cop.x() = cop->x;
  this->cop.y() = cop->y;
  this->cop.z() = cop->z;
}

void RvizPublisher::publishCop(){
  printf("publish %d\n", (int)data.size() );

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