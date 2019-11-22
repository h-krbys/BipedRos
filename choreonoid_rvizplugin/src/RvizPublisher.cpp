#include "RvizPublisher.h"

using namespace std;
using namespace cnoid;
using namespace Eigen;

RvizPublisher::RvizPublisher() : markerSize(0.1), lineWidth(markerSize / 4.0){
  // node
  int    argc = 0;
  char** argv = 0;
  ros::init(argc, argv, "simplecontroller");
  nh = new ros::NodeHandle("");

  // publisher
  pubComRef     = nh->advertise<visualization_msgs::Marker>("/marker/com_ref", 1);
  pubCopRef     = nh->advertise<visualization_msgs::Marker>("/marker/cop_ref", 1);
  pubIcpRef     = nh->advertise<visualization_msgs::Marker>("/marker/icp_ref", 1);
  pubCom        = nh->advertise<visualization_msgs::Marker>("/marker/com", 1);
  pubCop        = nh->advertise<visualization_msgs::Marker>("/marker/cop", 1);
  pubIcp        = nh->advertise<visualization_msgs::Marker>("/marker/icp", 1);
  pubFootstepR  = nh->advertise<visualization_msgs::MarkerArray>("/marker/footstep_r", 1);
  pubFootstepL  = nh->advertise<visualization_msgs::MarkerArray>("/marker/footstep_l", 1);
  pubComRefTraj = nh->advertise<visualization_msgs::Marker>("/marker/com_ref_traj", 1);
  pubCopRefTraj = nh->advertise<visualization_msgs::Marker>("/marker/cop_ref_traj", 1);
  pubIcpRefTraj = nh->advertise<visualization_msgs::Marker>("/marker/icp_ref_traj", 1);
  pubComTraj    = nh->advertise<visualization_msgs::Marker>("/marker/com_traj", 1);
  pubCopTraj    = nh->advertise<visualization_msgs::Marker>("/marker/cop_traj", 1);
  pubIcpTraj    = nh->advertise<visualization_msgs::Marker>("/marker/icp_traj", 1);
  pubFootRTraj  = nh->advertise<visualization_msgs::Marker>("/marker/foot_r_traj", 1);
  pubFootLTraj  = nh->advertise<visualization_msgs::Marker>("/marker/foot_l_traj", 1);

  // get instance of each bar
  timeBar = TimeBar::instance();

  // connection
  timeBarConnection = timeBar->sigTimeChanged().connect(
    bind(&RvizPublisher::timeChanged, this, _1) );

  // initialize
  dt           = 0.001;
  isSimulation = true;
  isPlayback   = false;
}

RvizPublisher::~RvizPublisher(){
}

void RvizPublisher::initialize(){
  footstepR.clear();
  footstepL.clear();
  data.clear();
}

bool RvizPublisher::timeChanged(double time){
  if(isSimulation) {
    if(time > dt && !timeBar->isDoingPlayback() ) {
      isSimulation = false;
    }
  }else{
    isPlayback = true;
  }

  if(isPlayback) {
    playback(time);
  }
}

void RvizPublisher::simulation(double time){
  PlotData data_;
  data_.copRef    = copRef;
  data_.comRef    = comRef;
  data_.icpRef    = icpRef;
  data_.cop       = cop;
  data_.com       = com;
  data_.icp       = icp;
  data_.footR     = footR;
  data_.footL     = footL;
  data_.footstepR = footstepR;
  data_.footstepL = footstepL;
  data.push_back(data_);

  publishComRef();
  publishCopRef();
  publishIcpRef();
  publishCom();
  publishCop();
  publishIcp();
  publishFootstepR();
  publishFootstepL();
  publishComRefTraj(time);
  publishCopRefTraj(time);
  publishIcpRefTraj(time);
  publishComTraj(time);
  publishCopTraj(time);
  publishIcpTraj(time);
  publishFootRTraj(time);
  publishFootLTraj(time);

  maxTime = time;
}

void RvizPublisher::playback(double time){
  if(time <= maxTime) {
    const int num_timestep = time / dt;
    copRef    = data[num_timestep].copRef;
    comRef    = data[num_timestep].comRef;
    icpRef    = data[num_timestep].icpRef;
    cop       = data[num_timestep].cop;
    com       = data[num_timestep].com;
    icp       = data[num_timestep].icp;
    footR     = data[num_timestep].footR;
    footL     = data[num_timestep].footL;
    footstepR = data[num_timestep].footstepR;
    footstepL = data[num_timestep].footstepL;
    publishComRef();
    publishCopRef();
    publishIcpRef();
    publishCom();
    publishCop();
    publishIcp();
    publishFootstepR();
    publishFootstepL();
    publishComRefTraj(time);
    publishCopRefTraj(time);
    publishIcpRefTraj(time);
    publishComTraj(time);
    publishCopTraj(time);
    publishIcpTraj(time);
    publishFootRTraj(time);
    publishFootLTraj(time);
  }
}

void RvizPublisher::setTimeStep(double timestep){
  this->dt = timestep;
}

void RvizPublisher::setPose(cnoid::BodyPtr body){
  this->body = body;
  for (int i = 0; i < body->numLinks(); ++i) {
    publishPose(*( body->link(i) ) );
  }
  footR.x() = body->link("rightFootSole")->translation().x();
  footR.y() = body->link("rightFootSole")->translation().y();
  footR.z() = body->link("rightFootSole")->translation().z();
  footL.x() = body->link("leftFootSole")->translation().x();
  footL.y() = body->link("leftFootSole")->translation().y();
  footL.z() = body->link("leftFootSole")->translation().z();
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

void RvizPublisher::setFootstepR(std::vector<Eigen::Vector3f> footstepR){
  this->footstepR = footstepR;
}

void RvizPublisher::setFootstepL(std::vector<Eigen::Vector3f> footstepL){
  this->footstepL = footstepL;
}

void RvizPublisher::publishPose(cnoid::Link link){
  Position::TranslationPart pos = link.translation();
  tf::Vector3               p(pos.x(),
                              pos.y(),
                              pos.z() );
  Quaternion     quat(link.rotation() );
  tf::Quaternion q(quat.x(),
                   quat.y(),
                   quat.z(),
                   quat.w() );
  tf::Transform transform;
  transform.setOrigin(p);
  transform.setRotation(q);

  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", link.name() ) );
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

void RvizPublisher::publishFootstepR(){
  const int num_step = (int)footstepR.size();

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(num_step);
  for (int i = 0; i < num_step; i++) {
    marker_array.markers[i].header.frame_id = "world";
    marker_array.markers[i].header.stamp    = ros::Time::now();
    marker_array.markers[i].ns              = "markers";
    marker_array.markers[i].id              = i;
    marker_array.markers[i].lifetime        = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;

    marker_array.markers[i].action = visualization_msgs::Marker::ADD;

    marker_array.markers[i].pose.position.x    = footstepR[i].x();
    marker_array.markers[i].pose.position.y    = footstepR[i].y();
    marker_array.markers[i].pose.position.z    = footstepR[i].z();
    marker_array.markers[i].pose.orientation.x = 0.0;
    marker_array.markers[i].pose.orientation.y = 0.0;
    marker_array.markers[i].pose.orientation.z = 0.0;
    marker_array.markers[i].pose.orientation.w = 1.0;

    marker_array.markers[i].scale.x = 1.0;
    marker_array.markers[i].scale.y = 1.0;
    marker_array.markers[i].scale.z = 1.0;

    marker_array.markers[i].mesh_resource = "package://choreonoid_rvizplugin/meshes/valkyrie_foot.stl";
    marker_array.markers[i].color.r       = 0.0;
    marker_array.markers[i].color.g       = 1.0;
    marker_array.markers[i].color.b       = 0.0;
    marker_array.markers[i].color.a       = 0.5;
  }
  pubFootstepR.publish(marker_array);
}

void RvizPublisher::publishFootstepL(){
  const int num_step = (int)footstepL.size();

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(num_step);
  for (int i = 0; i < num_step; i++) {
    marker_array.markers[i].header.frame_id = "world";
    marker_array.markers[i].header.stamp    = ros::Time::now();
    marker_array.markers[i].ns              = "markers";
    marker_array.markers[i].id              = i;
    marker_array.markers[i].lifetime        = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;

    marker_array.markers[i].action = visualization_msgs::Marker::ADD;

    marker_array.markers[i].pose.position.x    = footstepL[i].x();
    marker_array.markers[i].pose.position.y    = footstepL[i].y();
    marker_array.markers[i].pose.position.z    = footstepL[i].z();
    marker_array.markers[i].pose.orientation.x = 0.0;
    marker_array.markers[i].pose.orientation.y = 0.0;
    marker_array.markers[i].pose.orientation.z = 0.0;
    marker_array.markers[i].pose.orientation.w = 1.0;

    marker_array.markers[i].scale.x = 1.0;
    marker_array.markers[i].scale.y = 1.0;
    marker_array.markers[i].scale.z = 1.0;

    marker_array.markers[i].mesh_resource = "package://choreonoid_rvizplugin/meshes/valkyrie_foot.stl";
    marker_array.markers[i].color.r       = 1.0;
    marker_array.markers[i].color.g       = 0.0;
    marker_array.markers[i].color.b       = 0.0;
    marker_array.markers[i].color.a       = 0.5;
  }
  pubFootstepL.publish(marker_array);
}

void RvizPublisher::publishComRefTraj(double time){
  const int num_points = time / dt;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = lineWidth;
  marker.scale.y = lineWidth;
  marker.scale.z = lineWidth;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;

  marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    marker.points[i].x = data[i].comRef.x();
    marker.points[i].y = data[i].comRef.y();
    marker.points[i].z = data[i].comRef.z();
  }

  marker.lifetime = ros::Duration();

  pubComRefTraj.publish(marker);
}

void RvizPublisher::publishCopRefTraj(double time){
  const int num_points = time / dt;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = lineWidth;
  marker.scale.y = lineWidth;
  marker.scale.z = lineWidth;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;

  marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    marker.points[i].x = data[i].copRef.x();
    marker.points[i].y = data[i].copRef.y();
    marker.points[i].z = data[i].copRef.z();
  }

  marker.lifetime = ros::Duration();

  pubCopRefTraj.publish(marker);
}

void RvizPublisher::publishIcpRefTraj(double time){
  const int num_points = time / dt;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = lineWidth;
  marker.scale.y = lineWidth;
  marker.scale.z = lineWidth;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.5;

  marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    marker.points[i].x = data[i].icpRef.x();
    marker.points[i].y = data[i].icpRef.y();
    marker.points[i].z = data[i].icpRef.z();
  }

  marker.lifetime = ros::Duration();

  pubIcpRefTraj.publish(marker);
}

void RvizPublisher::publishComTraj(double time){
  const int num_points = time / dt;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = lineWidth;
  marker.scale.y = lineWidth;
  marker.scale.z = lineWidth;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    marker.points[i].x = data[i].com.x();
    marker.points[i].y = data[i].com.y();
    marker.points[i].z = data[i].com.z();
  }

  marker.lifetime = ros::Duration();

  pubComTraj.publish(marker);
}

void RvizPublisher::publishCopTraj(double time){
  const int num_points = time / dt;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = lineWidth;
  marker.scale.y = lineWidth;
  marker.scale.z = lineWidth;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    marker.points[i].x = data[i].cop.x();
    marker.points[i].y = data[i].cop.y();
    marker.points[i].z = data[i].cop.z();
  }

  marker.lifetime = ros::Duration();

  pubCopTraj.publish(marker);
}

void RvizPublisher::publishIcpTraj(double time){
  const int num_points = time / dt;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = lineWidth;
  marker.scale.y = lineWidth;
  marker.scale.z = lineWidth;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    marker.points[i].x = data[i].icp.x();
    marker.points[i].y = data[i].icp.y();
    marker.points[i].z = data[i].icp.z();
  }

  marker.lifetime = ros::Duration();

  pubIcpTraj.publish(marker);
}

void RvizPublisher::publishFootRTraj(double time){
  const int num_points = time / dt;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = lineWidth;
  marker.scale.y = lineWidth;
  marker.scale.z = lineWidth;

  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    marker.points[i].x = data[i].footR.x();
    marker.points[i].y = data[i].footR.y();
    marker.points[i].z = data[i].footR.z();
  }

  marker.lifetime = ros::Duration();

  pubFootRTraj.publish(marker);
}

void RvizPublisher::publishFootLTraj(double time){
  const int num_points = time / dt;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = lineWidth;
  marker.scale.y = lineWidth;
  marker.scale.z = lineWidth;

  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    marker.points[i].x = data[i].footL.x();
    marker.points[i].y = data[i].footL.y();
    marker.points[i].z = data[i].footL.z();
  }

  marker.lifetime = ros::Duration();

  pubFootLTraj.publish(marker);
}