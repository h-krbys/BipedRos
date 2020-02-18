#include "RvizPublisher.h"

using namespace std;
using namespace cnoid;
using namespace Eigen;

RvizPublisher::RvizPublisher() : markerSize(0.1), lineWidth(markerSize / 10.0), forceScale(0.0025){
  // node
  int    argc = 0;
  char** argv = 0;
  ros::init(argc, argv, "simplecontroller");
  nh = new ros::NodeHandle("");

  // publisher
  pubComRef       = nh->advertise<visualization_msgs::Marker>("/marker/com_ref", 1);
  pubCopRef       = nh->advertise<visualization_msgs::Marker>("/marker/cop_ref", 1);
  pubIcpRef       = nh->advertise<visualization_msgs::Marker>("/marker/icp_ref", 1);
  pubCom          = nh->advertise<visualization_msgs::Marker>("/marker/com", 1);
  pubCop          = nh->advertise<visualization_msgs::Marker>("/marker/cop", 1);
  pubIcp          = nh->advertise<visualization_msgs::Marker>("/marker/icp", 1);
  pubFootRRef     = nh->advertise<visualization_msgs::Marker>("/marker/foot_r_ref", 1);
  pubFootLRef     = nh->advertise<visualization_msgs::Marker>("/marker/foot_l_ref", 1);
  pubFootstepRef  = nh->advertise<visualization_msgs::MarkerArray>("/marker/footstep_ref", 1);
  pubFootstepR    = nh->advertise<visualization_msgs::MarkerArray>("/marker/footstep_r", 1);
  pubFootstepL    = nh->advertise<visualization_msgs::MarkerArray>("/marker/footstep_l", 1);
  pubGridMap      = nh->advertise<visualization_msgs::Marker>("/marker/capture_region", 1);
  pubForce        = nh->advertise<visualization_msgs::Marker>("/marker/force", 1);
  pubPendulumRef  = nh->advertise<visualization_msgs::Marker>("/marker/pendulum_ref", 1);
  pubPendulum     = nh->advertise<visualization_msgs::Marker>("/marker/pendulum", 1);
  pubComRefTraj   = nh->advertise<visualization_msgs::Marker>("/marker/com_ref_traj", 1);
  pubCopRefTraj   = nh->advertise<visualization_msgs::Marker>("/marker/cop_ref_traj", 1);
  pubIcpRefTraj   = nh->advertise<visualization_msgs::Marker>("/marker/icp_ref_traj", 1);
  pubFootRRefTraj = nh->advertise<visualization_msgs::Marker>("/marker/foot_r_ref_traj", 1);
  pubFootLRefTraj = nh->advertise<visualization_msgs::Marker>("/marker/foot_l_ref_traj", 1);
  pubComTraj      = nh->advertise<visualization_msgs::Marker>("/marker/com_traj", 1);
  pubCopTraj      = nh->advertise<visualization_msgs::Marker>("/marker/cop_traj", 1);
  pubIcpTraj      = nh->advertise<visualization_msgs::Marker>("/marker/icp_traj", 1);
  pubFootRTraj    = nh->advertise<visualization_msgs::Marker>("/marker/foot_r_traj", 1);
  pubFootLTraj    = nh->advertise<visualization_msgs::Marker>("/marker/foot_l_traj", 1);

  // get instance of each bar
  timeBar = TimeBar::instance();

  // connection
  timeBarConnection = timeBar->sigTimeChanged().connect(
    bind(&RvizPublisher::timeChanged, this, _1) );

  // initialize
  dt           = 0.001;
  maxTime      = 0.0;
  isSimulation = true;
  isPlayback   = false;
}

RvizPublisher::~RvizPublisher(){
}

void RvizPublisher::initialize(){
  footstepRef.clear();
  footstepR.clear();
  footstepL.clear();
  gridMap.clear();
  data.clear();
}

void RvizPublisher::save(){
  printf("saving ...\n");
  FILE *fp = fopen("/home/dl-box/simulation.csv", "w");
  if(fp) {
    fprintf(fp, "%s,", "time");
    fprintf(fp, "%s, %s,", "copRef.x", "copRef.y");
    fprintf(fp, "%s, %s,", "cop.x", "cop.y");
    fprintf(fp, "%s, %s,", "comRef.x", "comRef.y");
    fprintf(fp, "%s, %s,", "com.x", "com.y");
    fprintf(fp, "%s, %s,", "icpRef.x", "icpRef.y");
    fprintf(fp, "%s, %s,", "icp.x", "icp.y");
    fprintf(fp, "%s, %s, %s,", "footRRef.x", "footRRef.y", "footRRef.z");
    fprintf(fp, "%s, %s, %s,", "footR.x", "footR.y", "footR.z");
    fprintf(fp, "%s, %s, %s,", "footLRef.x", "footLRef.y", "footLRef.z");
    fprintf(fp, "%s, %s, %s,", "footL.x", "footL.y", "footL.z");
    fprintf(fp, "%s, %s\n", "force.x", "force.y");
    for(size_t i = 0; i < data.size(); i++) {
      fprintf(fp, "%1.4lf,", i * dt);
      fprintf(fp, "%1.3lf, %1.3lf,", data[i].copRef.x(), data[i].copRef.y() );
      fprintf(fp, "%1.3lf, %1.3lf,", data[i].cop.x(), data[i].cop.y() );
      fprintf(fp, "%1.3lf, %1.3lf,", data[i].comRef.x(), data[i].comRef.y() );
      fprintf(fp, "%1.3lf, %1.3lf,", data[i].com.x(), data[i].com.y() );
      fprintf(fp, "%1.3lf, %1.3lf,", data[i].icpRef.x(), data[i].icpRef.y() );
      fprintf(fp, "%1.3lf, %1.3lf,", data[i].icp.x(), data[i].icp.y() );
      fprintf(fp, "%1.3lf, %1.3lf, %1.3lf,", data[i].footRRef.x(), data[i].footRRef.y(), data[i].footRRef.z() );
      fprintf(fp, "%1.3lf, %1.3lf, %1.3lf,", data[i].footR.x(), data[i].footR.y(), data[i].footR.z() );
      fprintf(fp, "%1.3lf, %1.3lf, %1.3lf,", data[i].footLRef.x(), data[i].footLRef.y(), data[i].footLRef.z() );
      fprintf(fp, "%1.3lf, %1.3lf, %1.3lf,", data[i].footLRef.x(), data[i].footLRef.y(), data[i].footLRef.z() );
      fprintf(fp, "%1.3lf, %1.3lf\n", data[i].force.x(), data[i].force.y() );
    }
    fclose(fp);
    printf("saved !\n");
  }else{
    printf("Error: file couldn't opened.\n");
  }
}

bool RvizPublisher::timeChanged(double time){
  if(isSimulation) {
    if(time > dt && !timeBar->isDoingPlayback() ) {
      isSimulation = false;
      save();
    }
  }else{
    isPlayback = true;
  }

  if(isPlayback) {
    playback(time);
  }
}

void RvizPublisher::simulation(double time){
  data_.e_copRef      = e_copRef;
  data_.e_comRef      = e_comRef;
  data_.e_icpRef      = e_icpRef;
  data_.e_cop         = e_cop;
  data_.e_com         = e_com;
  data_.e_icp         = e_icp;
  data_.e_footRRef    = e_footRRef;
  data_.e_footLRef    = e_footLRef;
  data_.e_footR       = e_footR;
  data_.e_footL       = e_footL;
  data_.e_footstepRef = e_footstepRef;
  data_.e_footstepR   = e_footstepR;
  data_.e_footstepL   = e_footstepL;
  data_.e_gridMap     = e_gridMap;
  data_.e_force       = e_force;

  data_.link        = link;
  data_.copRef      = copRef;
  data_.comRef      = comRef;
  data_.icpRef      = icpRef;
  data_.footRRef    = footRRef;
  data_.footLRef    = footLRef;
  data_.cop         = cop;
  data_.com         = com;
  data_.icp         = icp;
  data_.footR       = footR;
  data_.footL       = footL;
  data_.footstepRef = footstepRef;
  data_.footstepR   = footstepR;
  data_.footstepL   = footstepL;
  data_.gridMap     = gridMap;
  data_.force       = force;
  data.push_back(data_);

  publishAll(time);

  maxTime = time;
}

void RvizPublisher::playback(double time){
  if(time <= maxTime) {
    const int num_timestep = time / dt;
    e_copRef      = data[num_timestep].e_copRef;
    e_comRef      = data[num_timestep].e_comRef;
    e_icpRef      = data[num_timestep].e_icpRef;
    e_cop         = data[num_timestep].e_cop;
    e_com         = data[num_timestep].e_com;
    e_icp         = data[num_timestep].e_icp;
    e_footRRef    = data[num_timestep].e_footRRef;
    e_footLRef    = data[num_timestep].e_footLRef;
    e_footR       = data[num_timestep].e_footR;
    e_footL       = data[num_timestep].e_footL;
    e_footstepRef = data[num_timestep].e_footstepRef;
    e_footstepR   = data[num_timestep].e_footstepR;
    e_footstepL   = data[num_timestep].e_footstepL;
    e_gridMap     = data[num_timestep].e_gridMap;
    e_force       = data[num_timestep].e_force;

    link        = data[num_timestep].link;
    copRef      = data[num_timestep].copRef;
    comRef      = data[num_timestep].comRef;
    icpRef      = data[num_timestep].icpRef;
    footRRef    = data[num_timestep].footRRef;
    footLRef    = data[num_timestep].footLRef;
    cop         = data[num_timestep].cop;
    com         = data[num_timestep].com;
    icp         = data[num_timestep].icp;
    footR       = data[num_timestep].footR;
    footL       = data[num_timestep].footL;
    footstepRef = data[num_timestep].footstepRef;
    footstepR   = data[num_timestep].footstepR;
    footstepL   = data[num_timestep].footstepL;
    gridMap     = data[num_timestep].gridMap;
    force       = data[num_timestep].force;

    publishAll(time);
  }
}

void RvizPublisher::publishAll(double time){
  publishPose();
  publishComRef();
  publishCopRef();
  publishIcpRef();
  publishFootRRef();
  publishFootLRef();
  publishCom();
  publishCop();
  publishIcp();
  publishFootstepRef();
  publishFootstepR();
  publishFootstepL();
  publishGridMap();
  publishForce();
  publishPendulumRef();
  publishPendulum();
  publishComRefTraj(time);
  publishCopRefTraj(time);
  publishIcpRefTraj(time);
  publishFootRRefTraj(time);
  publishFootLRefTraj(time);
  publishComTraj(time);
  publishCopTraj(time);
  publishIcpTraj(time);
  publishFootRTraj(time);
  publishFootLTraj(time);
}

void RvizPublisher::setTimeStep(double timestep){
  this->dt = timestep;
}

void RvizPublisher::setPose(cnoid::BodyPtr body){
  link.clear();
  for (int i = 0; i < body->numLinks(); ++i) {
    LinkData link_;
    link_.name = body->link(i)->name();
    Position::TranslationPart pos = body->link(i)->translation();
    link_.p_x = pos.x();
    link_.p_y = pos.y();
    link_.p_z = pos.z();
    Quaternion quat(body->link(i)->rotation() );
    link_.q_x = quat.x();
    link_.q_y = quat.y();
    link_.q_z = quat.z();
    link_.q_w = quat.w();
    link.push_back(link_);
  }
  footR.x() = body->link("rightFootSole")->translation().x();
  footR.y() = body->link("rightFootSole")->translation().y();
  footR.z() = body->link("rightFootSole")->translation().z();
  footL.x() = body->link("leftFootSole")->translation().x();
  footL.y() = body->link("leftFootSole")->translation().y();
  footL.z() = body->link("leftFootSole")->translation().z();
}

void RvizPublisher::setCopRef(Vector3 copRef){
  e_copRef     = true;
  this->copRef = copRef;
}

void RvizPublisher::setComRef(Vector3 comRef){
  e_comRef     = true;
  this->comRef = comRef;
}

void RvizPublisher::setIcpRef(Vector3 icpRef){
  e_icpRef     = true;
  this->icpRef = icpRef;
}

void RvizPublisher::setFootRRef(Vector3 footRRef){
  e_footRRef     = true;
  this->footRRef = footRRef;
}

void RvizPublisher::setFootLRef(Vector3 footLRef){
  e_footLRef     = true;
  this->footLRef = footLRef;
}

void RvizPublisher::setCom(Vector3 com){
  e_com     = true;
  this->com = com;
}

void RvizPublisher::setCop(Vector3 cop){
  e_cop     = true;
  this->cop = cop;
}

void RvizPublisher::setIcp(Vector3 icp){
  e_icp     = true;
  this->icp = icp;
}

void RvizPublisher::setFootstepRef(std::vector<Vector3> footstepRef){
  e_footstepRef = true;
  this->footstepRef.clear();
  this->footstepRef = footstepRef;
}

void RvizPublisher::setFootstepR(std::vector<Vector3> footstepR){
  e_footstepR = true;
  this->footstepR.clear();
  this->footstepR = footstepR;
}

void RvizPublisher::setFootstepL(std::vector<Vector3> footstepL){
  e_footstepL = true;
  this->footstepL.clear();
  this->footstepL = footstepL;
}

void RvizPublisher::setGridMap(std::vector<CaptData> gridMap){
  e_gridMap = true;
  this->gridMap.clear();
  this->gridMap = gridMap;
}

void RvizPublisher::setForce(Vector3 force){
  e_force     = true;
  this->force = force;
}

void RvizPublisher::publishPose(){
  for(size_t i = 0; i < link.size(); i++) {
    tf::Vector3 p(link[i].p_x,
                  link[i].p_y,
                  link[i].p_z );
    tf::Quaternion q(link[i].q_x,
                     link[i].q_y,
                     link[i].q_z,
                     link[i].q_w );
    tf::Transform transform;
    transform.setOrigin(p);
    transform.setRotation(q);

    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", link[i].name.c_str() ) );
  }
}

void RvizPublisher::publishComRef(){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type = visualization_msgs::Marker::SPHERE;
  if(e_comRef == true) {
    marker.action = visualization_msgs::Marker::ADD;
  }else{
    marker.action = visualization_msgs::Marker::DELETE;
  }

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

  marker.type = visualization_msgs::Marker::SPHERE;
  if(e_copRef == true) {
    marker.action = visualization_msgs::Marker::ADD;
  }else{
    marker.action = visualization_msgs::Marker::DELETE;
  }

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

  marker.type = visualization_msgs::Marker::SPHERE;
  if(e_icpRef == true) {
    marker.action = visualization_msgs::Marker::ADD;
  }else{
    marker.action = visualization_msgs::Marker::DELETE;
  }

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

void RvizPublisher::publishFootRRef(){
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;
  marker.lifetime        = ros::Duration();

  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  if(e_footRRef == true) {
    marker.action = visualization_msgs::Marker::ADD;
  }else{
    marker.action = visualization_msgs::Marker::DELETE;
  }

  marker.pose.position.x    = footRRef.x();
  marker.pose.position.y    = footRRef.y();
  marker.pose.position.z    = footRRef.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.mesh_resource = "package://choreonoid_rvizplugin/meshes/valkyrie_foot.stl";
  marker.color.r       = 1.0;
  marker.color.g       = 1.0;
  marker.color.b       = 1.0;
  marker.color.a       = 1.0;

  pubFootRRef.publish(marker);
}

void RvizPublisher::publishFootLRef(){
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;
  marker.lifetime        = ros::Duration();

  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  if(e_footLRef == true) {
    marker.action = visualization_msgs::Marker::ADD;
  }else{
    marker.action = visualization_msgs::Marker::DELETE;
  }

  marker.pose.position.x    = footLRef.x();
  marker.pose.position.y    = footLRef.y();
  marker.pose.position.z    = footLRef.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.mesh_resource = "package://choreonoid_rvizplugin/meshes/valkyrie_foot.stl";
  marker.color.r       = 1.0;
  marker.color.g       = 1.0;
  marker.color.b       = 1.0;
  marker.color.a       = 1.0;

  pubFootLRef.publish(marker);
}

void RvizPublisher::publishCom(){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type = visualization_msgs::Marker::SPHERE;
  if(e_com == true) {
    marker.action = visualization_msgs::Marker::ADD;
  }else{
    marker.action = visualization_msgs::Marker::DELETE;
  }

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

  marker.type = visualization_msgs::Marker::SPHERE;
  if(e_cop == true) {
    marker.action = visualization_msgs::Marker::ADD;
  }else{
    marker.action = visualization_msgs::Marker::DELETE;
  }

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

  marker.type = visualization_msgs::Marker::SPHERE;
  if(e_icp == true) {
    marker.action = visualization_msgs::Marker::ADD;
  }else{
    marker.action = visualization_msgs::Marker::DELETE;
  }

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

void RvizPublisher::publishFootstepRef(){
  const int num_step = (int)footstepRef.size();

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(num_step);
  for (int i = 0; i < num_step; i++) {
    marker_array.markers[i].header.frame_id = "world";
    marker_array.markers[i].header.stamp    = ros::Time::now();
    marker_array.markers[i].ns              = "markers";
    marker_array.markers[i].id              = i;
    marker_array.markers[i].lifetime        = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;

    if(e_footstepRef == true) {
      marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    }else{
      marker_array.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }

    marker_array.markers[i].pose.position.x    = footstepRef[i].x();
    marker_array.markers[i].pose.position.y    = footstepRef[i].y();
    marker_array.markers[i].pose.position.z    = footstepRef[i].z();
    marker_array.markers[i].pose.orientation.x = 0.0;
    marker_array.markers[i].pose.orientation.y = 0.0;
    marker_array.markers[i].pose.orientation.z = 0.0;
    marker_array.markers[i].pose.orientation.w = 1.0;

    marker_array.markers[i].scale.x = 1.0;
    marker_array.markers[i].scale.y = 1.0;
    marker_array.markers[i].scale.z = 1.0;

    marker_array.markers[i].mesh_resource = "package://choreonoid_rvizplugin/meshes/valkyrie_foot.stl";
    marker_array.markers[i].color.r       = 1.0;
    marker_array.markers[i].color.g       = 1.0;
    marker_array.markers[i].color.b       = 1.0;
    marker_array.markers[i].color.a       = 0.5;
  }

  if(e_footstepRef == false) {
    marker_array.markers.resize(1);
    marker_array.markers[0].action = visualization_msgs::Marker::DELETEALL;
  }

  pubFootstepRef.publish(marker_array);
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

    if(e_footstepR == true) {
      marker_array.markers[i].action = visualization_msgs::Marker::MODIFY;
    }else{
      marker_array.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }

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

  if(num_step == 0) {
    marker_array.markers.clear();
    marker_array.markers.resize(1);
    marker_array.markers[0].header.frame_id = "world";
    marker_array.markers[0].header.stamp    = ros::Time::now();
    marker_array.markers[0].ns              = "markers";
    marker_array.markers[0].id              = 0;
    marker_array.markers[0].lifetime        = ros::Duration();

    marker_array.markers[0].type   = visualization_msgs::Marker::MESH_RESOURCE;
    marker_array.markers[0].action = visualization_msgs::Marker::DELETEALL;
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

    if(e_footstepL == true) {
      marker_array.markers[i].action = visualization_msgs::Marker::MODIFY;
    }else{
      marker_array.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }

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

  if(num_step == 0) {
    marker_array.markers.clear();
    marker_array.markers.resize(1);
    marker_array.markers[0].header.frame_id = "world";
    marker_array.markers[0].header.stamp    = ros::Time::now();
    marker_array.markers[0].ns              = "markers";
    marker_array.markers[0].id              = 0;
    marker_array.markers[0].lifetime        = ros::Duration();

    marker_array.markers[0].type   = visualization_msgs::Marker::MESH_RESOURCE;
    marker_array.markers[0].action = visualization_msgs::Marker::DELETEALL;
  }

  pubFootstepL.publish(marker_array);
}

void RvizPublisher::publishGridMap(){
  const int size = (int)gridMap.size();

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;
  marker.lifetime        = ros::Duration();

  marker.type = visualization_msgs::Marker::CUBE_LIST;
  if(e_gridMap == true) {
    marker.action = visualization_msgs::Marker::ADD;
  }else{
    marker.action = visualization_msgs::Marker::DELETE;
  }

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.001;

  marker.points.resize(size);
  marker.colors.resize(size);
  for (size_t i = 0; i < size; i++) {
    marker.points[i].x = gridMap[i].pos.x();
    marker.points[i].y = gridMap[i].pos.y();
    marker.points[i].z = gridMap[i].pos.z();

    switch (gridMap[i].nstep) {
    case 0:
      marker.colors[i].r = 1.0;
      marker.colors[i].g = 1.0;
      marker.colors[i].b = 1.0;
      marker.colors[i].a = 0.0;
      break;
    case 1: // #cbfeff
      marker.colors[i].r = 203 / 255.0;
      marker.colors[i].g = 254 / 255.0;
      marker.colors[i].b = 255 / 255.0;
      marker.colors[i].a = 1.0;
      break;
    case 2: // #68fefe
      marker.colors[i].r = 104 / 255.0;
      marker.colors[i].g = 254 / 255.0;
      marker.colors[i].b = 254 / 255.0;
      marker.colors[i].a = 1.0;
      break;
    case 3: // #0097ff
      marker.colors[i].r = 0 / 255.0;
      marker.colors[i].g = 151 / 255.0;
      marker.colors[i].b = 255 / 255.0;
      marker.colors[i].a = 1.0;
      break;
    case 4: // #0000ff
      marker.colors[i].r = 0 / 255.0;
      marker.colors[i].g = 0 / 255.0;
      marker.colors[i].b = 255 / 255.0;
      marker.colors[i].a = 1.0;
      break;
    }
  }
  pubGridMap.publish(marker);
}

void RvizPublisher::publishForce(){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type = visualization_msgs::Marker::ARROW;
  if(e_force == true) {
    marker.action = visualization_msgs::Marker::ADD;
  }else{
    marker.action = visualization_msgs::Marker::DELETE;
  }

  marker.scale.x = 0.03; // line width
  marker.scale.y = 0.06; // hat width
  marker.scale.z = 0.05; // simeq hat length / line length

  marker.color.r = 25.0 / 255;
  marker.color.g = 255.0 / 255;
  marker.color.b = 240.0 / 255;
  marker.color.a = 0.8;

  marker.points.resize(2);
  marker.points[0].x = com.x(); // start pos.
  marker.points[0].y = com.y();
  marker.points[0].z = com.z();
  marker.points[1].x = com.x() + forceScale * force.x(); // end pos.
  marker.points[1].y = com.y() + forceScale * force.y();
  marker.points[1].z = com.z() + forceScale * force.z();

  marker.lifetime = ros::Duration();

  pubForce.publish(marker);
}

void RvizPublisher::publishPendulumRef(){
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  // white
  // marker.scale.x = lineWidth * 2;
  // marker.scale.y = lineWidth * 2;
  // marker.scale.z = lineWidth * 2;
  //
  // marker.color.r = 1.0;
  // marker.color.g = 1.0;
  // marker.color.b = 1.0;
  // marker.color.a = 0.5;

  // black
  marker.scale.x = lineWidth;
  marker.scale.y = lineWidth;
  marker.scale.z = lineWidth;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;

  if(e_copRef && e_comRef) {
    marker.points.resize(2);
    marker.points[0].x = copRef.x();
    marker.points[0].y = copRef.y();
    marker.points[0].z = copRef.z();
    marker.points[1].x = comRef.x();
    marker.points[1].y = comRef.y();
    marker.points[1].z = comRef.z();
  }

  marker.lifetime = ros::Duration();

  pubPendulumRef.publish(marker);
}

void RvizPublisher::publishPendulum(){
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "markers";
  marker.id              = 0;

  marker.type   = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  // white
  // marker.scale.x = lineWidth * 2;
  // marker.scale.y = lineWidth * 2;
  // marker.scale.z = lineWidth * 2;
  //
  // marker.color.r = 1.0;
  // marker.color.g = 1.0;
  // marker.color.b = 1.0;
  // marker.color.a = 0.5;

  // black
  marker.scale.x = lineWidth;
  marker.scale.y = lineWidth;
  marker.scale.z = lineWidth;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  if(e_cop && e_com) {
    marker.points.resize(2);
    marker.points[0].x = cop.x();
    marker.points[0].y = cop.y();
    marker.points[0].z = cop.z();
    marker.points[1].x = com.x();
    marker.points[1].y = com.y();
    marker.points[1].z = com.z();
  }

  marker.lifetime = ros::Duration();

  pubPendulum.publish(marker);
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

  // marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    if(data[i].e_comRef) {
      geometry_msgs::Point point;
      point.x = data[i].comRef.x();
      point.y = data[i].comRef.y();
      point.z = data[i].comRef.z();
      marker.points.push_back(point);
    }
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

  // marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    if(data[i].e_copRef) {
      geometry_msgs::Point point;
      point.x = data[i].copRef.x();
      point.y = data[i].copRef.y();
      point.z = data[i].copRef.z();
      marker.points.push_back(point);
    }
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

  // marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    if(data[i].e_icpRef) {
      geometry_msgs::Point point;
      point.x = data[i].icpRef.x();
      point.y = data[i].icpRef.y();
      point.z = data[i].icpRef.z();
      marker.points.push_back(point);
    }
  }

  marker.lifetime = ros::Duration();

  pubIcpRefTraj.publish(marker);
}

void RvizPublisher::publishFootRRefTraj(double time){
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
  marker.color.b = 0.0;
  marker.color.a = 0.5;

  // marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    if(data[i].e_footRRef) {
      geometry_msgs::Point point;
      point.x = data[i].footRRef.x();
      point.y = data[i].footRRef.y();
      point.z = data[i].footRRef.z();
      marker.points.push_back(point);
    }
  }

  marker.lifetime = ros::Duration();

  pubFootRRefTraj.publish(marker);
}

void RvizPublisher::publishFootLRefTraj(double time){
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
  marker.color.b = 0.0;
  marker.color.a = 0.5;

  // marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    if(data[i].e_footLRef) {
      geometry_msgs::Point point;
      point.x = data[i].footLRef.x();
      point.y = data[i].footLRef.y();
      point.z = data[i].footLRef.z();
      marker.points.push_back(point);
    }
  }

  marker.lifetime = ros::Duration();

  pubFootLRefTraj.publish(marker);
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

  // marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    if(data[i].e_com) {
      geometry_msgs::Point point;
      point.x = data[i].com.x();
      point.y = data[i].com.y();
      point.z = data[i].com.z();
      marker.points.push_back(point);
    }
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

  // marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    if(data[i].e_cop) {
      geometry_msgs::Point point;
      point.x = data[i].cop.x();
      point.y = data[i].cop.y();
      point.z = data[i].cop.z();
      marker.points.push_back(point);
    }
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

  // marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    if(data[i].e_icp) {
      geometry_msgs::Point point;
      point.x = data[i].icp.x();
      point.y = data[i].icp.y();
      point.z = data[i].icp.z();
      marker.points.push_back(point);
    }
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

  // marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    if(data[i].e_footR) {
      geometry_msgs::Point point;
      point.x = data[i].footR.x();
      point.y = data[i].footR.y();
      point.z = data[i].footR.z();
      marker.points.push_back(point);
    }
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

  // marker.points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    if(data[i].e_footL) {
      geometry_msgs::Point point;
      point.x = data[i].footL.x();
      point.y = data[i].footL.y();
      point.z = data[i].footL.z();
      marker.points.push_back(point);
    }
  }

  marker.lifetime = ros::Duration();

  pubFootLTraj.publish(marker);
}