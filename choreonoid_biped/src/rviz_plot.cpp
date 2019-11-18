/*
    author: H. Kuribayashi

    点や線などの図形をRVizに描画
 */

#include "rviz_plot.h"

RvizPlot::RvizPlot() : resolution(100), arrow_ratio(5) {
  sub_point = nh.subscribe<biped_msgs::RvPoint>(
    "rviz_plot/point", 10, &RvizPlot::setPointCallback, this);
  sub_line = nh.subscribe<biped_msgs::RvLine>(
    "rviz_plot/line", 10, &RvizPlot::setLineCallback, this);
  sub_circle = nh.subscribe<biped_msgs::RvCircle>(
    "rviz_plot/circle", 10, &RvizPlot::setCircleCallback, this);
  sub_square = nh.subscribe<biped_msgs::RvSquare>(
    "rviz_plot/square", 10, &RvizPlot::setSquareCallback, this);
  sub_arrow = nh.subscribe<biped_msgs::RvArrow>(
    "rviz_plot/arrow", 10, &RvizPlot::setArrowCallback, this);
  sub_stl = nh.subscribe<biped_msgs::RvStl>("rviz_plot/stl", 10,
                                            &RvizPlot::setStlCallback, this);

  pub = nh.advertise<visualization_msgs::MarkerArray>(
    "visualization_marker_array", 1);

  point.clear();
  line.clear();
  circle.clear();
  square.clear();
  arrow.clear();
  stl.clear();
}

RvizPlot::~RvizPlot() {
}

void RvizPlot::setPointCallback(
  const biped_msgs::RvPoint::ConstPtr &rv_point) {
  this->point[rv_point->name] = *rv_point;
  update();
}

void RvizPlot::setLineCallback(const biped_msgs::RvLine::ConstPtr &rv_line) {
  this->line[rv_line->name] = *rv_line;
  update();
}

void RvizPlot::setCircleCallback(
  const biped_msgs::RvCircle::ConstPtr &rv_circle) {
  this->circle[rv_circle->name] = *rv_circle;
  update();
}

void RvizPlot::setSquareCallback(
  const biped_msgs::RvSquare::ConstPtr &rv_square) {
  this->square[rv_square->name] = *rv_square;
  update();
}

void RvizPlot::setArrowCallback(
  const biped_msgs::RvArrow::ConstPtr &rv_arrow) {
  this->arrow[rv_arrow->name] = *rv_arrow;
  update();
}

void RvizPlot::setStlCallback(const biped_msgs::RvStl::ConstPtr &rv_stl) {
  this->stl[rv_stl->name] = *rv_stl;
  update();
}

void RvizPlot::update() {
  int num_marker = 0;
  num_marker += point.size();
  num_marker += line.size();
  num_marker += circle.size();
  num_marker += square.size();
  num_marker += arrow.size();
  num_marker += stl.size();

  // publishする描画アイテム
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(num_marker);

  // 描画対象の合計数
  // 描画対象を追加ごとにインクリメントされる
  int i = 0;

  // pointの描画
  for (auto itr = point.begin(); itr != point.end(); ++itr) {
    marker_array.markers[i].header.frame_id = "world";
    marker_array.markers[i].header.stamp    = ros::Time::now();
    marker_array.markers[i].ns              = itr->second.name;
    marker_array.markers[i].id              = i;
    marker_array.markers[i].lifetime        = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::POINTS;

    if (itr->second.action == "add")
      marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    if (itr->second.action == "delete")
      marker_array.markers[i].action = visualization_msgs::Marker::DELETE;

    marker_array.markers[i].scale.x = itr->second.size;
    marker_array.markers[i].scale.y = itr->second.size;
    marker_array.markers[i].scale.z = itr->second.size;

    marker_array.markers[i].color = itr->second.color;

    marker_array.markers[i].points.resize(itr->second.position.size() );
    for (int j = 0; j < itr->second.position.size(); j++) {
      marker_array.markers[i].points[j] = itr->second.position[j];
    }

    i++;
  }

  // lineの描画
  for (auto itr = line.begin(); itr != line.end(); ++itr) {
    marker_array.markers[i].header.frame_id = "world";
    marker_array.markers[i].header.stamp    = ros::Time::now();
    marker_array.markers[i].ns              = itr->second.name;
    marker_array.markers[i].id              = i;
    marker_array.markers[i].lifetime        = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::LINE_STRIP;

    if (itr->second.action == "add")
      marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    if (itr->second.action == "delete")
      marker_array.markers[i].action = visualization_msgs::Marker::DELETE;

    marker_array.markers[i].scale.x = itr->second.size;
    marker_array.markers[i].scale.y = itr->second.size;
    marker_array.markers[i].scale.z = itr->second.size;

    marker_array.markers[i].color = itr->second.color;

    marker_array.markers[i].points.resize(itr->second.position.size() );
    for (int j = 0; j < itr->second.position.size(); j++) {
      marker_array.markers[i].points[j] = itr->second.position[j];
    }

    i++;
  }

  // circleの描画
  for (auto itr = circle.begin(); itr != circle.end(); ++itr) {
    marker_array.markers[i].header.frame_id = "world";
    marker_array.markers[i].header.stamp    = ros::Time::now();
    marker_array.markers[i].ns              = itr->second.name;
    marker_array.markers[i].id              = i;
    marker_array.markers[i].lifetime        = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::LINE_STRIP;

    if (itr->second.action == "add")
      marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    if (itr->second.action == "delete")
      marker_array.markers[i].action = visualization_msgs::Marker::DELETE;

    marker_array.markers[i].scale.x = itr->second.size;
    marker_array.markers[i].scale.y = itr->second.size;
    marker_array.markers[i].scale.z = itr->second.size;

    marker_array.markers[i].color = itr->second.color;

    // 円を多角形に近似して描画
    // 最初と最後を結ぶため、頂点の数は resolution + 1
    marker_array.markers[i].points.resize(resolution + 1);
    double               theta = 0;
    geometry_msgs::Point vertex;
    for (int j = 0; j <= resolution; j++) {
      vertex.x                          = itr->second.position.x + itr->second.radius * cos(theta);
      vertex.y                          = itr->second.position.y + itr->second.radius * sin(theta);
      vertex.z                          = itr->second.position.z;
      marker_array.markers[i].points[j] = vertex;
      theta                            += 2 * M_PI / resolution;
    }

    i++;
  }

  // squareの描画
  for (auto itr = square.begin(); itr != square.end(); ++itr) {
    marker_array.markers[i].header.frame_id = "world";
    marker_array.markers[i].header.stamp    = ros::Time::now();
    marker_array.markers[i].ns              = itr->second.name;
    marker_array.markers[i].id              = i;
    marker_array.markers[i].lifetime        = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::LINE_STRIP;

    if (itr->second.action == "add")
      marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    if (itr->second.action == "delete")
      marker_array.markers[i].action = visualization_msgs::Marker::DELETE;

    marker_array.markers[i].scale.x = itr->second.size;
    marker_array.markers[i].scale.y = itr->second.size;
    marker_array.markers[i].scale.z = itr->second.size;

    marker_array.markers[i].color = itr->second.color;

    // 頂点４つ + 最初と最後を結ぶための余分な点１つ = ５点
    marker_array.markers[i].points.resize(5);
    geometry_msgs::Point vertex;
    vertex.z = itr->second.position.z;
    // 右上
    vertex.x                          = itr->second.position.x + itr->second.width / 2;
    vertex.y                          = itr->second.position.y + itr->second.height / 2;
    marker_array.markers[i].points[0] = vertex;
    // 左上
    vertex.x                          = itr->second.position.x - itr->second.width / 2;
    vertex.y                          = itr->second.position.y + itr->second.height / 2;
    marker_array.markers[i].points[1] = vertex;
    // 左下
    vertex.x                          = itr->second.position.x - itr->second.width / 2;
    vertex.y                          = itr->second.position.y - itr->second.height / 2;
    marker_array.markers[i].points[2] = vertex;
    // 右下
    vertex.x                          = itr->second.position.x + itr->second.width / 2;
    vertex.y                          = itr->second.position.y - itr->second.height / 2;
    marker_array.markers[i].points[3] = vertex;
    // 再度右上
    vertex.x                          = itr->second.position.x + itr->second.width / 2;
    vertex.y                          = itr->second.position.y + itr->second.height / 2;
    marker_array.markers[i].points[4] = vertex;

    i++;
  }

  // arrowの描画
  for (auto itr = arrow.begin(); itr != arrow.end(); ++itr) {
    marker_array.markers[i].header.frame_id = "world";
    marker_array.markers[i].header.stamp    = ros::Time::now();
    marker_array.markers[i].ns              = itr->second.name;
    marker_array.markers[i].id              = i;
    marker_array.markers[i].lifetime        = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::ARROW;

    if (itr->second.action == "add")
      marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    if (itr->second.action == "delete")
      marker_array.markers[i].action = visualization_msgs::Marker::DELETE;

    marker_array.markers[i].scale.x = itr->second.size; // shaft diameter
    marker_array.markers[i].scale.y =
      itr->second.size * ( 1 + arrow_ratio ); // head diameter
    marker_array.markers[i].scale.z =
      itr->second.size * arrow_ratio;   // head length

    marker_array.markers[i].color = itr->second.color;

    marker_array.markers[i].points.resize(2);
    marker_array.markers[i].points[0] = itr->second.start;
    marker_array.markers[i].points[1] = itr->second.end;

    i++;
  }

  // stlの描画
  for (auto itr = stl.begin(); itr != stl.end(); ++itr) {
    marker_array.markers[i].header.frame_id = "world";
    marker_array.markers[i].header.stamp    = ros::Time::now();
    marker_array.markers[i].ns              = itr->second.name;
    marker_array.markers[i].id              = i;
    marker_array.markers[i].lifetime        = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;

    if (itr->second.action == "add")
      marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    if (itr->second.action == "delete")
      marker_array.markers[i].action = visualization_msgs::Marker::DELETE;

    marker_array.markers[i].pose.position.x = itr->second.position.x;
    marker_array.markers[i].pose.position.y = itr->second.position.y;
    marker_array.markers[i].pose.position.z = itr->second.position.z;
    // quaternion
    marker_array.markers[i].pose.orientation.x = 0.0;
    marker_array.markers[i].pose.orientation.y = 0.0;
    marker_array.markers[i].pose.orientation.z =
      1.0 * sin(itr->second.orientation / 2.0);
    marker_array.markers[i].pose.orientation.w =
      cos(itr->second.orientation / 2.0);

    marker_array.markers[i].scale.x = 1.0;
    marker_array.markers[i].scale.y = 1.0;
    marker_array.markers[i].scale.z = 1.0;

    if (itr->second.name == "turtlebot") {
      marker_array.markers[i].mesh_resource =
        "package://robot_description/meshes/turtlebot.stl";
      marker_array.markers[i].color.r = 0.8;
      marker_array.markers[i].color.g = 0.8;
      marker_array.markers[i].color.b = 0.8;
      marker_array.markers[i].color.a = 0.5;
    }
    if (itr->second.name == "planter") {
      marker_array.markers[i].mesh_resource =
        "package://robot_description/meshes/planter.stl";
      marker_array.markers[i].color.r = 0.0;
      marker_array.markers[i].color.g = 0.8;
      marker_array.markers[i].color.b = 0.0;
      marker_array.markers[i].color.a = 1.0;
    }

    i++;
  }

  pub.publish(marker_array);
}

// main関数
// 定義したクラスのオブジェクト（ノード）を生成
int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_plot");
  RvizPlot rviz_plot;
  ros::spin();
}