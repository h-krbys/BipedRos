/*
    author: H. Kuribayashi

    点や線などの図形をRVizに描画
 */

#ifndef __RVIZ_PLOT_H__
#define __RVIZ_PLOT_H__

// ヘッダファイルの読み込み
// C言語系
#include <cmath>
#include <map>
#include <vector> // 動的配列
// ROS系
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>      // Rvizに点や線を描画するメッセージ型
#include <visualization_msgs/MarkerArray.h> // 配列版
// 自分で定義したメッセージ型
#include <biped_msgs/RvArrow.h>  // 矢印描画用
#include <biped_msgs/RvCircle.h> // 円形描画用
#include <biped_msgs/RvLine.h>   // 線描画用
#include <biped_msgs/RvPoint.h>  // 点描画用
#include <biped_msgs/RvSquare.h> // 四角形描画用
#include <biped_msgs/RvStl.h>    // stlモデル描画用

// 描画クラス
class RvizPlot {
public:
  RvizPlot();
  ~RvizPlot();

  // 描画対象が登録された時に呼ばれるコールバック関数
  // 描画アイテムごとに１つずつ
  void setPointCallback(const biped_msgs::RvPoint::ConstPtr &rv_point);
  void setLineCallback(const biped_msgs::RvLine::ConstPtr &rv_line);
  void setCircleCallback(const biped_msgs::RvCircle::ConstPtr &rv_circle);
  void setSquareCallback(const biped_msgs::RvSquare::ConstPtr &rv_square);
  void setArrowCallback(const biped_msgs::RvArrow::ConstPtr &rv_arrow);
  void setStlCallback(const biped_msgs::RvStl::ConstPtr &rv_stl);

private:
  // 現在登録されている描画対象をRvizに描画
  void update();

  // ROSシステムの変数
  ros::NodeHandle nh;
  ros::Subscriber sub_point, sub_line, sub_circle;
  ros::Subscriber sub_square, sub_arrow, sub_stl;
  ros::Publisher  pub;

  // 描画対象の情報を保存しておく変数
  std::map<std::string, biped_msgs::RvPoint>  point;
  std::map<std::string, biped_msgs::RvLine>   line;
  std::map<std::string, biped_msgs::RvCircle> circle;
  std::map<std::string, biped_msgs::RvSquare> square;
  std::map<std::string, biped_msgs::RvArrow>  arrow;
  std::map<std::string, biped_msgs::RvStl>    stl;

  // 円の直接的な描画はサポートされていないので
  // 円を正N角形に近似して描画するときのNの値
  const int resolution;

  // 矢印のサイズに対する矢印部の比
  const double arrow_ratio;
};

#endif // __RVIZ_PLOT_H__