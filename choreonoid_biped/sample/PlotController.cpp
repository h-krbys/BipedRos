#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <biped_msgs/RvLine.h>
#include <mutex>

using namespace std;
using namespace cnoid;

class PlotController : public SimpleController
{
  ros::NodeHandle *nh;
  ros::Publisher   pub;

public:
  virtual bool initialize(SimpleControllerIO* io) override
  {
    std::string name = "simplecontroller";
    int         argc = 0;
    ros::init(argc, NULL, name);

    // ROS Nodehandle
    nh  = new ros::NodeHandle("");
    pub = nh->advertise<biped_msgs::RvLine>("rviz_plot/line", 1);

    return true;
  }

  virtual bool control() override
  {
    biped_msgs::RvLine rv_line;

    rv_line.name    = "line1";
    rv_line.action  = "add";
    rv_line.size    = 0.02;
    rv_line.color.r = 0;
    rv_line.color.g = 1;
    rv_line.color.b = 0;
    rv_line.color.a = 1;

    geometry_msgs::Point line;
    for (int i = 0; i < 10; i++) {
      line.x = -1.0 + 0.2 * ( rand() % 10 );
      line.y = -1.0 + 0.2 * ( rand() % 10 );
      line.z = -1.0 + 0.2 * ( rand() % 10 );
      rv_line.position.push_back(line);
    }

    pub.publish(rv_line);

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PlotController)