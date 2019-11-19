// #include "StatePublisherItem.h"
#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/spinner.h>

using namespace std;
using namespace cnoid;

class RvizPlugin : public Plugin
{
  std::unique_ptr<ros::AsyncSpinner> spinner;

public:
  RvizPlugin() : Plugin("Rviz") {
    require("Body");
  }

  virtual bool initialize()
  {
    int    argc = 0;
    char** argv = 0;

    if(!ros::isInitialized() ) {
      ros::init(argc, argv, "choreonoid", ros::init_options::NoSigintHandler);
    }

    if(!ros::master::check() ) {
      MessageView::instance()->putln(
        MessageView::WARNING, "The ROS master is not found.");
      return false;
    }

    spinner.reset(new ros::AsyncSpinner(0) );
    spinner->start();

    // StatePublisherItem::initialize(this);

    return true;
  }

  virtual bool finalize()
  {
    ros::requestShutdown();
    ros::waitForShutdown();
  }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(RvizPlugin)