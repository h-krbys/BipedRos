#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include "../include/RvizPublisher.h"
#include <mutex>

using namespace std;
using namespace cnoid;

const double pgain = 10;
const double dgain = 0.0;

class PlotController : public SimpleController
{
  BodyPtr             ioBody;
  double              t, dt;
  std::vector<double> qref;
  std::vector<double> qold;

  ros::NodeHandle     *nh;
  ros::Publisher       pubCop, pubSim;
  geometry_msgs::Point point;
  std_msgs::Float64    data;

public:
  virtual bool initialize(SimpleControllerIO* io) override
  {
    ioBody = io->body();
    dt     = io->timeStep();

    for(int i = 0; i < ioBody->numJoints(); ++i) {
      Link* joint = ioBody->joint(i);
      joint->setActuationMode(Link::JOINT_VELOCITY);
      io->enableIO(joint);
      qref.push_back(joint->q() );
    }
    qold = qref;

    int    argc = 0;
    char** argv = 0;
    ros::init(argc, argv, "simplecontroller");
    nh = new ros::NodeHandle("");

    pubCop = nh->advertise<geometry_msgs::Point>("/simulation/cop", 1000);
    pubSim = nh->advertise<std_msgs::Float64>("/simulation/time", 1000);

    t = 0.0;

    return true;
  }

  virtual bool control() override
  {
    if(t < 0.5) {
      point.x = 1.0;
      point.y = 1.0;
      point.z = 1.0;
      pubCop.publish(point);

      data.data = t;
      printf("A\n");
      pubSim.publish(data);
      printf("B\n");

      for(int i = 0; i < ioBody->numJoints(); ++i) {
        Link * joint = ioBody->joint(i);
        double q     = joint->q();
        double dq    = ( q - qold[i] ) / dt;
        joint->dq_target() = ( qref[i] - q ) * pgain + ( 0.0 - dq ) * dgain;
        qold[i]            = q;
      }
      printf("%lf\n", t);
    }

    t += dt;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PlotController)