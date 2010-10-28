#include <rtt/plugin/ServicePlugin.hpp>
#include <brics_actuator/CartesianPose.h>
#include <rtt/Port.hpp>

using namespace RTT;

class CircleGeneratorService : public RTT::Service {

private:
  double speed;
  double period;
  double radius;
  double centerx, centery, centerz;
  double theta;

  brics_actuator::CartesianPose initPose;

public:
  CircleGeneratorService(RTT::TaskContext* owner) : Service("CircleGeneratorService", owner)
  {
    this->addOperation("getNextPose", &CircleGeneratorService::getNextPose, this).doc("Returns the next point in the trajectory.");
    this->addOperation("initService", &CircleGeneratorService::initService, this).doc("Initializes the service.");
    this->addOperation("setSpeed", &CircleGeneratorService::setSpeed, this).doc("Sets the speed of trajectory generation.");
  }

  void setSpeed(double speed)
  {
    this->speed=speed;
  }
  void getNextPose(brics_actuator::CartesianPose* curr_next)
  {
    theta += speed*period;
    curr_next->position.x = centerx + radius * sin(M_PI / 180.0 * theta);
    curr_next->position.y = centery + radius * cos(M_PI / 180.0 * theta);
    curr_next->position.z = centerz;
    curr_next->orientation=initPose.orientation;

  }

  void initService(brics_actuator::CartesianPose iPose,InputPort<brics_actuator::CartesianPose> *msrPort )
  {
    initPose = iPose;
    speed = atof(getOwner()->getProperty("speed")->getDataSource()->toString().c_str());
    radius = atof(getOwner()->getProperty("radius")->getDataSource()->toString().c_str());
    centerx = iPose.position.x;
    centery = iPose.position.y-radius;
    centerz = iPose.position.z;
    period = getOwner()->getPeriod();
    theta=0.0;

    cout<<"Service <CircleGenerator> initialized with property <radius>="<<radius<<endl;
    cout<<"Service <CircleGenerator> initialized with property <speed>="<<speed<<endl;

  }




};

ORO_SERVICE_NAMED_PLUGIN(CircleGeneratorService, "CircleGeneratorService")
