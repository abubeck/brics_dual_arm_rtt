#include <rtt/plugin/ServicePlugin.hpp>
#include <brics_actuator/CartesianPose.h>

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
    this->addOperation("getNextPose", &CircleGeneratorService::getNextPose, this).doc("Returns the name of the owner of this object.");
    this->addOperation("initService", &CircleGeneratorService::initService, this).doc("Returns the name of the owner of this object.");

  }

  void getNextPose(brics_actuator::CartesianPose* desPose)
  {
    theta += speed*period;
    desPose->position.x = centerx + radius * sin(M_PI / 180.0 * theta);
    desPose->position.y = centery + radius * cos(M_PI / 180.0 * theta);
    desPose->position.z = centerz;
    desPose->orientation=initPose.orientation;

  }

  void initService(brics_actuator::CartesianPose iPose)
  {
    initPose = iPose;
    speed = atof(getOwner()->getProperty("speed")->getDataSource()->toString().c_str());
    radius = atof(getOwner()->getProperty("radius")->getDataSource()->toString().c_str());
    centerx = iPose.position.x;
    centery = iPose.position.y-radius;
    centerz = iPose.position.z;
    period = getOwner()->getPeriod();
    theta=0.0;

    cout<<radius<<endl;
    cout<<speed<<endl;

  }




};

ORO_SERVICE_NAMED_PLUGIN(CircleGeneratorService, "CircleGeneratorService")
