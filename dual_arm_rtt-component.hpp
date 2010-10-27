#ifndef OROCOS_DUAL_ARM_RTT_COMPONENT_HPP
#define OROCOS_DUAL_ARM_RTT_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <brics_actuator/CartesianPose.h>

#include <iostream>

using namespace RTT;

class Dual_arm_rtt
: public RTT::TaskContext
  {

private:
  InputPort<brics_actuator::CartesianPose> measured_position_port;
    brics_actuator::CartesianPose measured_position;

    OutputPort<brics_actuator::CartesianPose> desired_position_port;
    brics_actuator::CartesianPose desired_position;


  public:

  OperationCaller<void(brics_actuator::CartesianPose)> initService;
  OperationCaller<void(brics_actuator::CartesianPose*)> getNextPose;

  Dual_arm_rtt(string const& name): TaskContext(name,PreOperational)
  {

    this->addPort("measured_position",measured_position_port).doc("Port to get the current pose");
    this->addPort("desired_position",desired_position_port).doc("Port to send the desired pose");

//    this->addOperation("setSpeed",&Dual_arm_rtt::setSpeed,this,OwnThread).doc("change the circulation speed at runtime");

  }


  bool configureHook() {

    if(!this->provides()->hasService("CircleGeneratorService")){
      log(Error)<<"service CircleGeneratorService not available"<<endlog();
      return false;
    }

    if(!this->provides()->provides("CircleGeneratorService")->hasOperation("initService")){
          log(Error)<<"service CircleGeneratorService does not provide operation initService"<<endlog();
          return false;
        }
    initService=this->provides()->provides("CircleGeneratorService")->getOperation("initService");

    if(!measured_position_port.connected()){
      log(Error)<<"Measured port was not connected!"<<endlog();
      return false;
    }
    if(NoData==measured_position_port.read(measured_position)){
      log(Error)<<"Measured port has not value yet!"<<endlog();
      return false;
    }
    measured_position_port.read(measured_position);

    initService.call(measured_position);


    if(!this->provides()->provides("CircleGeneratorService")->hasOperation("getNextPose")){
      log(Error)<<"service CircleGeneratorService does not provide operation getNextPose"<<endlog();
      return false;
    }
    getNextPose=this->provides()->provides("CircleGeneratorService")->getOperation("getNextPose");



    return true;
  }

  bool startHook() {

    if(this->getPeriod()<=0){
      log(Error)<<"No period, we want to be executed periodically!!!"<<endlog();
      return false;
    }

    return true;

  }

  void updateHook() {


    getNextPose.call(&desired_position);
    desired_position_port.write(desired_position);
    log(Debug)<<desired_position.position.x<<endlog();
  }

  void stopHook() {

    if(!measured_position_port.connected()){
      log(Error)<<"Measured port was not connected!"<<endlog();

    }
    if(NoData==measured_position_port.read(measured_position)){
      log(Error)<<"Measured port has not value yet!"<<endlog();

    }
    desired_position_port.write(measured_position);

    std::cout << "Dual_arm_rtt executes stopping !" <<std::endl;
  }

  void cleanupHook() {
    if(!measured_position_port.connected()){
      log(Error)<<"Measured port was not connected!"<<endlog();

    }
    if(NoData==measured_position_port.read(measured_position)){
      log(Error)<<"Measured port has not value yet!"<<endlog();

    }
    desired_position_port.write(measured_position);
    std::cout << "Dual_arm_rtt cleaning up !" <<std::endl;
  }
  };

#endif
