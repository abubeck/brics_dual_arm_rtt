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
  InputPort<brics_actuator::CartesianPose> ipa_read_port;
  brics_actuator::CartesianPose measured_position;

  InputPort<brics_actuator::CartesianPose> relative_position_port;
  InputPort<brics_actuator::CartesianPose> kul_read_port;


  OutputPort<brics_actuator::CartesianPose> kul_command_port;

  brics_actuator::CartesianPose desired_position;

  string generatorServiceName;

  public:

  OperationCaller<void(brics_actuator::CartesianPose, InputPort<brics_actuator::CartesianPose>*)> initService;
  OperationCaller<void(brics_actuator::CartesianPose*)> getNextPose;
  OperationCaller<void(double)> setSpeedManual;

  Dual_arm_rtt(string const& name): TaskContext(name,PreOperational)
  {

    this->addProperty("generatorServiceName", generatorServiceName);
//    cout<<"Service:  "<<generatorServiceName<<endl;
//    generatorServiceName = "ApproachService";

    this->addPort("command_port",kul_command_port).doc("Port to send the desired pose");
    this->addPort("read_port",ipa_read_port).doc("Port to get the current pose");
    this->addPort("command_read_port",kul_read_port).doc("Port to get the current pose");
    this->addPort("relative_position_port",relative_position_port).doc("Port to get the current pose");
    this->addOperation("setSpeed",&Dual_arm_rtt::setSpeed,this,OwnThread).doc("change the circulation speed at runtime");

  }

  bool setSpeed(double speed)
  {
    setSpeedManual.call(speed);
    return true;
  }

  bool configureHook() {

    if(!this->provides()->hasService(generatorServiceName)){
      log(Error)<<"service   "<<generatorServiceName<<" not available"<<endlog();

      return false;
    }

    if(!this->provides()->provides(generatorServiceName)->hasOperation("initService")){
      log(Error)<<"service   does not provide operation initService"<<endlog();
      return false;
    }
    initService=this->provides()->provides(generatorServiceName)->getOperation("initService");

    if(!ipa_read_port.connected()){
      log(Error)<<"Measured port was not connected!"<<endlog();
      return false;
    }
    if(NoData==ipa_read_port.read(measured_position)){
      log(Error)<<"Measured port has not value yet!"<<endlog();
      return false;
    }

    if(!relative_position_port.connected()){
      log(Error)<<"Relative port was not connected!"<<endlog();
//      return false;
    }


    ipa_read_port.read(measured_position);

    initService.call(measured_position, &relative_position_port);


    if(!this->provides()->provides(generatorServiceName)->hasOperation("getNextPose")){
      log(Error)<<"service   does not provide operation getNextPose"<<endlog();
      return false;
    }
    getNextPose=this->provides()->provides(generatorServiceName)->getOperation("getNextPose");

    if(!this->provides()->provides(generatorServiceName)->hasOperation("setSpeed")){
      log(Error)<<"service   does not provide operation setSpeed"<<endlog();
      return false;
    }
    setSpeedManual=this->provides()->provides(generatorServiceName)->getOperation("setSpeed");



    return true;
  }

  bool startHook() {

    if(this->getPeriod()<=0){
      log(Error)<<"No period, we want to be executed periodically!!!"<<endlog();
      return false;
    }

    kul_read_port.read(desired_position);

    return true;

  }

  void updateHook() {

    kul_read_port.read(desired_position);
//    cout<<"**********************************\n";
//    cout<<"Current: "<<desired_position.position.x<<","<<desired_position.position.y<<desired_position.position.z<<endl;
    getNextPose.call(&desired_position);

//    cout<<"Commanded: "<<desired_position.position.x<<","<<desired_position.position.y<<desired_position.position.z<<endl;
//    cout<<"**********************************\n";
    kul_command_port.write(desired_position);


//    getchar();
//    cout<<desired_position.position.x<<endl;
//        log(Debug)<<desired_position.position.x<<endlog();
  }

  void stopHook() {


    if(!kul_read_port.connected()){
      log(Error)<<"Measured port was not connected!"<<endlog();

    }
    if(NoData==kul_read_port.read(measured_position)){
      log(Error)<<"Measured port has not value yet!"<<endlog();

    }
    kul_command_port.write(measured_position);

    std::cout << "Dual_arm_rtt executes stopping !" <<std::endl;
  }

  void cleanupHook() {
    if(!kul_read_port.connected()){
      log(Error)<<"Measured port was not connected!"<<endlog();

    }
    if(NoData==kul_read_port.read(measured_position)){
      log(Error)<<"Measured port has not value yet!"<<endlog();

    }
    kul_command_port.write(measured_position);
    std::cout << "Dual_arm_rtt cleaning up !" <<std::endl;
  }
  };

#endif
