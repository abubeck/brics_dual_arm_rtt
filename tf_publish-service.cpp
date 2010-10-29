#include <rtt/plugin/ServicePlugin.hpp>

class TranformPublishExecutor : RTT::ExecutableInterface{
public:
  TransformPublishExecutor(RTT::Operation& operation){
    op_call=operation;
  private:
  RTT::OperationCaller op_call;
  

  bool execute(){
    operation()

class FRITransformPublisher : public RTT::Service {
public:
    PointGeneratorService(RTT::TaskContext* owner) : Service("FRITransformPublisher", owner) 
    {
      this->addOperation("publish", &::tf_publish, this).doc("Returns the name of the owner of this object.");
      Transformpublisherservice exec()
    }

    string getOwnerName() {
        // getOwner() returns the TaskContext pointer we got in
        // the constructor:
        return getOwner()->getName();
    }
};

ORO_SERVICE_NAMED_PLUGIN(PointGeneratorService, "PointGeneratorService")

/*
   #include <rtt/TaskContext.hpp>
  #include <iostream>

  class MyServer : public RTT::TaskContext {
  public:
    MyServer() : TaskContext("server") {
       this->provides("display")
             ->addOperation("showErrorMsg", &MyServer::showErrorMsg, this, RTT::OwnThread)
                   .doc("Shows an error on the display.")
                   .arg("code", "The error code")
                   .arg("msg","An error message");
       this->provides("display")
             ->addOperation("clearErrors", &MyServer::clearErrors, this, RTT::OwnThread)
                   .doc("Clears any error on the display.");
    }
    void showErrorMsg(int code, std::string msg) {
       std::cout << "Code: "<<code<<" - Message: "<< msg <<std::endl;
    }
    void clearErrors() {
       std::cout << "No errors present." << std::endl;
    }
  };
ORO_SERVICE_NAMED_PLUGIN(MyServer, "MyServer")*/

