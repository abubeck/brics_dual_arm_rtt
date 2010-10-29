#include <rtt/plugin/ServicePlugin.hpp>
#include <brics_actuator/CartesianPose.h>
#include <rtt/Port.hpp>
#include <kdl/frames.hpp>
#include "friComm.h"
#define SIGN(x) ((x)<0?-1:1)
#define STEP_OR 0.007
#define STEP_POS 0.005
using namespace RTT;
using namespace KDL;
using namespace brics_actuator;
class ApproachService : public RTT::Service {

private:

  InputPort<brics_actuator::CartesianPose> *measured_position_port;
  brics_actuator::CartesianPose measured_position;

  InputPort<brics_actuator::CartesianPose> *relative_position_port;
  brics_actuator::CartesianPose relative_position;

  Frame pose_to_frame(CartesianPose pose){
    Vector p(pose.position.x,pose.position.y,pose.position.z);
    Rotation M = Rotation::Quaternion(pose.orientation.x,pose.orientation.y,
                                      pose.orientation.z,pose.orientation.w);
    return Frame(M,p);
  }

  CartesianPose frame_to_pose(Frame frame){
    CartesianPose pose;
    pose.position.x=frame.p.x();
    pose.position.y=frame.p.y();
    pose.position.z=frame.p.z();
    frame.M.GetQuaternion(pose.orientation.x,pose.orientation.y,
                          pose.orientation.z,pose.orientation.w);
    return pose;brics_actuator::CartesianPose pose_cob;
  }
  //  struct PID_param {double Kp; double Ki; double Kd; double prev_error; double integral;};
  //  struct PID_param px = {0.1,1,0,0,0};
  //  struct PID_param py = {0.1,1,0,0,0};
  //  struct PID_param pz = {0.1,1,0,0,0};
  //  struct PID_param pwx = {0.1,1,0,0,0};
  //  struct PID_param pwy = {0.1,1,0,0,0};
  //  struct PID_param pwz = {0.1,1,0,0,0};
  //  struct PID_param pww = {0.1,1,0,0,0};
  double dt;
  double dummy_x, dummy_y, dummy_z, dummy_ox, dummy_oy, dummy_oz, dummy_ow;
  Frame fr ;
  //  struct return_struct {double desInc; double integral;};
  //  struct return_struct PID_return;

public:

  double Kp,Ki,Kd;
  double int_x,int_y,int_z,int_wx,int_wy,int_wz, int_ww, int_ox, int_oy, int_oz, int_ow;
  double prev_error_x,prev_error_y,prev_error_z,prev_error_wx,prev_error_wy,prev_error_wz,prev_error_ww;

  double goal[3];
  ApproachService(RTT::TaskContext* owner) : Service("ApproachService", owner)
  {
    this->addOperation("getNextPose", &ApproachService::getNextPose, this).doc("Returns the next point in the trajectory.");
    this->addOperation("initService", &ApproachService::initService, this).doc("Initializes the service.");
    this->addOperation("setSpeed", &ApproachService::setSpeed, this).doc("Initializes the service.");
    Kp=0.1 ;Ki=0.00000 ;Kd=0.05;
    int_x=0.0;int_y=0.0;int_z=0.0;int_wx=0.0;int_wy=0.0;int_wz=0.0; int_ww=0.0;int_ox=0.0; int_oy=0.0; int_oz=0.0; int_ow=0.0;
    prev_error_x=0.0;prev_error_y=0.0;prev_error_z=0.0;prev_error_wx=0.0;prev_error_wy=0.0;prev_error_wz=0.0;prev_error_ww=0.0;
    //this->addPort("approach_msr_port",measured_position_port).doc("Port to get the current pose");
    //    this->addPort("approach_rel_port",relative_position_port).doc("Port to get the current pose");
    dt = getOwner()->getPeriod();
    goal[0] = 0.0;
    goal[1] = 0.0;
    goal[2] = -0.295;
    goal[3] = 0.701691;
    goal[4] =  -0.0109917;
    goal[5] =  -0.0233162;
    goal[6] = 0.712015;
  }

  bool setSpeed(double speed)
  {
    return true;
  }
  void getNextPose(brics_actuator::CartesianPose* curr_next)
  {

    //    cout<<"curr: "<<curr_next->orientation.x<<","<<curr_next->orientation.y<<","<<curr_next->orientation.z<<","<<curr_next->orientation.w<<","<<endl;
    //    relative_position_port->read(relative_position);
    measured_position_port->read(measured_position);

    //    curr_next->position.x += 0.001;
    dummy_x=(measured_position.position.x  - goal[0])*Kp+Ki*int_x;
    dummy_y=(measured_position.position.y  - goal[1])*Kp+Ki*int_y;
    dummy_z=(measured_position.position.z  - goal[2])*Kp+Ki*int_z;
    dummy_ox = (-curr_next->orientation.x + goal[3])*Kp+Ki*int_ox;
    dummy_oy = (-curr_next->orientation.y + goal[4])*Kp+Ki*int_oy;
    dummy_oz = (-curr_next->orientation.z + goal[5])*Kp+Ki*int_oz;
    dummy_ow = (-curr_next->orientation.w + goal[6])*Kp+Ki*int_ow;

    if (abs(dummy_x) > STEP_POS) {
      dummy_x=SIGN(dummy_x)*STEP_POS;
    }
    if (abs(dummy_y )> STEP_POS) {
      dummy_y=SIGN(dummy_y)*STEP_POS;
    }
    if (abs(dummy_z )> STEP_POS) {
      dummy_z=SIGN(dummy_z)*STEP_POS;
    }
    if (abs(dummy_ox )> STEP_OR) {
      dummy_ox=SIGN(dummy_ox)*STEP_OR;
    }
    if (abs(dummy_oy )> STEP_OR) {
      dummy_oy=SIGN(dummy_oy)*STEP_OR;
    }
    if (abs(dummy_oz )> STEP_OR) {
      dummy_oz=SIGN(dummy_oz)*STEP_OR;
    }
    if (abs(dummy_ow )> STEP_OR) {
      dummy_ow=SIGN(dummy_ow)*STEP_OR;
    }

//    log(Error)<<sqrt(  )<<endlog();
//    cout<<sqrt( (measured_position.position.x-goal[0]) *(measured_position.position.x-goal[0])   + (measured_position.position.y-goal[1]) *(measured_position.position.y -goal[1]) + (measured_position.position.z-goal[2]) *(measured_position.position.z-goal[2]) )<<endl;

    if(sqrt((measured_position.position.x-goal[0]) *(measured_position.position.x-goal[0])   + (measured_position.position.y-goal[1]) *(measured_position.position.y -goal[1]) + (measured_position.position.z-goal[2]) *(measured_position.position.z-goal[2])) < 0.3)
    {

//      ((Property<tFriKrlData>)getOwner()->getPeer("KULRobot")->properties()->getProperty("toKRL")).value().intData[1]=3;

//      ((Property<tFriKrlData>)getPeer("KULRobot")->getProperty("toKRL")).value().intData[1]=3;
      if(this->getOwner()->hasPeer("KULRobot")){
            this->getOwner()->getPeer("KULRobot")->properties()->getPropertyType<tFriKrlData>("toKRL")->value().intData[1]=3;
      }
      log(Warning)<<"Grasp position reached."<<endlog();
    }

    if(sqrt(dummy_x*dummy_x + dummy_y*dummy_y + dummy_z*dummy_z) < 0.01)
    {

//      ((Property<tFriKrlData>)getOwner()->getPeer("KULRobot")->properties()->getProperty("toKRL")).value().intData[1]=3;

//      ((Property<tFriKrlData>)getPeer("KULRobot")->getProperty("toKRL")).value().intData[1]=3;
      if(this->getOwner()->hasPeer("KULRobot")){
            this->getOwner()->getPeer("KULRobot")->properties()->getPropertyType<tFriKrlData>("toKRL")->value().intData[1]=3;
      }
      log(Warning)<<"Grasp position reached."<<endlog();
    }




    //        cout<<dummy_x<<","<<dummy_y<<","<<dummy_z<<","<<dummy_ox<<","<<dummy_oy<<","<<dummy_oz<<endl;


    curr_next->position.x +=dummy_x; //+Kd*(measured_position.position.x - 0.02427-prev_error_x)/0.005;

    curr_next->position.y +=dummy_y; //+Kd*(measured_position.position.y - 0.02427-prev_error_y)/0.005;
    curr_next->position.z += dummy_z;//+Kd*(measured_position.position.z - 0.02427-prev_error_z)/0.005;

    curr_next->orientation.x += dummy_ox;
    curr_next->orientation.y += dummy_oy;
    curr_next->orientation.z += dummy_oz;
    curr_next->orientation.w += dummy_ow;





    int_x=int_x+dummy_x*0.005;
    int_y=int_y+dummy_y*0.005;
    int_z=int_z+dummy_z*0.005;

    int_ox=int_ox+dummy_ox*0.005;
    int_oy=int_oy+dummy_oy*0.005;
    int_oz=int_oz+dummy_oz*0.005;
    int_ow=int_ow+dummy_ow*0.005;
    prev_error_x=dummy_x;
    prev_error_y=dummy_y;
    prev_error_z=dummy_z;


    //    (*curr_next) = measured_position;

    //    PID_return=calcDesiredIncremenPID(relative_position.position.x,x,dt);
    //    next->position.x=measured_position.position.x+PID_return.desInc;
    //    x.integral=PID_return.integral;
    //    x.prev_error=relative_position.position.x;
    //
    //    PID_return=calcDesiredIncremenPID(relative_position.position.y,y,dt);
    //    next->position.y=measured_position.position.y+PID_return.desInc;
    //    y.integral=PID_return.integral;
    //    y.prev_error=relative_position.position.y;
    //
    //    PID_return=calcDesiredIncremenPID(relative_position.position.z,z,dt);
    //    next->position.z=measured_position.position.z+PID_return.desInc;
    //    z.integral=PID_return.integral;
    //    z.prev_error=relative_position.position.z;
    //
    //    PID_return=calcDesiredIncremenPID(relative_position.orientation.wx,wx,dt);
    //    next->orientation.wx=measured_position.orientation.wx+PID_return.desInc;
    //    wx.integral=PID_return.integral;
    //    wx.prev_error=relative_position.orientation.wx;
    //
    //    PID_return=calcDesiredIncremenPID(relative_position.orientation.wy,wy,dt);
    //    next->orientation.wy=measured_position.orientation.wy+PID_return.desInc;
    //    wy.integral=PID_return.integral;
    //    wy.prev_error=relative_position.orientation.wy;
    //
    //    PID_return=calcDesiredIncremenPID(relative_position.orientation.wz,wz,dt);
    //    next->orientation.wz=measured_position.orientation.wz+PID_return.desInc;
    //    wz.integral=PID_return.integral;
    //    wz.prev_error=relative_position.orientation.wz;
    //
    //    PID_return=calcDesiredIncremenPID(relative_position.orientation.ww,ww,dt);
    //    next->orientation.ww=measured_position.orientation.ww+PID_return.desInc;
    //    ww.integral=PID_return.integral;
    //    ww.prev_error=relative_position.orientation.ww;

  }

  bool initService(brics_actuator::CartesianPose iPose,InputPort<brics_actuator::CartesianPose> *msrPort)
  {
    double kp_pos = atof(getOwner()->getProperty("kp_pos")->getDataSource()->toString().c_str());
    double kd_pos = atof(getOwner()->getProperty("kd_pos")->getDataSource()->toString().c_str());
    double ki_pos = atof(getOwner()->getProperty("ki_pos")->getDataSource()->toString().c_str());

    double kp_or = atof(getOwner()->getProperty("kp_or")->getDataSource()->toString().c_str());
    double kd_or = atof(getOwner()->getProperty("kd_or")->getDataSource()->toString().c_str());
    double ki_or = atof(getOwner()->getProperty("ki_or")->getDataSource()->toString().c_str());


    //    x.Kp = kp_pos;x.Kd = kd_pos;x.Ki = ki_pos;x.prev_error = 0;x.integral = 0;
    //    y.Kp = kp_pos;y.Kd = kd_pos;y.Ki = ki_pos;y.prev_error = 0;y.integral = 0;
    //    z.Kp = kp_pos;z.Kd = kd_pos;z.Ki = ki_pos;z.prev_error = 0;z.integral = 0;
    //
    //    wx.Kp = kp_or;wx.Kd = kd_or;wx.Ki = ki_or;wx.prev_error = 0;wx.integral = 0;
    //    wy.Kp = kp_or;wy.Kd = kd_or;wy.Ki = ki_or;wy.prev_error = 0;wy.integral = 0;
    //    wz.Kp = kp_or;wz.Kd = kd_or;wz.Ki = ki_or;wz.prev_error = 0;wz.integral = 0;
    //    ww.Kp = kp_or;ww.Kd = kd_or;ww.Ki = ki_or;ww.prev_error = 0;ww.integral = 0;

    if(!msrPort->connected()){
      log(Error)<<"Measured port was not connected!"<<endlog();
      return false;
    }
    if(NoData==msrPort->read(measured_position)){
      log(Error)<<"Measured port has no value yet!"<<endlog();
      return false;
    }
    //    if(!relPort->connected()){
    //      log(Error)<<"relative port was not connected!"<<endlog();
    //      return false;
    //    }
    //    if(NoData==relPort->read(relative_position)){
    //      log(Error)<<"relative port has no value yet!"<<endlog();
    //      return false;
    //    }

    this->measured_position_port = msrPort;
    //    this->relative_position_port = relPort;
    return true;


  }



};

ORO_SERVICE_NAMED_PLUGIN(ApproachService, "ApproachService")
