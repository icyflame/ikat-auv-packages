#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ikat_sensor_data/depth_sensor_data.h>
#include <ikat_sensor_data/mt9_sensor_data.h>
#include <ikat_sensor_data/control_data.h>
#include <ikat_thrusters/Thrusters.hpp>

#define CH0 0x00
#define CH1 0x01
#define CH2 0x02
#define CH3 0x03

#define SurgeLeftThruster	CH0
#define SurgeRightThruster	CH2
#define DepthRightThruster	CH3
#define DepthLeftThruster	CH1

// Thrusters definition
using namespace std;
using namespace ikat_hardware;
class controller
{
 public:
    /////////////////////////////////////////////////////
    ///Depth Sensor Variables
    /////////////////////////////////////////////////////
    float depth;
    float steady_depth;
    float KP_DEPTH,KI_DEPTH,error_depth,sum_depth,prev_error_depth,diff_depth;
    int   no_of_times_from_begining_for_depth_sensor;
    float DEPTH_AT_SURFACE;
    /////////////////////////////////////////////////////

    /////////////////////////////////////////////////////
    ///Mt9 variables
    /////////////////////////////////////////////////////
    float roll_mt9,pitch_mt9, yaw_mt9;
    float steady_angle;
    float KP_YAW,KI_YAW,KD_YAW,error_yaw,sum_yaw,prev_error_yaw,diff_yaw;
    float theta_relative;
    int   no_of_times_from_begining_for_mt9;
    /////////////////////////////////////////////////////

    ////////////////////////////////////////////////////
    ///thrusters variables
    ////////////////////////////////////////////////////
    float horizontal_speed,vertical_speed,differential_surge_speed,thruster_surge_left,thruster_surge_right;

    //////////////////////////////////////////////////////
    int is_image;
    float x_coordinate_of_image,y_coordinate_of_image,KP_IMAGE,KI_IMAGE,KD_IMAGE;

    //////////////////////////////////////////////////////
    /////member functions
    //////////////////////////////////////////////////////
    controller();
    void yawController();
    void yawController(float x_coordinate,float y_coordinate,float KP_YAW_IMAGE,float KI_YAW_IMAGE,float KD_YAW_IMAGE);
    void speedCallibration();
    void depthController();
    void depthController(float x_coordinate,float y_coordinate,float KP_DEPTH_IMAGE,float KI_DEPTH_IMAGE,float KD_DEPTH_IMAGE);
} obj;

controller::controller()
{
    /////////////////////////////////////////////////////
    ///Depth Sensor Variables
    /////////////////////////////////////////////////////
    depth=0;
    steady_depth =0;
    KP_DEPTH=5.5,KI_DEPTH=.3,error_depth=0,sum_depth=0,prev_error_depth=0,diff_depth=0;
    no_of_times_from_begining_for_depth_sensor=0;
    DEPTH_AT_SURFACE=0;
    /////////////////////////////////////////////////////

    /////////////////////////////////////////////////////
    ///Mt9 variables
    /////////////////////////////////////////////////////
    roll_mt9=0,pitch_mt9=0, yaw_mt9=0;
    steady_angle =0;
    KP_YAW=0.14,KI_YAW=0.00,KD_YAW=.1,error_yaw=0,sum_yaw=0,prev_error_yaw=0.0,diff_yaw=0.0;
    theta_relative=0;
    no_of_times_from_begining_for_mt9=0;
    ////////////////////////////////////////////////////
    is_image=0,x_coordinate_of_image=0,y_coordinate_of_image=0,KP_IMAGE=0,KI_IMAGE,KD_IMAGE=0;
    ////////////////////////////////////////////////////
    ///thrusters variables
    ////////////////////////////////////////////////////
    horizontal_speed=0,vertical_speed=0,differential_surge_speed=0,thruster_surge_left=0,thruster_surge_right=0;
}

void depthCallback(const ikat_sensor_data::depth_sensor_data::ConstPtr& msg)
{

  if (obj.no_of_times_from_begining_for_depth_sensor==5)
  {
       obj.DEPTH_AT_SURFACE=obj.depth;
  }
  if(obj.no_of_times_from_begining_for_depth_sensor<10)
        obj.no_of_times_from_begining_for_depth_sensor++;

  obj.depth = msg->depth-obj.DEPTH_AT_SURFACE;

  ROS_INFO("depth: [%f]",obj.depth);

}
void mt9Callback(const ikat_sensor_data::mt9_sensor_data::ConstPtr& msg)
{
    obj.roll_mt9=msg->MT9_data[0];
    obj.pitch_mt9=msg->MT9_data[1];
    obj.yaw_mt9=msg->MT9_data[2];
    if (obj.yaw_mt9>0 && obj.no_of_times_from_begining_for_mt9==5)
    {
         obj.theta_relative=obj.yaw_mt9;
    }
    obj.yaw_mt9=obj.yaw_mt9-obj.theta_relative+180;
    if(obj.no_of_times_from_begining_for_mt9<20)
          obj.no_of_times_from_begining_for_mt9++;
    if (obj.yaw_mt9<0)
    {
         obj.yaw_mt9+=360;
    }
    if (obj.yaw_mt9>360)
    {
         obj.yaw_mt9-=360;
    }
    ROS_INFO("Roll [%f]      ",obj.roll_mt9);
    ROS_INFO("Pitch [%f]     ",obj.pitch_mt9);
    ROS_INFO("Yaw [%f]       ",obj.yaw_mt9);
}
void controllSignalCallback(const ikat_sensor_data::control_data::ConstPtr& msg)
{
    obj.steady_angle=msg->angle;
    obj.steady_depth=msg->depth;
    obj.is_image=msg->use_image;
    obj.x_coordinate_of_image=msg->X;
    obj.y_coordinate_of_image=msg->Y;
    obj.KP_IMAGE=msg->Kp;
    obj.KI_IMAGE=msg->Ki;
    obj.KD_IMAGE=msg->Kd;
}

void controller::yawController()
{
    if(no_of_times_from_begining_for_mt9<=5)
        steady_angle=yaw_mt9;


    if(steady_angle>360)
        steady_angle-=360;
    if(steady_angle<0)
        steady_angle+=360;

    error_yaw=steady_angle-yaw_mt9;
    sum_yaw+=error_yaw;
    diff_yaw=error_yaw-prev_error_yaw;
    prev_error_yaw=error_yaw;
    differential_surge_speed=KP_YAW*error_yaw+KI_YAW*sum_yaw+KD_YAW*diff_yaw;
    thruster_surge_left=-differential_surge_speed+horizontal_speed;
    thruster_surge_right= differential_surge_speed+horizontal_speed;
    speedCallibration();
}

void controller::yawController( float x_coordinate,float y_coordinate,float KP_YAW_IMAGE,float KI_YAW_IMAGE,float KD_YAW_IMAGE )
{
    error_yaw=x_coordinate;
    sum_yaw+=error_yaw;
    diff_yaw=error_yaw-prev_error_yaw;
    prev_error_yaw=error_yaw;
    differential_surge_speed=KP_YAW_IMAGE*error_yaw + KI_YAW_IMAGE*sum_yaw + KD_YAW_IMAGE*diff_yaw;
    thruster_surge_left=-differential_surge_speed+horizontal_speed;
    thruster_surge_right= differential_surge_speed+horizontal_speed;
    //cout<<"ERROR:"<<x_coordinate<<endl;
    speedCallibration();
}

void controller::depthController()
{
    error_depth=-depth+steady_depth;
    sum_depth=sum_depth+error_depth;
    vertical_speed=KP_DEPTH*error_depth+KI_DEPTH*sum_depth+3;
}
void controller::depthController(float x_coordinate,float y_coordinate,float KP_DEPTH_IMAGE,float KI_DEPTH_IMAGE,float KD_DEPTH_IMAGE)
{
    error_depth=y_coordinate;
    sum_depth=sum_depth+error_depth;
    diff_depth=error_depth-prev_error_depth;
    prev_error_depth=error_depth;
    vertical_speed=KP_DEPTH_IMAGE*error_depth+KI_DEPTH_IMAGE*sum_depth+KD_DEPTH_IMAGE*diff_depth+3;
}

void controller::speedCallibration()
{
    ///////////when still
    if(thruster_surge_left<0 && horizontal_speed==0)
    {
            thruster_surge_left-=2.1;
    }
    if(thruster_surge_right<0 && horizontal_speed==0)
    {
            thruster_surge_right-=2.1;
    }
    if(thruster_surge_left>0 && horizontal_speed==0)
    {
            thruster_surge_left+=2.1;
    }
    if(thruster_surge_right>0 && horizontal_speed==0)
    {
            thruster_surge_right+=2.1;
    }
    //////////when moving forward
    if(thruster_surge_left<2.2 && horizontal_speed>0)
    {
            thruster_surge_left-=4.4;
    }
    if(thruster_surge_right<2.2 && horizontal_speed>0)
    {
            thruster_surge_right-=4.4;
    }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "controller");


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber depth_callback = n.subscribe("current_depth", 10, depthCallback);
  ros::Subscriber mt9_callback =   n.subscribe("Orientation_data_from_MT9", 10, mt9Callback);
  ros::Subscriber task_planner_callback=n.subscribe("control_signal",10, controllSignalCallback);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::Rate loopRate(1);
  std::string port_name = "/dev/ttylUSB0";
  ikat_hardware::Thruster th(port_name,9600);
  //thruster th("/dev/ttylUSB0",9600);
  //if(th.startThrusters())
  //   cout<<"The serial port is not connected\n";

  /////////starting depth/////////////////
  if(!th.connectThrusters())
      cout<<"thrusters not connected\n"<<endl;
  obj.vertical_speed=4.0;
  th.sendData(obj.vertical_speed,DepthRightThruster);
  th.sendData(obj.vertical_speed,DepthLeftThruster);
  sleep(1);
  obj.vertical_speed=0;
  ///////////////////////////////////////

  while(ros::ok())
  {
      ros::spinOnce();
      if(!obj.is_image)
      {
        obj.yawController();
        obj.depthController();
      }
      else
      {
          obj.yawController(obj.x_coordinate_of_image,obj.y_coordinate_of_image,obj.KP_IMAGE,obj.KI_IMAGE,obj.KD_IMAGE);
          obj.depthController();
      }

      ///////////////////////////////////////////////
      //////sending information to thrusters
      ///////////////////////////////////////////////
      th.sendData(obj.thruster_surge_left,SurgeLeftThruster);
      th.sendData(obj.thruster_surge_right,SurgeRightThruster);
      th.sendData(obj.vertical_speed,DepthRightThruster);
      th.sendData(obj.vertical_speed,DepthLeftThruster);

      loopRate.sleep();

  }
  return 0;
}
