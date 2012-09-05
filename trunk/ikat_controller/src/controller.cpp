#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ikat_sensor_data/depth_sensor_data.h>
#include <ikat_sensor_data/mt9_sensor_data.h>
#include <ikat_controller/Technadyne.h>


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
class controller
{

    /////////////////////////////////////////////////////
    ///Depth Sensor Variables
    /////////////////////////////////////////////////////
    float depth=0,threshold=4.0;
    float steady_depth =0;
    float KP_DEPTH=5.5,KI_DEPTH=.3,error_depth=0,sum_depth=0;
    int   no_of_times_from_begining_for_depth_sensor=0;
    float DEPTH_AT_SURFACE=0,vertical_speed=0;
    /////////////////////////////////////////////////////

    /////////////////////////////////////////////////////
    ///Mt9 variables
    /////////////////////////////////////////////////////
    float roll_mt9=0,pitch_mt9=0, yaw_mt9=0;
    float steady_angle =0;
    float KP_YAW=0.14,KI_YAW=0.00,KD_YAW=.1,error_yaw=0,sum_yaw=0,prev_error_yaw=0.0,diff_yaw=0.0;
    float horizontal_speed=0,theta_relative=0,differential_speed;
    int   no_of_times_from_begining_for_mt9=0;
    //////////////////////////////////////////////////////
public:
    void depthCallback(const ikat_sensor_data::depth_sensor_data::ConstPtr &msg);
    void mt9Callback(const ikat_sensor_data::mt9_sensor_data::ConstPtr &msg);
    void yawController( float steady_angle,float KP_YAW,float KI_YAW,float KD_YAW );
    void depthController();

};


void controller::depthCallback(const ikat_sensor_data::depth_sensor_data::ConstPtr& msg)
{

  if (no_of_times_from_begining_for_depth_sensor==5)
  {
       DEPTH_AT_SURFACE=depth;
  }
  if(no_of_times_from_begining_for_depth_sensor<10)
        no_of_times_from_begining_for_depth_sensor++;

  depth = msg->depth-DEPTH_AT_SURFACE;

  ROS_INFO("depth: [%f]",depth);

}
void controller::mt9Callback(const ikat_sensor_data::mt9_sensor_data::ConstPtr& msg)
{
    roll_mt9=msg->MT9_data[0];
    pitch_mt9=msg->MT9_data[1];
    yaw_mt9=msg->MT9_data[2];
    if (yaw_mt9>0 && no_of_times_from_begining_for_mt9==5)
    {
         theta_relative=yaw_mt9;
    }
    yaw_mt9=yaw_mt9-theta_relative+180;
    if(no_of_times_from_begining_for_mt9<20)
          no_of_times_from_begining_for_mt9++;
    if (yaw_mt9<0)
    {
         yaw_mt9+=360;
    }
    if (yaw_mt9>360)
    {
         yaw_mt9-=360;
    }
    ROS_INFO("Roll [%f]      ",roll_mt9);
    ROS_INFO("Pitch [%f]     ",pitch_mt9);
    ROS_INFO("Yaw [%f]       ",yaw_mt9);
}
void controller::yawController( float steady_angle,float KP_YAW,float KI_YAW,float KD_YAW )
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
    differential_speed=KP_YAW*error_yaw+KI_YAW*sum_yaw+KD_YAW*diff_yaw;

}
void controller::depthController()
{

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
  controller obj;
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
  ros::Subscriber depth = n.subscribe("current_depth", 10, depthCallback);
  ros::Subscriber mt9 =   n.subscribe("Orientation_data_from_MT9", 100, mt9Callback);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::Rate loopRate(1);
  thruster th("/dev/ttylUSB0",9600);
  if(th.startThrusters())
      cout<<"The serial port is not connected\n";
  /////////starting depth/////////////////
  vertical_speed=4.0;
  th.sendCommand(vertical_speed,DepthRightThruster);
  th.sendCommand(vertical_speed,DepthLeftThruster);
  sleep(1);
  vertical_speed=0;
  /////////////////////

  while(ros::ok())
  {
      ros::spinOnce();

      obj.yawController(steady_angle,KP_YAW,KD_YAW,KI_YAW);
      obj.depthController();




    if(th.sendCommand(2.5,SurgeLeftThruster))
        cout<<"Thruster communication was successful\n";

    sleep(30);//delay

    if(th.sendCommand(0,SurgeLeftThruster))
        cout<<"Thruster stopped successfully \n";

    loopRate.sleep();

  }
  return 0;
}
