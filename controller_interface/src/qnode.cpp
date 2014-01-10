/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/controller_interface/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace controller_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
        {
    ros::init(init_argc,init_argv,"controllerinterface");
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {

	if ( ! ros::master::check() ) {
            std::cout<<"False"<<std::endl;
		return false;
	}
        std::cout<<"Connected"<<std::endl;
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
        no_of_times_from_begining_for_depth_sensor=0;
        no_of_times_from_begining_for_mt9 = 0;
        steady_angle = 180;
        steady_depth = 0 ;
        free_thrusters = true;
        resetDepthController();
        resetYawController();
        resetSpeedController();
        depth_callback = n.subscribe("current_depth", 10, &QNode::depthCallback,this);
        mt9_callback =   n.subscribe("Orientation_data_from_MT9", 10, &QNode::mt9Callback,this);
        thrusterPub = n.advertise<ikat_sensor_data::thruster_data>("thrusterControlData",3);
	start();
        stopThruster();
	return true;
}


void QNode::run() {
    ros::Rate loop_rate(5);
    float buff[2];
    int count=0;
	while ( ros::ok() ) 
    {
        if(runThrusters)
        {
            thrusterData.heaveThrust=depthController();
            yawController(buff);
            thrusterData.surgeLeft = buff[0];
            thrusterData.surgeRight = buff[1];
            thrusterPub.publish(thrusterData);
            free_thrusters = true;
        }
        else
        {
            if(free_thrusters)
            {
                thrusterData.heaveThrust=0;
                thrusterData.surgeLeft = 0;
                thrusterData.surgeRight =0;
                thrusterPub.publish(thrusterData);
                free_thrusters = false;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
        count++;
        ROS_INFO("[%f] [%f]",error_yaw,error_depth);
        }
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::stopThruster()
{
    runThrusters = false;
    thrusterData.heaveThrust=0;
    thrusterData.surgeLeft = 0;
    thrusterData.surgeRight =0;
    thrusterPub.publish(thrusterData);
}

void QNode::testThrusters()
{
    if(runThrusters)
    {
    }
    else
    {
        while(free_thrusters);
        std::cout<<"testing thruster"<<std::endl;
        thrusterData.heaveThrust=2.5;
        thrusterData.surgeLeft = 0;
        thrusterData.surgeRight = 0;
        thrusterPub.publish(thrusterData);
        sleep(1);
        thrusterData.heaveThrust=0;
        thrusterData.surgeLeft = 2.5;
        thrusterData.surgeRight = 0;
        thrusterPub.publish(thrusterData);
        sleep(1);
        thrusterData.heaveThrust=0;
        thrusterData.surgeLeft = 0;
        thrusterData.surgeRight = 2.5;
        thrusterPub.publish(thrusterData);
        sleep(1);
        thrusterData.heaveThrust=0;
        thrusterData.surgeLeft = 0;
        thrusterData.surgeRight =0;
        thrusterPub.publish(thrusterData);
        sleep(1);
    }
}

void QNode::resetSpeedController()
{
    speed_controller = false;
    horizontal_speed = 0;
}

void QNode::resetDepthController()
{
    error_depth=0;
    sum_depth=0;
    diff_depth = 0;
    prev_error_depth = 0;
    depth_controller = false;
}

void QNode::resetYawController()
{
    error_yaw=0;
    sum_yaw=0;
    diff_yaw=0;
    prev_error_yaw=0;
    differential_surge_speed=0;
    thruster_surge_left=0;
    thruster_surge_right=0;
    yaw_controller = false;
}

float QNode::depthController()
{
    if(depth_controller)
    {
        error_depth=steady_depth-depth;
        sum_depth=sum_depth+error_depth;
        diff_depth = error_depth - prev_error_depth;
        prev_error_depth = error_depth;
        return (KP_DEPTH*error_depth+KI_DEPTH*sum_depth+KD_DEPTH*diff_depth+4.1);
    }
    else
    {
        return 0;
    }
}

void QNode::yawController(float * buff)
{
    if(yaw_controller)
    {
        error_yaw=steady_angle-yaw_mt9;
        sum_yaw+=error_yaw;
        diff_yaw=error_yaw-prev_error_yaw;
        prev_error_yaw=error_yaw;
        differential_surge_speed=KP_YAW*error_yaw+KI_YAW*sum_yaw+KD_YAW*diff_yaw;
        thruster_surge_left=-differential_surge_speed+horizontal_speed;
        thruster_surge_right= differential_surge_speed+horizontal_speed;
        speedCallibration();
        buff[0]=thruster_surge_left;
        buff[1]=thruster_surge_right;
    }
    else
    {
        buff[0]=0;
        buff[1]=0;
    }
}

void QNode::speedCallibration()
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

void QNode::depthCallback(const ikat_sensor_data::depth_sensor_data::ConstPtr& msg)
{
      if (no_of_times_from_begining_for_depth_sensor==5)
      {
           DEPTH_AT_SURFACE=msg->depth;
      }
      if(no_of_times_from_begining_for_depth_sensor<10)
            no_of_times_from_begining_for_depth_sensor++;
      float data = msg->depth-DEPTH_AT_SURFACE;
      //ROS_INFO("depth: [%f]",data);
      depth = data;
      //if(no_of_times_from_begining_for_depth_sensor>5)
      {
          Q_EMIT updateDepth(depth);
      }
}

void QNode::mt9Callback(const ikat_sensor_data::mt9_sensor_data::ConstPtr& msg)
{
    float Data[3]={0};
    Data[0]=msg->MT9_data[0];
    Data[1]=msg->MT9_data[1];
    Data[2]=msg->MT9_data[2];
    if (no_of_times_from_begining_for_mt9==5)
    {
         theta_relative=Data[2];
    }
    Data[2]=Data[2]-theta_relative+180;
    if(no_of_times_from_begining_for_mt9<20)
          no_of_times_from_begining_for_mt9++;
    if (Data[2]<0)
    {
         Data[2]+=360;
    }
    if (Data[2]>360)
    {
         Data[2]-=360;
    }
   // ROS_INFO("Roll [%f]      ",Data[0]);
   // ROS_INFO("Pitch [%f]     ",Data[1]);
   //ROS_INFO("Yaw [%f]       ",Data[2]);
    roll_mt9 = Data[0];
    pitch_mt9 = Data[1];
    yaw_mt9 = Data[2];
    //if(no_of_times_from_begining_for_mt9>5)
    {
        //std::cout<<"Yaw Updating"<<yaw_mt9<<std::endl;
        Q_EMIT updateYaw(yaw_mt9);
    }
}

}  // namespace controller_gui
