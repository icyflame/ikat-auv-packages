/**
 * @file /include/controller_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef controller_gui_QNODE_HPP_
#define controller_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include "std_msgs/String.h"
#include <ikat_sensor_data/depth_sensor_data.h>
#include <ikat_sensor_data/mt9_sensor_data.h>
#include <ikat_sensor_data/control_data.h>
#include <ikat_sensor_data/thruster_data.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace controller_gui {

#define CH0 0x00
#define CH1 0x01
#define CH2 0x02
#define CH3 0x03

#define SurgeLeftThruster	CH0
#define SurgeRightThruster	CH2
#define DepthRightThruster	CH3
#define DepthLeftThruster	CH1

#define MINPOLLING 20

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	/*********************
	** Logging
	**********************/
	/*enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };*/
    
	//QStringListModel* loggingModel() { return &logging_model; }
	//void log( const LogLevel &level, const std::string &msg);
    void mt9Callback(const ikat_sensor_data::mt9_sensor_data::ConstPtr& msg);
    void depthCallback(const ikat_sensor_data::depth_sensor_data::ConstPtr& msg);
    /**
     * Variable to be used by GUI
     **/
    bool runThrusters;
    void testThrusters();
    void stopThruster();
    /******************************************
	** Depth Controller;
	*******************************************/
    float steady_depth;
    float KP_DEPTH,KI_DEPTH,KD_DEPTH;
    bool depth_controller;
    void resetDepthController();
    
    
    /******************************************
	** Yaw Controller;
	*******************************************/
    float error_yaw,steady_angle;
    float KP_YAW,KI_YAW,KD_YAW;
    bool yaw_controller;
    void resetYawController();
    
    /******************************************
	** Speed Controller;
	*******************************************/
    float error_depth,horizontal_speed;
    float KP_SPEED,KI_SPEED,KD_SPEED;
    bool speed_controller;
    void resetSpeedController();
    
    
 Q_SIGNALS:
    void rosShutdown();
    void updateDepth(double value);
    void updateYaw(double value);

private:
	int init_argc;
	char** init_argv;
    ros::Subscriber depth_callback;
    ros::Subscriber mt9_callback;
    ros::Publisher thrusterPub;
    ikat_sensor_data::thruster_data thrusterData;
    bool free_thrusters;
	//ros::Publisher chatter_publisher;
    //QStringListModel logging_model;
    /******************************************
	** Depth Controller;
	*******************************************/
    float depth;
    float sum_depth,prev_error_depth,diff_depth;
    int   no_of_times_from_begining_for_depth_sensor;
    float DEPTH_AT_SURFACE;
    float depthController();
    
    /******************************************
	** Yaw Controller;
	*******************************************/
    float roll_mt9,pitch_mt9, yaw_mt9;
    float sum_yaw,prev_error_yaw,diff_yaw;
    float differential_surge_speed,thruster_surge_left,thruster_surge_right;
    float theta_relative;
    int   no_of_times_from_begining_for_mt9;
    void yawController(float * buff);
    void speedCallibration();
    
    /******************************************
	** Speed Controller;
	*******************************************/
    
    
};

}  // namespace controller_gui

#endif /* controller_gui_QNODE_HPP_ */
