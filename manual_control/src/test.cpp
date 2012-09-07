
#include <ros/ros.h>
#include <ikat_sensor_data/control_data.h>
#include <manual_control/manual_control.hpp>

#define DEPTH_DEBUG 1

int main(int argc,char** argv)
{
    ros::init(argc,argv,"serial_test");
    #ifdef DEPTH_DEBUG
        log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
        my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
    #endif
    ros::NodeHandle n;
    ros::Publisher publish = n.advertise<ikat_sensor_data::control_data>("control_signal",10);
    ikat_sensor_data::control_data control;
    while(ros::ok())
    {
        control.use_image = 0;
        char commmand;
        std::cout<<"Give command";
        std::cin>>commmand;
        float value;
        switch(commmand)
        {
            case 'D':
                std::cout<<"Give Depth value";
                std::cin>>value;
                control.depth = value;
                break;
            case 'A':
                std::cout<<"Give Angle value";
                std::cin>>value;
                control.angle = value;
                break;
            default:
                std::cout<<"Wrong command"<<std::endl;
        }
        publish.publish(control);
    }
    return 0;
}
