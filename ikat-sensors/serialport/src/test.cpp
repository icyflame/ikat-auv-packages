#include <serialport/IkatSerialPort.h>
#include <ros/ros.h>

// comment the below line to hide the data from console
#define DEPTH_DEBUG 1

int main(int argc, char** argv)
{
    if(argc>1)
    {
        ros::init(argc,argv,"serial_test");
        #ifdef DEPTH_DEBUG
            log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
            my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
        #endif
        ikat_sensor::IkatSerialPort serial_port(argv[1],9600,PARITYEN|STOPBIT2);
        serial_port.openPort();
        std::cout<<"test started :)"<<std::endl;
        serial_port.closePort();
    }
  return 0;
}
