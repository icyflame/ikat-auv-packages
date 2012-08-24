#ifndef MT9_SENSOR_H
#define MT9_SENSOR_H
#include <ros/ros.h>
#include <mt9_sensor/MotionTracker.h>
#include <dlfcn.h>
#include <unistd.h>
#include <ikat_sensor_data/mt9_sensor_data.h>
#include <boost/thread.hpp>
namespace ikat_sensor
{

    // Return values for MT_GetOrientationData function
    #define MT_NEWDATA			1
    #define MT_NODATA			2
    #define MT_NOSENSORID		3
    #define MT_INCOMPLETE		4
    #define MT_CHECKSUMERROR	5
    #define MT_NOPORT			6
    #define MT_NOCALIBVALUES	7
    #define MT_POWERLOSS		8

    // Output possibilites for MotionTracker library
    #define MT_LOGQUATERNION	0
    #define MT_LOGEULER			1
    #define MT_LOGROTMATRIX		2

    #define RESET_HEADING		0
    #define RESET_GLOBAL		1
    #define RESET_OBJECT		2
    #define RESET_ALIGN			3

    class MT9Sensor
    {
      private:
        void *module;
        MotionTracker* pMT;
        destroy_t* destroy_mtobject;
        std::string port_name ;
        bool data_is_streaming;
        bool setupFilter();
        bool stopProcess();
        float fOrientationData[10];
        boost::recursive_mutex mutex_;
      public:
        MT9Sensor(std::string port);
        bool startDataProcessing();
        inline bool setPortName(std::string name);
        bool getData(ikat_sensor_data::mt9_sensor_data &dataptr);
        MT9Sensor();
        virtual ~MT9Sensor();
    };
}// end namespace ikat_sensor
#endif
