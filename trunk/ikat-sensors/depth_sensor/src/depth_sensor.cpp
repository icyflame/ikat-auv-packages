#include <depth_sensor/depth_sensor.h>

namespace ikat_sensor
{
DepthSensor::DepthSensor(const std::string &name):serial_port(name,9600)
{
    ROS_DEBUG_STREAM("Serial port has been created with name "<<name);
}

DepthSensor::~DepthSensor()
{
    serial_port.~IkatSerialPort();
}

bool DepthSensor::startTransmission()
{
    if(serial_port.openPort())
    {
        std::string response;
        /// setting baudrate on the sensor
        sendData("*9900BR=9600\r\n");

        //ros::spinOnce();
        if(recieveData(response))
        {
            ROS_INFO_STREAM("In response of *9900BR=9600 recieved signal is "<<response);
        }
        else
        {
            ROS_ERROR_STREAM("Error in communication while sending *9900BR=9600");
        }
        /// setting noparity on the sensor
        sendData("*9900PT=N\r\n");
        if(recieveData(response))
        {
            ROS_INFO_STREAM("In response of *9900PT=N recieved signal is "<<response);
        }
        else
        {
            ROS_ERROR_STREAM("Error in communication while sending *9900PT=N");
        }
        /// setting unit on the sensor
        sendData("*0100EW*0100UN=8\r\n");
        if(recieveData(response))
        {
            ROS_INFO_STREAM("In response of *0100EW*0100UN=8 recieved signal is "<<response);
        }
        else
        {
            ROS_ERROR_STREAM("Error in communication while sending *0100EW*0100UN=");
        }
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("Can't open the port connected to depth Sensor");
        return false;
    }
}

void DepthSensor::pauseTransmission()
{
    serial_port.closePort();
}

bool DepthSensor::sendData(const std::string &data)
{
    boost::recursive_mutex::scoped_lock lock(mutex_);
    int datano = serial_port.writeData(data);
    if(datano>-1)
    {
        ROS_DEBUG_STREAM("send data : "<< data<<" to device ");
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("Not able to send data : "<< data<<" to device ");
        return false;
    }
}

bool DepthSensor::recieveData(std::string &data)
{
    usleep(1000);
    data.clear();
    char buff,buff1;
    int datano =0;
    do
    {
       buff1 = buff;
       if(!serial_port.readChar(buff))
       {
           ROS_ERROR_STREAM("character not read properly hence aborting reading");
           return false;
       }
       data+=buff;
    }while((buff!='\n'));
    return true;
}

bool DepthSensor::getDepth(float &depth)
{
    std::string response;
    if(sendData("*0100P3\r\n"))
    {
        if(recieveData(response))
        {
            ROS_DEBUG_STREAM("In response of *0100P3 recieved signal is "<<response);
            depth = current_depth = strtod(&(response.c_str()[5]),NULL);
            ROS_DEBUG_STREAM(depth);
            return true;
        }
        else
        {
            ROS_ERROR_STREAM("Error in communication while sending *0100P3");
        }
    }
    return false;
}

}
