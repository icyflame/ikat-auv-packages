#include <mt9_sensor/MT9Sensor.h>

namespace ikat_sensor
{

MT9Sensor::MT9Sensor(std::string port)
{
    data_is_streaming = false;
    setPortName(port);
}

MT9Sensor::MT9Sensor()
{
    data_is_streaming = false;
    setPortName("");
}

inline bool MT9Sensor::setPortName(std::string name)
{
    if(!data_is_streaming)
    {
        port_name = name;
        ROS_DEBUG_STREAM("MT9 port has been set to "<<port_name);
        return false;
    }
    else
    {
        ROS_ERROR_STREAM("MT9 port is already running on port "<<port_name<<" can't change it ot "<<name);
        return false;
    }
}

bool MT9Sensor::setupFilter()
{
    short bLogCalibratedData = 0;
    float fGain = 1.0f;
    short nCorInterval = 1;
    float fRho = 1.0f;
    short nMode = MT_LOGEULER;
    short nSampleFrequency = 100;
    ROS_INFO("Loading MotionTracker library...");
    module = dlopen("libmtobject.so",RTLD_NOW);
    if (!module)
    {
        ROS_ERROR("Couldn't open MotionTracker library: %s\n", dlerror());
        return false;
    }
    ROS_INFO("Library loaded");
    create_t* create_mtobject = (create_t*) dlsym(module,"create");
    destroy_mtobject = (destroy_t*) dlsym(module,"destroy");
    if (!create_mtobject || !destroy_mtobject)
    {
        ROS_ERROR("cannot load symbols from the library for MT9\n");
        return false;
    }
    pMT = create_mtobject();
    ROS_INFO("Setting filter parameters for MT9");
    int i=0;
    if((i=pMT->MT_SetCOMPort_DeviceName((char *)port_name.c_str()))>0)
    {
        pMT->MT_SetCalibratedOutput(bLogCalibratedData);
        pMT->MT_SetFilterSettings(fGain,nCorInterval,fRho);
        pMT->MT_SetTimeout(3);
        pMT->MT_SetSampleFrequency(nSampleFrequency);
        pMT->MT_SetOutputMode(nMode);
        ROS_INFO("MT9 initiated succesfully");
        data_is_streaming = true;
        ROS_INFO("%d",i);
    }
    else
    {
        pMT->MT_StopProcess();
        usleep(1000);
        destroy_mtobject(pMT);
        dlclose(module);
        data_is_streaming = false;
        ROS_ERROR_STREAM("No device connected at "<<port_name);
    }
    return data_is_streaming;
}

bool MT9Sensor::startDataProcessing()
{
    boost::recursive_mutex::scoped_lock lock(mutex_);
    if(!data_is_streaming)
    {
        if(setupFilter())
        {
            pMT->MT_StartProcess();
            ROS_DEBUG("Data streaming has been started");
            return true;
        }
    }
    else
    {
        ROS_ERROR("Data streaming is already running");
    }
    return false;
}

bool MT9Sensor::getData(ikat_sensor_data::mt9_sensor_data &dataptr)
{
    if(data_is_streaming)
    {
        boost::recursive_mutex::scoped_lock lock(mutex_);
        short nNew = 0;
        short nRetval = 0;
        pMT->MT_GetOrientationData(&nNew,fOrientationData,0);
        float *data = dataptr.MT9_data.c_array();
        nRetval = nNew;
        switch(nNew)
        {
            case MT_NEWDATA:
                data = fOrientationData;
                break;
            case MT_NODATA:
                ROS_ERROR("No Data On COM Port\n\n");
                break;
            case MT_NOSENSORID:
                ROS_ERROR("No Sensor ID Received From Sensor\n\n");
                break;
            case MT_INCOMPLETE:
                ROS_ERROR("Incomplete Data Received (Connection Lost)\n\n");
                break;
            case MT_CHECKSUMERROR:
                ROS_ERROR("Checksum Error\n\n");
                break;
            case MT_NOPORT:
                ROS_ERROR("COM port Could Not Be opened\n\n");
                break;
            case MT_NOCALIBVALUES:
                ROS_ERROR("XMU File With Calibration Data Could Not Be Read or \nMTS Data With Calibration Data Not Set\n\n");
                break;
            case MT_POWERLOSS:
                ROS_ERROR("Power Supply To The Sensor Was Probably Interupted\n\n");
                break;
        }
        if(nRetval==MT_NEWDATA)
        {
            return true;
        }
    }
    return false;
}

bool MT9Sensor::stopProcess()
{
    boost::recursive_mutex::scoped_lock lock(mutex_);
    ROS_INFO("Stopping.. MT9 data stream");
    pMT->MT_StopProcess();
    usleep(1000);
    destroy_mtobject(pMT);
    ROS_INFO("Closing MotionTracker shared object...");
    dlclose(module);
    ROS_INFO("MT9 stoped \n\n");
    data_is_streaming = false;
    return true;
}

MT9Sensor::~MT9Sensor()
{
    if(data_is_streaming)
        stopProcess();
}
}// end namespace ikat_sensor
