#include <serialport/IkatSerialPort.h>
#define THRUSTER_V_MIN -5.0
#define THRUSTER_V_MAX +5.0


class thruster{
                double thv;
                int chan;
                std::string str;
                int byteswritten;
                ikat_sensor::IkatSerialPort Serial;
public:
                thruster(const std::string& port,int baudrate);
                virtual ~thruster();
                void strcmb(double,int);
                bool sendData(void);
                bool startThrusters();
};
