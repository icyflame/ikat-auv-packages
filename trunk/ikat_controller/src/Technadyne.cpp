#include <stdio.h>
#include <ikat_controller/Technadyne.h>
#include <string.h>
#include <iostream>
#include <cstring>


#define ADD1 0x01
#define ADD2 0x02


thruster::thruster(const std::string &port,int baudrate=9600):Serial(port,baudrate)
{
}

bool thruster::startThrusters()
{
    return Serial.openPort();
}

thruster::~thruster()
{
    Serial.~IkatSerialPort();
}

void thruster::strcmb(double a,int b)
{
    thv=a;
    chan=b;
    str.clear();
    char buff[20];
    memset(buff,0,sizeof(buff));
    if(thv>5.0)
    {
        printf("thruster voltage crossed +5.0 !!!\n");
        thv=5.0;
    }
    if(thv<-5.0)
    {
        printf("thruster voltage crossed -5.0 !!!\n");
        thv=-5.0;
    }
    if(thv>=0.0)
    {
        byteswritten=sprintf(buff,"#0%dC%d+0%.3lf\r",ADD1,chan,thv);
    }
    else
    {   
        byteswritten=sprintf(buff,"#0%dC%d-0%.3lf\r",ADD1,chan,-thv);//"#0%dC%d-0%.3lf\r"
    }
    str = buff;
}

bool thruster::sendCommand(double speed,int channel)
{
    strcmb(speed,channel);
    if(Serial.writeData(str)==byteswritten)
    {
        return true;
    }
    return false;
}
