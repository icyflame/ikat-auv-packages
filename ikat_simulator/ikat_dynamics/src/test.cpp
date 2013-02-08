#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <ikat_dynamics/controllers.hpp>
int main(int argc, char** argv)
{
    ros::init(argc,argv,"simlaotr");
    /*float p[6][2]   = { {10, 1,}, 
                        {10, 1,}, 
                        {10, 1},
                        {10, 1,}, 
                        {10, 1,}, 
                        {10, 1}
                      };*
    float p[6][3]   = { {10, 1, 1}, 
                        {10, 1, 1,}, 
                        {10, 1, 1},
                        {10, 1, 1,}, 
                        {10, 1, 1,}, 
                        {10, 1, 1}
                      };*/
    float p[6]   = { 10,
                     10, 
                     10,
                     10, 
                     10,
                     10
                    };
    float vel[] = {.5, .5, .0, 0, 0, 0};
    ikat_simulator::PController controller(p,vel);
    std::string file = "P_contoller";
    controller.run(10,file);
    return 0;
}
