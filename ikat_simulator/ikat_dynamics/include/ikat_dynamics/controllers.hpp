#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <ikat_dynamics/auv_model.hpp>
#include <iostream>
#include <fstream>
#include <string>
#define BODY 1
namespace ikat_simulator{

class PController
{
    public:
        PController(float p_value[6],float velocity[6]);
        void run(float time, std::string &filename);
        void changeP(float p_value[6]);
        ~PController();
    private:
        float _reference[6];
        float _error[6];
        float _thrusts[6];
        const static float _velocity_to_thrust[6];
        AuvModelSimple6DoF* auv;
        float _p_value[6];
        void getError();
        void updateThrust();
        
};

class PIController
{
    public:
        PIController(float pi_value[6][2],float velocity[6]);
        void run(float time, std::string &filename);
        void changePI(float pi_value[6][2]);
        ~PIController();
    private:
        float _reference[6];
        float _error[6];
        float _error_accumulated[6];
        float _thrusts[6];
        const static float _velocity_to_thrust[6];
        AuvModelSimple6DoF* auv;
        float _pi_value[6][2];
        void getError();
        void updateThrust();
};

class PIDController
{
    public:
        PIDController(float pid_value[6][3],float velocity[6]);
        void run(float time, std::string &filename);
        void changePID(float pid_value[6][3]);
        ~PIDController();
    private:
        float _reference[6];
        float _error[6];
        float _error_accumulated[6];
        float _error_previous[6];
        float _error_difference[6];
        float _thrusts[6];
        const static float _velocity_to_thrust[6];
        AuvModelSimple6DoF* auv;
        float _pid_value[6][3];
        void getError();
        void updateThrust();
};

const float PController::_velocity_to_thrust[6]   = {1,1,1,1,1,1};
const float PIController::_velocity_to_thrust[6]  = {1,1,1,1,1,1};
const float PIDController::_velocity_to_thrust[6] = {1,1,1,1,1,1};

}// end ikat_simulator

#endif
