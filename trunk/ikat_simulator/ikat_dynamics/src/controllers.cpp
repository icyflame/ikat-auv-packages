#include <ikat_dynamics/controllers.hpp>

namespace ikat_simulator{

PController::PController(float p_value[6], float velocity[6])
{
    for(int i=0; i<6; i++)
    {
        _p_value[i]   = p_value[i];
        _reference[i] = velocity[i];
    }
    auv = new AuvModelSimple6DoF(.01);
}

PController::~PController()
{
    
}

void PController::changeP(float p_value[6])
{
    for(int i=0; i<6; i++)
    {
        _p_value[i]   = p_value[i];
    }
}

void PController::run(float time, std::string &filename)
{
    std::string names = filename+"_error.txt";
    std::ofstream file_error(names.c_str());
    names = filename+"_position.txt";
    std::ofstream file_position(names.c_str());
    names = filename+"_velocity.txt";
    std::ofstream file_velocity(names.c_str());
    names = filename+"_thrusts.txt";
    std::ofstream file_thrusts(names.c_str());
    names = filename+"_accelaration.txt";
    std::ofstream file_accelaration(names.c_str());
    int count = (int)(time/.01);
    for(int i=0; i<count; i++)
    {
        getError();
        updateThrust();
        auv->updateAuv(_thrusts);
        for(int j=0; j<6;j++)
        {
#ifdef BODY
            file_position<<auv->_current_position_to_body[j]<<"\t";
#else
            file_position<<auv->_current_position_to_world[j]<<"\t";
#endif
        }
        for(int j=0; j<6;j++)
        {
#ifdef BODY 
            file_velocity<<auv->_current_velocity_state_to_body[j]<<"\t";
#else
            file_velocity<<auv->_current_velocity_state_to_world[j]<<"\t";
#endif
        }
        for(int j=0; j<6;j++)
        {
            file_error<<_error[j]<<"\t";
        }
        for(int j=0; j<6;j++)
        {
            file_thrusts<<_thrusts[j]<<"\t";
        }
        for(int j=0; j<6;j++)
        {
#ifdef BODY
            file_accelaration<<auv->_current_accelaration_to_body[j]<<"\t";
#else
            file_accelaration<<auv->_current_accelaration_to_world[j]<<"\t";
#endif
        }
        file_error<<std::endl;
        file_position<<std::endl;
        file_velocity<<std::endl;
        file_thrusts<<std::endl;
        file_accelaration<<std::endl;
    }
    file_error.close();
    file_position.close();
    file_velocity.close();
    file_thrusts.close();
    file_accelaration.close();
}

void PController::getError()
{
    for(int i=0; i<6; i++)
    {
#ifdef BODY
        _error[i]=_reference[i]-auv->_current_velocity_state_to_body[i];
#else
        _error[i]=_reference[i]-auv->_current_velocity_state_to_world[i];
#endif
    }
}

void PController::updateThrust()
{
    for(int i=0; i<6; i++)
    {
        _thrusts[i]=_p_value[i]*_error[i];
    }
}

/////////////////////////////////////////////////////////////////////////////////////

PIController::PIController(float pi_value[6][2], float velocity[6])
{
    for(int i=0; i<6; i++)
    {
        _pi_value[i][0]   = pi_value[i][0];
        _pi_value[i][1]   = pi_value[i][1];
        _reference[i] = velocity[i];
        _error[i]=0;
        _error_accumulated[i]=0;
        _thrusts[i]=0;
    }
    auv = new AuvModelSimple6DoF(.01);
}

PIController::~PIController()
{
    
}

void PIController::changePI(float pi_value[6][2])
{
    for(int i=0; i<6; i++)
    {
        _pi_value[i][0]   = pi_value[i][0];
        _pi_value[i][1]   = pi_value[i][1];
    }
}

void PIController::run(float time, std::string &filename)
{
    std::string names = filename+"_error.txt";
    std::ofstream file_error(names.c_str());
    names = filename+"_position.txt";
    std::ofstream file_position(names.c_str());
    names = filename+"_velocity.txt";
    std::ofstream file_velocity(names.c_str());
    names = filename+"_thrusts.txt";
    std::ofstream file_thrusts(names.c_str());
    names = filename+"_accelaration.txt";
    std::ofstream file_accelaration(names.c_str());
    int count = (int)(time/.01);
    for(int i=0; i<count; i++)
    {
        getError();
        updateThrust();
        auv->updateAuv(_thrusts);
        for(int j=0; j<6;j++)
        {
#ifdef BODY
            file_position<<auv->_current_position_to_body[j]<<"\t";
#else
            file_position<<auv->_current_position_to_world[j]<<"\t";
#endif
        }
        for(int j=0; j<6;j++)
        {
#ifdef BODY 
            file_velocity<<auv->_current_velocity_state_to_body[j]<<"\t";
#else
            file_velocity<<auv->_current_velocity_state_to_world[j]<<"\t";
#endif
        }
        for(int j=0; j<6;j++)
        {
            file_error<<_error[j]<<"\t";
        }
        for(int j=0; j<6;j++)
        {
            file_thrusts<<_thrusts[j]<<"\t";
        }
        for(int j=0; j<6;j++)
        {
#ifdef BODY
            file_accelaration<<auv->_current_accelaration_to_body[j]<<"\t";
#else
            file_accelaration<<auv->_current_accelaration_to_world[j]<<"\t";
#endif
        }
        file_error<<std::endl;
        file_position<<std::endl;
        file_velocity<<std::endl;
        file_thrusts<<std::endl;
        file_accelaration<<std::endl;
    }
    file_error.close();
    file_position.close();
    file_velocity.close();
    file_thrusts.close();
    file_accelaration.close();
}

void PIController::getError()
{
    for(int i=0; i<6; i++)
    {
#ifdef BODY
        _error[i]            =_reference[i]-auv->_current_velocity_state_to_body[i];
        _error_accumulated[i]  +=_error[i]; 
#else
        _error[i]=_reference[i]-auv->_current_velocity_state_to_world[i];
        _error_accumulated[i]  +=_error[i]; 
#endif
    }
}

void PIController::updateThrust()
{
    for(int i=0; i<6; i++)
    {
        _thrusts[i]=_pi_value[i][0]*_error[i]+_pi_value[i][1]*_error_accumulated[i];
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////

PIDController::PIDController(float pid_value[6][3], float velocity[6])
{
    for(int i=0; i<6; i++)
    {
        _pid_value[i][0]   = pid_value[i][0];
        _pid_value[i][1]   = pid_value[i][1];
        _pid_value[i][2]   = pid_value[i][2];
        _reference[i] = velocity[i];
        _error[i]=0;
        _error_accumulated[i]=0;
        _error_previous[i]=0;
        _error_difference[i]=0;
        _thrusts[i]=0;
    }
    auv = new AuvModelSimple6DoF(.01);
}

PIDController::~PIDController()
{
    
}

void PIDController::changePID(float pid_value[6][3])
{
    for(int i=0; i<6; i++)
    {
        _pid_value[i][0]   = pid_value[i][0];
        _pid_value[i][1]   = pid_value[i][1];
        _pid_value[i][2]   = pid_value[i][2];
    }
}

void PIDController::run(float time, std::string &filename)
{
    std::string names = filename+"_error.txt";
    std::ofstream file_error(names.c_str());
    names = filename+"_position.txt";
    std::ofstream file_position(names.c_str());
    names = filename+"_velocity.txt";
    std::ofstream file_velocity(names.c_str());
    names = filename+"_thrusts.txt";
    std::ofstream file_thrusts(names.c_str());
    names = filename+"_accelaration.txt";
    std::ofstream file_accelaration(names.c_str());
    int count = (int)(time/.01);
    for(int i=0; i<count; i++)
    {
        getError();
        updateThrust();
        auv->updateAuv(_thrusts);
        for(int j=0; j<6;j++)
        {
#ifdef BODY
            file_position<<auv->_current_position_to_body[j]<<"\t";
#else
            file_position<<auv->_current_position_to_world[j]<<"\t";
#endif
        }
        for(int j=0; j<6;j++)
        {
#ifdef BODY
            file_velocity<<auv->_current_velocity_state_to_body[j]<<"\t";
#else
            file_velocity<<auv->_current_velocity_state_to_world[j]<<"\t";
#endif
        }
        for(int j=0; j<6;j++)
        {
            file_error<<_error[j]<<"\t";
        }
        for(int j=0; j<6;j++)
        {
            file_thrusts<<_thrusts[j]<<"\t";
        }
        for(int j=0; j<6;j++)
        {
#ifdef BODY
            file_accelaration<<auv->_current_accelaration_to_body[j]<<"\t";
#else
            file_accelaration<<auv->_current_accelaration_to_world[j]<<"\t";
#endif
        }
        file_error<<std::endl;
        file_position<<std::endl;
        file_velocity<<std::endl;
        file_thrusts<<std::endl;
        file_accelaration<<std::endl;
    }
    file_error.close();
    file_position.close();
    file_velocity.close();
    file_thrusts.close();
    file_accelaration.close();
}

void PIDController::getError()
{
    for(int i=0; i<6; i++)
    {
        _error_previous[i]=_error[i];
#ifdef BODY
        _error[i]            =_reference[i]-auv->_current_velocity_state_to_body[i];
        _error_accumulated[i]  +=_error[i]; 
#else
        _error[i]=_reference[i]-auv->_current_velocity_state_to_world[i];
        _error_accumulated[i]  +=_error[i]; 
#endif
        _error_difference[i]=_error[i]=_error_previous[i];
    }
}

void PIDController::updateThrust()
{
    for(int i=0; i<6; i++)
    {
        _thrusts[i] =_pid_value[i][0]*_error[i]+_pid_value[i][1]*_error_accumulated[i]
                +_pid_value[i][2]*_error_difference[i];
    }
}



}//end namespace ikat_simulator
