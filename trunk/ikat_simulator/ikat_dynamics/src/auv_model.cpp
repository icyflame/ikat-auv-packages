#include <ikat_dynamics/auv_model.hpp>

namespace ikat_simulator{

AuvModelSimple6DoF::AuvModelSimple6DoF(float time)
{
    _time = time;
    initAuv();
}

AuvModelSimple6DoF::~AuvModelSimple6DoF()
{
    
}

void AuvModelSimple6DoF::initAuv()
{
    _current_velocity_state_to_world  = new float[6];
    _previous_velocity_state_to_world    = new float[6];
    for(int i =0;i<6;i++)
    {
        _current_velocity_state_to_world[i]  = 0;
        _previous_velocity_state_to_world[i] = 0;
        _current_velocity_state_to_body[i]   = 0;
        _current_position_to_world[i]        = 0;
        _current_position_to_body[i]         = 0;
        _current_accelaration_to_body[i]     = 0;
        _current_accelaration_to_world[i]    = 0;
        _external_torque[i]                  = 0;
        _damping_torque[i]                   = 0;
    }
}

void AuvModelSimple6DoF::updateTransformMarixes()
{
    for(int i =0;i<3;i++)
    {
        _cos_rpy[i]   = cos(_current_position_to_world[i+3]);
        _sin_rpy[i] = sin(_current_position_to_world[i+3]);
        
    }
    //////////////////////////////////////////////////////////////
    _current_linear_velocity_body_to_world_matrix[0][0]   = _cos_rpy[2]*_cos_rpy[1];
    _current_linear_velocity_body_to_world_matrix[0][1]   = _cos_rpy[2]*_sin_rpy[1]*_sin_rpy[0]-_sin_rpy[2]*_cos_rpy[0];
    _current_linear_velocity_body_to_world_matrix[0][2]   = _cos_rpy[2]*_cos_rpy[0]*_sin_rpy[1]+_sin_rpy[2]*_sin_rpy[0];
    _current_linear_velocity_body_to_world_matrix[1][0]   = _sin_rpy[2]*_cos_rpy[1];
    _current_linear_velocity_body_to_world_matrix[1][1]   = _sin_rpy[0]*_sin_rpy[1]*_sin_rpy[2]+_cos_rpy[2]*_cos_rpy[0];
    _current_linear_velocity_body_to_world_matrix[1][2]   = _sin_rpy[1]*_sin_rpy[2]*_cos_rpy[0]-_cos_rpy[2]*_sin_rpy[0];
    _current_linear_velocity_body_to_world_matrix[2][0]   = -_sin_rpy[2];
    _current_linear_velocity_body_to_world_matrix[2][1]   = _sin_rpy[0]*_cos_rpy[1];
    _current_linear_velocity_body_to_world_matrix[2][2]   = _cos_rpy[0]*_cos_rpy[1];
    //////////////////////////////////////////////////////////////
    _current_angular_velocity_body_to_world_matrix[0][0]  = 1;
    _current_angular_velocity_body_to_world_matrix[0][1]  = _sin_rpy[0]*_sin_rpy[1]/_cos_rpy[1];
    _current_angular_velocity_body_to_world_matrix[0][2]  = _cos_rpy[0]*_sin_rpy[1]/_cos_rpy[1];
    _current_angular_velocity_body_to_world_matrix[1][0]  = 0;
    _current_angular_velocity_body_to_world_matrix[1][1]  = _cos_rpy[0];
    _current_angular_velocity_body_to_world_matrix[1][2]  = -_sin_rpy[0];
    _current_angular_velocity_body_to_world_matrix[2][0]  = 0;
    _current_angular_velocity_body_to_world_matrix[2][1]  = _sin_rpy[0]/_cos_rpy[1];
    _current_angular_velocity_body_to_world_matrix[2][2]  = _cos_rpy[0]/_cos_rpy[1];
}

void AuvModelSimple6DoF::updateTorques(float force[6])
{
    for(int i=0;i<6;i++)
    {
        float val=0,val1=0;
        for(int j=0;j<6;j++)
        {
            val +=L[i][j];
            val1+=D[i][j];
        }
        _external_torque[i]= val*force[i];
        _damping_torque[i] = val1*_current_velocity_state_to_body[i];
    }
}

void AuvModelSimple6DoF::updateCurrentAccelaration()
{
    for(int i=0;i<6;i++)
    {
        _current_accelaration_to_body[i] = (_external_torque[i]-G[i]-_damping_torque[i])/mass[i][i];
    }
    updateCurrentVelocity();
    /*multiply(_current_angular_velocity_body_to_world_matrix,
             &_current_accelaration_to_body[3],&_current_accelaration_to_world[3]);
    multiply(_current_linear_velocity_body_to_world_matrix,
             &_current_accelaration_to_body[0],&_current_accelaration_to_world[0]);*/
    for(int i =0;i<6;i++)
    {
        _current_accelaration_to_world[i] = (_current_velocity_state_to_world[i]-_previous_velocity_state_to_world[i])/_time;
    }
}

void AuvModelSimple6DoF::updateCurrentVelocity()
{
    float buff;
    for(int i=0;i<6;i++)
    {
        _current_velocity_state_to_body[i]+=_current_accelaration_to_body[i]*_time;
        buff = _previous_velocity_state_to_world[i];
        _previous_velocity_state_to_world[i] = _current_velocity_state_to_world[i];
        _current_velocity_state_to_world[i] = buff;
    }
    multiply(_current_linear_velocity_body_to_world_matrix,
             &_current_velocity_state_to_body[0],&_current_velocity_state_to_world[0]);
    multiply(_current_angular_velocity_body_to_world_matrix,
             &_current_velocity_state_to_body[3],&_current_velocity_state_to_world[3]);
}

void AuvModelSimple6DoF::updateCurrentPosition()
{
    for(int i =0;i<6;i++)
    {
        _current_position_to_world[i] +=(_current_velocity_state_to_world[i]+_current_accelaration_to_world[i]*_time/2.0)*_time;
        _current_position_to_body[i]  +=(_current_velocity_state_to_body[i]+_current_accelaration_to_body[i]*_time/2.0)*_time;
    }
}

void AuvModelSimple6DoF::updateAuv(float force[6])
{
    updateTorques(force);
    updateTransformMarixes();
    updateCurrentAccelaration();
    updateCurrentPosition();
}

void multiply(float matrix[][3], float* src_vec, float* dst_vec)
{
    for(int i=0; i<3; i++)
    {
        float val=0;
        for(int j =0; j<3; j++)
        {
            val+=matrix[i][j]*src_vec[j];
        }
    }
}

AuvModelWater6DoF::AuvModelWater6DoF(float time)
{
    _time = time;
    initAuv();
}

AuvModelWater6DoF::~AuvModelWater6DoF()
{
    
}

void AuvModelWater6DoF::initAuv()
{
    _current_velocity_state_to_world  = new float[6];
    _previous_velocity_state_to_world    = new float[6];
    for(int i =0;i<6;i++)
    {
        _current_velocity_state_to_world[i]  = 0;
        _previous_velocity_state_to_world[i] = 0;
        _current_velocity_state_to_body[i]   = 0;
        _current_position_to_world[i]        = 0;
        _current_position_to_body[i]         = 0;
        _current_accelaration_to_body[i]     = 0;
        _current_accelaration_to_world[i]    = 0;
        _external_torque[i]                  = 0;
        _damping_torque[i]                   = 0;
    }
}

void AuvModelWater6DoF::updateTransformMarixes()
{
    for(int i =0;i<3;i++)
    {
        _cos_rpy[i]   = cos(_current_position_to_world[i+3]);
        _sin_rpy[i] = sin(_current_position_to_world[i+3]);
        
    }
    //////////////////////////////////////////////////////////////
    _current_linear_velocity_body_to_world_matrix[0][0]   = _cos_rpy[2]*_cos_rpy[1];
    _current_linear_velocity_body_to_world_matrix[0][1]   = _cos_rpy[2]*_sin_rpy[1]*_sin_rpy[0]-_sin_rpy[2]*_cos_rpy[0];
    _current_linear_velocity_body_to_world_matrix[0][2]   = _cos_rpy[2]*_cos_rpy[0]*_sin_rpy[1]+_sin_rpy[2]*_sin_rpy[0];
    _current_linear_velocity_body_to_world_matrix[1][0]   = _sin_rpy[2]*_cos_rpy[1];
    _current_linear_velocity_body_to_world_matrix[1][1]   = _sin_rpy[0]*_sin_rpy[1]*_sin_rpy[2]+_cos_rpy[2]*_cos_rpy[0];
    _current_linear_velocity_body_to_world_matrix[1][2]   = _sin_rpy[1]*_sin_rpy[2]*_cos_rpy[0]-_cos_rpy[2]*_sin_rpy[0];
    _current_linear_velocity_body_to_world_matrix[2][0]   = -_sin_rpy[2];
    _current_linear_velocity_body_to_world_matrix[2][1]   = _sin_rpy[0]*_cos_rpy[1];
    _current_linear_velocity_body_to_world_matrix[2][2]   = _cos_rpy[0]*_cos_rpy[1];
    //////////////////////////////////////////////////////////////
    _current_angular_velocity_body_to_world_matrix[0][0]  = 1;
    _current_angular_velocity_body_to_world_matrix[0][1]  = _sin_rpy[0]*_sin_rpy[1]/_cos_rpy[1];
    _current_angular_velocity_body_to_world_matrix[0][2]  = _cos_rpy[0]*_sin_rpy[1]/_cos_rpy[1];
    _current_angular_velocity_body_to_world_matrix[1][0]  = 0;
    _current_angular_velocity_body_to_world_matrix[1][1]  = _cos_rpy[0];
    _current_angular_velocity_body_to_world_matrix[1][2]  = -_sin_rpy[0];
    _current_angular_velocity_body_to_world_matrix[2][0]  = 0;
    _current_angular_velocity_body_to_world_matrix[2][1]  = _sin_rpy[0]/_cos_rpy[1];
    _current_angular_velocity_body_to_world_matrix[2][2]  = _cos_rpy[0]/_cos_rpy[1];
}

void AuvModelWater6DoF::updateTorques(float force[6])
{
    for(int i=0;i<6;i++)
    {
        float val=0,val1=0;
        for(int j=0;j<6;j++)
        {
            val +=L[i][j];
            val1+=D[i][j];
        }
        _external_torque[i]= val*force[i];
        _damping_torque[i] = val1*_current_velocity_state_to_body[i];
    }
}

void AuvModelWater6DoF::updateCurrentAccelaration()
{
    for(int i=0;i<6;i++)
    {
        _current_accelaration_to_body[i] = (_external_torque[i]-G[i]-_damping_torque[i])/mass[i][i];
    }
    updateCurrentVelocity();
    /*multiply(_current_angular_velocity_body_to_world_matrix,
             &_current_accelaration_to_body[3],&_current_accelaration_to_world[3]);
    multiply(_current_linear_velocity_body_to_world_matrix,
             &_current_accelaration_to_body[0],&_current_accelaration_to_world[0]);*/
    for(int i =0;i<6;i++)
    {
        _current_accelaration_to_world[i] = (_current_velocity_state_to_world[i]-_previous_velocity_state_to_world[i])/_time;
    }
}

void AuvModelWater6DoF::updateCurrentVelocity()
{
    float buff;
    for(int i=0;i<6;i++)
    {
        _current_velocity_state_to_body[i]+=_current_accelaration_to_body[i]*_time;
        buff = _previous_velocity_state_to_world[i];
        _previous_velocity_state_to_world[i] = _current_velocity_state_to_world[i];
        _current_velocity_state_to_world[i] = buff;
    }
    multiply(_current_linear_velocity_body_to_world_matrix,
             &_current_velocity_state_to_body[0],&_current_velocity_state_to_world[0]);
    multiply(_current_angular_velocity_body_to_world_matrix,
             &_current_velocity_state_to_body[3],&_current_velocity_state_to_world[3]);
}

void AuvModelWater6DoF::updateCurrentPosition()
{
    for(int i =0;i<6;i++)
    {
        _current_position_to_world[i] +=(_current_velocity_state_to_world[i]+_current_accelaration_to_world[i]*_time/2.0)*_time;
        _current_position_to_body[i]  +=(_current_velocity_state_to_body[i]+_current_accelaration_to_body[i]*_time/2.0)*_time;
    }
}

void AuvModelWater6DoF::updateAuv(float force[6])
{
    updateTorques(force);
    updateTransformMarixes();
    updateCurrentAccelaration();
    updateCurrentPosition();
}

void AuvModelWater6DoF::updateDampingTorque()
{
    
}

}//end namespace ikat_simulator
