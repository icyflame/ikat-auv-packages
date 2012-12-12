#include <ikat_controller/YawController.h>

YawController::YawController(void)
{
    roll_mt9=0,pitch_mt9=0, yaw_mt9=0;
    steady_angle =0;
    KP_YAW=0.14,KI_YAW=0.00,KD_YAW=.1,error_yaw=0,sum_yaw=0,prev_error_yaw=0.0,diff_yaw=0.0;
    theta_relative=0;
    no_of_times_from_begining_for_mt9=0;
    horizontal_speed=0,differential_surge_speed=0,thruster_surge_left=0,thruster_surge_right=0;
}

void YawController::get_Data(float * Data)
{
    roll_mt9 = Data[0];
    pitch_mt9 = Data[1];
    yaw_mt9 = Data[2];
}

void YawController::yawController(float setangle,float * buff)
{
    steady_angle=setangle;
    error_yaw=steady_angle-yaw_mt9;
    sum_yaw+=error_yaw;
    diff_yaw=error_yaw-prev_error_yaw;
    prev_error_yaw=error_yaw;
    differential_surge_speed=KP_YAW*error_yaw+KI_YAW*sum_yaw+KD_YAW*diff_yaw;
    thruster_surge_left=-differential_surge_speed+horizontal_speed;
    thruster_surge_right= differential_surge_speed+horizontal_speed;
    speedCallibration();
    buff[0]=thruster_surge_left;
    buff[1]=thruster_surge_right;
}

void YawController::speedCallibration(void)
{
    ///////////when still
    if(thruster_surge_left<0 && horizontal_speed==0)
    {
            thruster_surge_left-=2.1;
    }
    if(thruster_surge_right<0 && horizontal_speed==0)
    {
            thruster_surge_right-=2.1;
    }
    if(thruster_surge_left>0 && horizontal_speed==0)
    {
            thruster_surge_left+=2.1;
    }
    if(thruster_surge_right>0 && horizontal_speed==0)
    {
            thruster_surge_right+=2.1;
    }
    //////////when moving forward
    if(thruster_surge_left<2.2 && horizontal_speed>0)
    {
            thruster_surge_left-=4.4;
    }
    if(thruster_surge_right<2.2 && horizontal_speed>0)
    {
            thruster_surge_right-=4.4;
    }
}

YawController::~YawController()
{
    thruster_surge_left=0,thruster_surge_right=0;
}
