#include <ikat_controller/ImageController.h>
#include <task_planner/TASKS_PARAM.h>

ImageController::ImageController()
{
    KP_YAW = 0.004;
    KI_YAW = 0;
    KD_YAW = 0.001;
    KP_DEPTH = 0.007;
    KI_DEPTH = 0;
    KD_DEPTH = 0.03;
    sum_yaw = 0;
    sum_depth = 0;
    prev_error_yaw = 0;
    prev_error_depth = 0;
    horizontal_speed = 0;
    TASKS.readFromFile();
}
void ImageController::setControlParamters(char choice)
{
    switch(choice)
    {
        case BUOY:
            KP_YAW = 0.002;
            KI_YAW = 0;
            KD_YAW = 0.001;
            KP_DEPTH = 0.007;
            KI_DEPTH = 0;
            KD_DEPTH = 0.0;
            sum_yaw = 0;
            sum_depth = 0;
            prev_error_yaw = 0;
            prev_error_depth = 0;
            horizontal_speed = 3;
            break;
        case MARKER:
            KP_YAW = 0.07;
            KI_YAW = 0;
            KD_YAW = 0.05;
            KP_DEPTH = 0.007;
            KI_DEPTH = 0;
            KD_DEPTH = 0.03;
            sum_yaw = 0;
            sum_depth = 0;
            prev_error_yaw = 0;
            prev_error_depth = 0;
            horizontal_speed = 2.5;
            break;
       default:
            KP_YAW = 0.0;
            KI_YAW = 0;
            KD_YAW = 0.0;
            KP_DEPTH = 0.0;
            KI_DEPTH = 0;
            KD_DEPTH = 0.0;
            sum_yaw = 0;
            sum_depth = 0;
            prev_error_yaw = 0;
            prev_error_depth = 0;
            horizontal_speed = 0;
            break;
    }
}

void ImageController::yawController(float error_yaw_image,float *buff)
{
    error_yaw=error_yaw_image;
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

void ImageController::speedCallibration(void)
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

void ImageController::reset()
{
    KP_YAW = 0.000;
    KI_YAW = 0;
    KD_YAW = 0.000;
    KP_DEPTH = 0.000;
    KI_DEPTH = 0;
    KD_DEPTH = 0.00;
    sum_yaw = 0;
    sum_depth = 0;
    prev_error_yaw = 0;
    prev_error_depth = 0;
    horizontal_speed = 0;
}

float ImageController::depthController(float error_depth_image)
{      
    error_depth=-error_depth_image;
    sum_depth=sum_depth+error_depth;
    diff_depth = error_depth - prev_error_depth;
    prev_error_depth = error_depth;
    return (KP_DEPTH*error_depth+KI_DEPTH*sum_depth+KD_DEPTH*diff_depth+4.1);
}
