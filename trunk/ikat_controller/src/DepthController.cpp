#include <ikat_controller/DepthController.h>
DepthController::DepthController()
{
    depth = 0;
    steady_depth = 0;
    KP_DEPTH=35,KI_DEPTH=0.5,KD_DEPTH=15;error_depth=0,sum_depth=0,prev_error_depth=0,diff_depth=0;
    no_of_times_from_begining_for_depth_sensor=0;
    DEPTH_AT_SURFACE=0;
    verticalspeed=0;
    _memory.push(0);
}

void DepthController::getdepth(float Data)
{
    depth = Data;
}

float DepthController::depthController(float setDepth)
{

    steady_depth=setDepth;
    error_depth=depth-steady_depth;
    diff_depth = error_depth-prev_error_depth;
    prev_error_depth = error_depth;
    sum_depth+=error_depth;
    verticalspeed=KP_DEPTH*error_depth+KI_DEPTH*sum_depth+KD_DEPTH*prev_error_depth+3;
    //cout<<error_depth<<endl;
    /*if(_memory.size()<100)
    {
        _memory.push(sum_depth);
    }
    else
    {
        sum_depth = sum_depth-_memory.front();
        _memory.pop();
       _memory.push(sum_depth);
    }*/
    return -verticalspeed;
}

DepthController::~DepthController()
{
    verticalspeed=0;
}
