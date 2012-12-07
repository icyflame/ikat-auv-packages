#include <ikat_controller/DepthController.h>

DepthController::DepthController()
{
    depth = 0;
    steady_depth = 0;
    KP_DEPTH=5.5,KI_DEPTH=.3,error_depth=0,sum_depth=0,prev_error_depth=0,diff_depth=0;
    no_of_times_from_begining_for_depth_sensor=0;
    DEPTH_AT_SURFACE=0;
    verticalspeed=0;
}

void DepthController::getdepth(float Data)
{
    depth = Data;
}

float DepthController::depthController(float setDepth)
{
    steady_depth=setDepth;
    error_depth=-depth+steady_depth;
    sum_depth=sum_depth+error_depth;
    verticalspeed=KP_DEPTH*error_depth+KI_DEPTH*sum_depth+3;
    cout<<error_depth<<endl;
    return verticalspeed;
}

DepthController::~DepthController()
{
    verticalspeed=0;
}
