#include <ikat_thrusters/Thrusters.hpp>

using namespace std;

class DepthController
{
private:
    float depth;
    float steady_depth;
    float KP_DEPTH,KI_DEPTH,error_depth,sum_depth,prev_error_depth,diff_depth;
    float verticalspeed;
public:
    int   no_of_times_from_begining_for_depth_sensor;
    float DEPTH_AT_SURFACE;
    DepthController();
    void getdepth(float);
    float depthController(float);
    ~DepthController();
};
