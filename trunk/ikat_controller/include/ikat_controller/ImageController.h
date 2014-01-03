#include <ikat_thrusters/Thrusters.hpp>

class ImageController
{
private:
    float roll_mt9,pitch_mt9, yaw_mt9;
    float steady_angle;
    float KP_YAW,KI_YAW,KD_YAW,error_yaw,sum_yaw,prev_error_yaw,diff_yaw;
    float horizontal_speed,differential_surge_speed,thruster_surge_left,thruster_surge_right;
    float depth;
    float steady_depth;
    float KP_DEPTH,KI_DEPTH,KD_DEPTH,error_depth,sum_depth,prev_error_depth,diff_depth;
    float verticalspeed;
public:
    ImageController();
    float theta_relative;
    float DEPTH_AT_SURFACE;
    void get_Data(float *);
    void yawController(float,float *);
    void speedCallibration(void);
	void setControlParamters(char);
    void reset();
    float depthController(float);
};
