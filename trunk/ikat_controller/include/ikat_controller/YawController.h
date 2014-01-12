#include <ikat_thrusters/Thrusters.hpp>

class YawController
{
private:
    float roll_mt9,pitch_mt9, yaw_mt9;
    float steady_angle;
    float KP_YAW,KI_YAW,KD_YAW,error_yaw,sum_yaw,prev_error_yaw,diff_yaw;
    float differential_surge_speed,thruster_surge_left,thruster_surge_right;
public:
    float theta_relative,horizontal_speed;
    int   no_of_times_from_begining_for_mt9;
    YawController();
    void get_Data(float *);
    void yawController(float,float *);
    void speedCallibration(void);
    ~YawController();
};
