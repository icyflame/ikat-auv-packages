#include <kraken_ideal_controller/ideal_controller.hpp>
geometry_msgs::Pose vehicle_position;
float pre_x=0,pre_z=0,pre_y=0,pre_yaw=0;
void controlInputCallback(const ikat_controller_msgs::control_operations::ConstPtr& msg)
{
    if(msg->heave!=0)
    {
        if(msg->heave>0)
        {
            pre_z+=.05;
        }
        else
        {
            pre_z+=-.05;
        }
    }
    if(msg->surge!=0)
    {
        if(msg->surge>0)
        {
            pre_x+=0.05*cos(pre_yaw);
            pre_y+=0.05*sin(pre_yaw);
        }
        else
        {
            pre_x+=-0.05*cos(pre_yaw);
            pre_y+=-0.05*sin(pre_yaw);
        }
    }
    if(msg->angle!=0)
    {
        if(msg->angle>0)
        {
            pre_yaw+=0.09;
        }
        else
        {
            pre_yaw+=-0.09;
        }
    }
    vehicle_position.position.x = pre_x;
    vehicle_position.position.y = pre_y;
    vehicle_position.position.z = pre_z;
    tf::Quaternion quat(0.0,0.0,pre_yaw);
    vehicle_position.orientation.w = quat.getW();
    vehicle_position.orientation.z = quat.getZ();
    vehicle_position.orientation.y = quat.getY();
    vehicle_position.orientation.x = quat.getX();
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"ideal_controller");
    ros::NodeHandle n;
    ros::Publisher pose_publisher = n.advertise<geometry_msgs::Pose>("/kraken/pose",1);
    ros::Subscriber ideal_control_subscriber = n.subscribe
            <ikat_controller_msgs::control_operations>
            ("/kraken/control_data",1,controlInputCallback);
    vehicle_position.position.x = 0;
    vehicle_position.position.y = 0;
    vehicle_position.position.z = 0;
    vehicle_position.orientation.w = 1;
    vehicle_position.orientation.z = 0;
    vehicle_position.orientation.y = 0;
    vehicle_position.orientation.x = 0;
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        pose_publisher.publish(vehicle_position);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
