#include <ros/ros.h>
#include <ikat_controller_msgs/control_operations.h>


int main(int argc,char** argv)
{
    ros::init(argc,argv,"controller_test_ideal");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<ikat_controller_msgs::control_operations>
            ("/kraken/control_data",1);
    while(ros::ok())
    {
        ikat_controller_msgs::control_operations msg;
        std::cout<<"for forward type f"<<std::endl;
        std::cout<<"for backward type b"<<std::endl;
        std::cout<<"for downward type d"<<std::endl;
        std::cout<<"for upward type u"<<std::endl;
        std::cout<<"for clocwise rotation type c"<<std::endl;
        std::cout<<"for anti-clocwise rotation type a"<<std::endl;
        std::cout<<"your choice : ";
        char input;
        std::cin>>input;
        msg.surge = 0;
        msg.heave = 0;
        msg.angle = 0;
        switch(input)
        {
            case 'f':
                msg.surge = 1.0;
                break;
            case 'b':
                msg.surge = -1.0;
                break;
            case 'd':
                msg.heave = 1.0;
                break;
            case 'u':
                msg.heave = -1.0;
                break;
            case 'a':
                msg.angle = -1.0;
                break;
            case 'c':
                msg.angle = 1.0;
                break;
            default :
                std::cout<<"wrong input "<<input<<" retry"<<std::endl;
        }
        publisher.publish(msg);
    }
    return 0;
}
