#include <auv_visulaization/setSimulator.hpp>


int main(int argc,char** argv)
{
    ros::init(argc,argv,"setSimulator_client");
    ros::NodeHandle n;
    //ros::ServiceClient client = n.serviceClient
            //<gazebo::DeleteModel>("deleteModel");
    gazebo::DeleteModel del_model;
    del_model.request.model_name = "plane1_model";
    ros::service::call("gazebo/delete_model",del_model);
    if(del_model.response.success)
    {
        ROS_INFO_STREAM("Yo YO");
    }
    else
    {
        ROS_INFO_STREAM("Fuck");
    }
    //
    return 0;
}
