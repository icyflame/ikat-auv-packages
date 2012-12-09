#include <auv_visulaization/setSimulator.hpp>


int main(int argc,char** argv)
{
    ros::init(argc,argv,"setSimulator_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient
            <gazebo::DeleteModel>("delete_model");
    gazebo::DeleteModel del_model;
    del_model.request.model_name = "plane1_model";
    client.call(del_model);
    return 0;
}
