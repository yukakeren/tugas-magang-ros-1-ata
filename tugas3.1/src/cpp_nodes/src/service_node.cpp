#include "ros/ros.h"
#include "std_srvs/Empty.h"

bool handleService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Service has been called!");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "service_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("simple_service", handleService);
    ROS_INFO("Service server ready to receive requests.");

    ros::spin();

    return 0;
}
