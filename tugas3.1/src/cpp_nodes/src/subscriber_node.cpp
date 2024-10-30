#include "ros/ros.h"
#include "std_msgs/String.h"

void topic1Callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received on topic1: %s", msg->data.c_str());
}

void topic2Callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received on topic2: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("topic1", 10, topic1Callback);
    ros::Subscriber sub2 = nh.subscribe("topic2", 10, topic2Callback);

    ros::spin();

    return 0;
}
