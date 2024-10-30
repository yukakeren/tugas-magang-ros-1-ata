#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh;

    // Define two publishers for two topics
    ros::Publisher pub1 = nh.advertise<std_msgs::String>("topic1", 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::String>("topic2", 10);

    ros::Rate loop_rate(1);  // 1 Hz

    while (ros::ok())
    {
        std_msgs::String msg1;
        std_msgs::String msg2;

        msg1.data = "Message on topic1";
        msg2.data = "Message on topic2";

        ROS_INFO("Publishing: %s", msg1.data.c_str());
        ROS_INFO("Publishing: %s", msg2.data.c_str());

        pub1.publish(msg1);
        pub2.publish(msg2);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
