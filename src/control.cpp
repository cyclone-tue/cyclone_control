#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

void posCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
    ROS_INFO("I am at: [%f, %f, %f]", pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "control");


    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/mavros/local_position/pose", 1000, posCallback);

    ros::spin();
}
