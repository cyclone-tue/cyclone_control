#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "../include/cyclone_control/control.h"


void posCallback(const geometry_msgs::PoseStamped::ConstPtr& position){
    ROS_INFO("I am at: [%f, %f, %f]", position->pose.position.x, position->pose.position.y, position->pose.position.z);
    broadcastTransform(position);
    if(currentPose == nullptr){
        currentPose = new geometry_msgs::PoseStamped();
        *currentPose = *position;
    }else{
        *currentPose = *position;
    }
    //ROS_INFO("Current pose stored as: [%f, %f, %f]", currentPose->pose.position.x, currentPose->pose.position.y, currentPose->pose.position.z);

}

void broadcastTransform(const geometry_msgs::PoseStamped::ConstPtr &position) {
    static tf::TransformBroadcaster br;
    tf::Transform transform(tf::Quaternion(position->pose.orientation.x, position->pose.orientation.y, position->pose.orientation.z, position->pose.orientation.w),
            tf::Vector3(position->pose.position.x, position->pose.position.y, position->pose.position.z));
    tf::StampedTransform st(transform, position->header.stamp, position->header.frame_id, "base_link");
    br.sendTransform(st);
    //br.sendTransform(position, transform);
    //transform.setOrigin(tf::Vector3(position->position.))
}

void pathCallback(const nav_msgs::Path::ConstPtr &navigationPath) {
    path.clear();
    for (geometry_msgs::PoseStamped pose: navigationPath->poses){
        tf::Stamped<tf::Point> point(tf::Point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z), pose.header.stamp, pose.header.frame_id);
        path.emplace_back(point);
    }
    pathIterator = path.begin();
    targetSet = false;
}

void stateCallback(const mavros_msgs::State::ConstPtr &state) {
    ready = state->armed && state->connected && state->guided;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "control");

    //Point path[];

    ros::NodeHandle n;
    path.emplace_back(tf::Stamped<tf::Point>(tf::Vector3(5.0,0.0,5.0), ros::Time::now(), "map"));
    path.emplace_back(tf::Stamped<tf::Point>(tf::Vector3(5.0,5.0,5.0), ros::Time::now(), "map"));
    path.emplace_back(tf::Stamped<tf::Point>(tf::Vector3(5.0,5.0,10.0), ros::Time::now(), "map"));

    pathIterator = path.begin();
    ROS_INFO("Path lenght is: %ld", path.size());


    ros::Subscriber posSub = n.subscribe("/mavros/local_position/pose", 1000, posCallback); //Position topic
    ros::Subscriber pathSub = n.subscribe("path", 1000, pathCallback); //Path topic
    ros::Subscriber stateSub = n.subscribe("/mavros/state", 1000, stateCallback); //State (arming/connected/guided) topic
    ros::Publisher targetPointPub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local" , 1000);

    ros::Rate loop_rate(10);



    ROS_INFO("Started Control node.");
    while (ros::ok()){
        if(currentPose != nullptr && ready){
            tf::Stamped<tf::Point> target = *pathIterator;
            tf::Point currentPosition(currentPose->pose.position.x, currentPose->pose.position.y, currentPose->pose.position.z);
            if((target - currentPosition).length() < 1 && pathIterator < path.end()-1){
                pathIterator++;
                targetSet = false;
            }else{
                if(!targetSet){
                    geometry_msgs::PoseStamped targetMessage;
                    targetMessage.header = std_msgs::Header();
                    targetMessage.header.stamp = target.stamp_;
                    targetMessage.header.frame_id = target.frame_id_;

                    targetMessage.pose.position.x = target.getX();
                    targetMessage.pose.position.y = target.getY();
                    targetMessage.pose.position.z = target.getZ();

                    targetPointPub.publish(targetMessage);
                    targetSet = true;
                    ROS_INFO("Going to [%g, %f, %f]", targetMessage.pose.position.x, targetMessage.pose.position.y, targetMessage.pose.position.z);
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
