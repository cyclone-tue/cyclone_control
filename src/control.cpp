#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "../include/cyclone_control/control.h"


void posCallback(const geometry_msgs::PoseStamped::ConstPtr& position){
    ROS_INFO("I am at: [%f, %f, %f]", position->pose.position.x, position->pose.position.y, position->pose.position.z);
    broadcastTransform(position);
    if(currentPose == nullptr){
        currentPose = new tf::Stamped<tf::Pose>();
        tf::poseStampedMsgToTF(*position, *currentPose);
    }else{
        tf::poseStampedMsgToTF(*position, *currentPose);
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
    tf::TransformListener pathListener;

    tf::StampedTransform transform;
    try{
        pathListener.waitForTransform("map", navigationPath->header.frame_id, navigationPath->header.stamp, ros::Duration(1.0));
        pathListener.lookupTransform("map", navigationPath->header.frame_id, navigationPath->header.stamp, transform);
    }catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ROS_ERROR("Could not update path. No valid tranform from %s to map", navigationPath->header.frame_id.c_str());
        return;
    }


    for (const geometry_msgs::PoseStamped &pose: navigationPath->poses){//Transform every point to world space and add to the path list.
        tf::Stamped<tf::Point> point(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z), navigationPath->header.stamp, navigationPath->header.frame_id);
        tf::Stamped<tf::Point> mapPoint;
        pathListener.transformPoint("map", point, mapPoint);
        path.emplace_back(mapPoint);
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
            tf::Point currentPosition = currentPose->getOrigin();
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
