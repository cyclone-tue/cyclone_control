//
// Created by stijn on 3-3-19.
//

#include "../include/cyclone_control/interactive_path.h"
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include "../include/cyclone_control/TargetMarker.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pathPublisher;
nav_msgs::Path path;

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){

    geometry_msgs::PoseStamped point;
    switch (feedback->event_type){
        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            ROS_INFO_STREAM( feedback->marker_name << " is now at "
                                                   << feedback->pose.position.x << ", " << feedback->pose.position.y
                                                   << ", " << feedback->pose.position.z );
            path.header.stamp = feedback->header.stamp;
            path.header.frame_id = feedback->header.frame_id;
            path.poses.clear();
            point.header.stamp = feedback->header.stamp;
            point.header.frame_id = feedback->header.frame_id;
            point.pose.position.x = feedback->pose.position.x;
            point.pose.position.y = feedback->pose.position.y;
            point.pose.position.z = feedback->pose.position.z;
            path.poses.emplace_back(point);
            break;
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            ROS_INFO_STREAM("Send path data to the drone.");
            pathPublisher.publish(path);
            break;
    }



}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_path");

    interactive_markers::InteractiveMarkerServer server("drone_target");

    /*visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "target";
    int_marker.description = "Simple 1-DOF target";

    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.45;
    box_marker.scale.y = 0.45;
    box_marker.scale.z = 0.45;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;

    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    //box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    box_control.markers.push_back(box_marker);

    int_marker.controls.push_back(box_control);


    server.insert(int_marker, &processFeedback);*/

    ros::NodeHandle n;
    pathPublisher = n.advertise<nav_msgs::Path>("path", 1000);


    TargetMarker marker(false, visualization_msgs::InteractiveMarkerControl::NONE, tf::Vector3(0.0, 0.0, 0.0), true, true, &server, &processFeedback);

    server.applyChanges();

    ros::spin();
}

