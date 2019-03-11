//
// Created by stijn on 3-3-19.
//

#include <tf/transform_datatypes.h>
#include "../include/cyclone_control/TargetMarker.h"
#include <interactive_markers/interactive_marker_server.h>

using namespace visualization_msgs;


visualization_msgs::Marker TargetMarker::makeBox(visualization_msgs::InteractiveMarker &msg) {
    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = msg.scale *0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

visualization_msgs::InteractiveMarkerControl &TargetMarker::makeBoxControl(visualization_msgs::InteractiveMarker &msg, bool showBox) {
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    if(showBox){
        control.markers.push_back( makeBox(msg) );
    }
    msg.controls.push_back( control );

    return msg.controls.back();
}

TargetMarker::TargetMarker(bool fixed, unsigned int interaction_mode, tf::Vector3 position, bool show_6dof, bool awaitConfirm, interactive_markers::InteractiveMarkerServer* server,  callback_function pFunc) {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "target";
    int_marker.description = "Target location for the drone";

    this->makeBoxControl(int_marker, !awaitConfirm);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    if (fixed){
        int_marker.name += "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if (interaction_mode != InteractiveMarkerControl::NONE){
        std::string mode_text;

        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
        int_marker.name += "_" + mode_text;
        int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
    }

    if(show_6dof){
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    if(awaitConfirm){
        InteractiveMarkerControl buttonControl;
        buttonControl.name = "OK?";
        buttonControl.interaction_mode = InteractiveMarkerControl::BUTTON;
        buttonControl.always_visible = true;
        Marker marker = makeBox(int_marker);
        buttonControl.markers.push_back(marker);
        int_marker.controls.push_back(buttonControl);
    }
    server->insert(int_marker, pFunc);

    /*if(interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE){
        menu_handler.apply(* server, int_marker.name);
    }*/
}
