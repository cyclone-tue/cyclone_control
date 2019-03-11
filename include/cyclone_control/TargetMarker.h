//
// Created by stijn on 3-3-19.
//

#ifndef CYCLONE_CONTROL_TARGETMARKER_H
#define CYCLONE_CONTROL_TARGETMARKER_H


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <tf/LinearMath/Vector3.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <interactive_markers/interactive_marker_server.h>

typedef void (*callback_function)(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);

class TargetMarker{
public:
    TargetMarker(bool fixed, unsigned int interaction_mode, tf::Vector3 position, bool show_6dof, bool awaitConfirm, interactive_markers::InteractiveMarkerServer* server,  callback_function pFunc);
private:
    visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);
    visualization_msgs::InteractiveMarkerControl& makeBoxControl(visualization_msgs::InteractiveMarker &msg, bool showBox);
};


#endif //CYCLONE_CONTROL_TARGETMARKER_H
