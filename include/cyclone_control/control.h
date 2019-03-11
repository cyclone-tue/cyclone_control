//
// Created by stijn on 2-3-19.
//

#ifndef CYCLONE_CONTROL_CONTROL_H
#define CYCLONE_CONTROL_CONTROL_H

#include <mavros_msgs/State.h>

std::vector<tf::Stamped<tf::Point>> path;
std::vector<tf::Stamped<tf::Point>>::iterator pathIterator;
tf::Stamped<tf::Pose>* currentPose = nullptr;
bool ready = false;
bool targetSet = false;

void posCallback(const geometry_msgs::PoseStamped::ConstPtr& position);
void broadcastTransform(const geometry_msgs::PoseStamped::ConstPtr& position);
void pathCallback(const nav_msgs::Path::ConstPtr& path);
void stateCallback(const mavros_msgs::State::ConstPtr& state);

#endif //CYCLONE_CONTROL_CONTROL_H
