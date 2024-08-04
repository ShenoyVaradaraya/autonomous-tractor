#pragma once
#ifndef UTILITIES_H_
#define UTILITIES_H_
#include <iostream>
#include <sstream>
#include <vector>
#include <array>
#include <algorithm>
#include <numeric>
#include <exception>
#include <functional>
#include <cmath>
#include <math.h>
#include <climits>
#include <chrono>
#include <map>
#include <bits/stdc++.h>
#include "flux_msgs/Node.h"
#include "flux_msgs/NodeArray.h"
#include <flux_msgs/TrackingPoints.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
// #include <flux_msgs/TrackingPoints.h>
#include <ros/console.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

const float PI = 3.141592654;

class Node{
public:
    float x, y, z = 0, orien;
    float g, f;
    int ind, parent;
    float delta, velocity;
    int velConstraint = -1;
    float time;
    bool direction_change_possible;


    Node(){}
    Node(float x_, float y_, float orien_) : x(x_), y(y_), orien(orien_){}
    Node(float x_, float y_, float z_, float orien_) : x(x_), y(y_), z(z_), orien(orien_){}
    void makeNode(float x_, float y_, float orien_){
        x = x_;
        y = y_;
        orien = orien_;
    }

};
inline int inRo(float val){
    return int(round(val));
}
inline bool operator <=(const Node& Node1, const Node& Node2)
    {return Node1.f <= Node2.f;}

inline bool operator <(const Node& Node1, const Node& Node2)
    {return Node1.f < Node2.f;}

inline bool operator ==(const Node& Node1, const Node& Node2)
    {return Node1.x == Node2.x && Node1.y == Node2.y && (inRo(Node1.orien*180/PI) == inRo(Node2.orien*180/PI));}

inline bool operator >=(const Node& Node1, const Node& Node2)
    {return Node1.f >= Node2.f;}

inline bool operator >(const Node& Node1, const Node& Node2)
    {return Node1.f > Node2.f;}
    
namespace MapData{
    inline std::string MapAddress;
    inline float OccupiedThresh;
    inline float FreeThresh;
    inline bool Negate;
    inline float MapRes;
    inline float OriginX;
    inline float OriginY;
    inline float OriginOrien;
}

namespace TuningParams{
const int nPtsLocalPath = 16; //number of points for bezier smoothing of curves
const int nPtsLocalPathfront = 15; //number of points for bezier smoothing of curves
const int nPtsLocalPathback = 15; //number of points for bezier smoothing of curves
const int dialation_width = 15; //to change the widht of racks
const int dialation_height = 6; //to change the height of racks. DO NOT CHANGE
const int dialation_size = 4; //Image dialation Factor. DO NOT CHANGE
const int step_size = 10; //a_star exploration step size. DO NOT CHANGE
const int obstacle_distance = 50; //to block the path generation in opposite direction to thor's orientation
const int thor_length = 30; // thor's length in pixel value. DO NOT CHANGE
const int thor_width = 20; // thor's width in pixel value. DO NOT CHANGE
const int overshoot_value = 15; // overshoot value for the orientation stack
const float orientationWeight = 10; // to keep the path from changing frequently. DO NOT CHANGE
const int motion = 0; // 0 for forward, 1 for reverse, 2 for mixed.
const int tracking_point = 0; // 0 for front, 1 for CG, 2 for rear.
const float distance_from_fc_to_front = 0.48; // in meter
const float distance_from_fc_to_rear = 1.331; // in meter
const float distance_from_rear_to_front = 1.9; // in meter
const float total_length = 2;
const float distance_to_left = 0.385; // in meter
const float distance_to_right = 0.385; // in meter
}

#endif
