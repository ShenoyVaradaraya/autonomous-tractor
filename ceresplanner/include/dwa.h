#ifndef DWA_H
#define DWA_H

#include "ros/ros.h"
#include <stdio.h>
#include <math.h>       /* atan2 */
#include <vector>
#include <algorithm> // for max(), max_element()
#include <iostream>
#include <cmath> // for std::ceil() function
#include <Eigen/Dense>

#include "utilities.h"
#include "kdtree.h"
// #include "local_planner.h"
//#include "vel_prof.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
//#include "TF.h"
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include "flux_msgs/TrackingPoints.h"
#include "vel_prof.h"
#include "glider.h"
#include "sensor_msgs/LaserScan.h"


//  ALL THE TYPEDEFS HERE
typedef std::pair<std::vector<Node>, std::vector<Node> > pairofTrajVectors;

class dwaLocalPlanner
{
public: 
    dwaLocalPlanner();
   // ~dwaLocalPlanner();
    void getVehicleNDTPose(const geometry_msgs::PoseStamped::ConstPtr&);
    void getVehicleRVIZPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
    //void getLocalGoal(const nav_msgs::Path::ConstPtr&);
    void CallBackOdin(const flux_msgs::TrackingPoints::ConstPtr&);
    void getGlobal(const nav_msgs::Path::ConstPtr&);
    void callbackcorienpath(const nav_msgs::Path::ConstPtr&);
    void callbackrecoverypath(const nav_msgs::Path::ConstPtr&);
    void CallBackObstacle(const geometry_msgs::PoseArray::ConstPtr&);
    void setInstantCB(const std_msgs::Float64::ConstPtr&);
    void callBackESInputs(const std_msgs::Float32MultiArray::ConstPtr&);
    void callBackLidarData(const sensor_msgs::LaserScan::ConstPtr&);


    bool traj_direction_chooser();
    long double calc_to_goal_cost(std::vector<Node>&, Node&);
    long double calc_to_obs_cost(std::vector<Node>&, std::vector<Node>&);
    long double calc_to_speed_cost(double, std::pair<double, double>&, double);

    void dwa_generate_trajectory(double&, double&, std::vector<Node>&, std::vector<Node>&);
    std::pair<std::pair<double, double>, bool> Window_gen(double);
    double calc_to_speed_cost(double,std::pair<double, double>&, bool);
    int genKDTree(int ind, KDT::KDTree<KDT::MyPoint>*, std::vector<Node>);
    void publish_local_path(std::vector<Node>&);
    float calc_distance(Node, Node);
   // std::pair<std::vector<Node>, std::vector<Node> > dwa_generate_trajectory(double&, double&);

    std::vector<Node> local_path_planner(Node , Node , Node , Node , int , bool&, bool, std::vector<Node>&);
    inline long double roundd(long double cost);
    int argMaxVel(std::vector<double>&);
    int argMinVel(std::vector<double>&);
    std::vector<Node> main_dwa_loop();
    void driver();
    void assignSteering();
    void setTargetCB(const std_msgs::Float32&);
    void gen_localpath(int, int, std::vector<Node>, std::vector<Node>&,std::vector<Node>&);

    // void setDirection();
    // all the function description
    // debug
    int counter;
    int first_halfendInd, endInd, path_end_size, second_halfendInd;
    int ind, rear_ind;
    std::string NEXT_GOAL;
    bool pallet_check;
    float feedbackVelocity;
    // local_planner localPlan;

    // vehicle param
    bool startposereceived;
    bool localgoalreceived;
    bool pathReceived;
    bool corien_pathReceived;
    bool recovery_pathReceived;

    bool end_of_traj;
    bool prevAccln;
    bool KdTreeGenerated;
    std::vector<Node> globalPath;

    // all publisher and subscribers is to be defined here
    ros::Publisher pub_vel_curr;
    ros::Publisher path_publisher; 
    ros::Publisher pathLengthPub;
    ros::Publisher localpath_publisher; 
    ros::Publisher vel_publisher; 
    ros::Subscriber sub_ndt_pose;
    ros::Subscriber sub_rviz_pose;
    ros::Subscriber sub_local_path;
    ros::Subscriber sub_global_path;
    ros::Subscriber sub_odin_trackPoints; 
    ros::Subscriber sub_obs_points; 
    ros::Subscriber target_speed_sub;
    ros::Subscriber sub_corien_path;
    ros::Subscriber sub_recovery_path;
    ros::Subscriber current_speed_sub;
    ros::Subscriber sub_es_inputs;
    ros::Subscriber sub_lidar_data;
    // ABBREVIATION :: Tuning Parameters::TP
    // 1. Trajectory Param
    double pi;
    double MAX_STEERING_ANGLE; //# TP
    //steering_angle = np.arange(-MAX_STEERING_ANGLE,
    //                                   MAX_STEERING_ANGLE, 0.01)//NEEDS A CUSTOM FUNCTION
    double VELOCITY;  // TP
    double TIME;   // TP
    double DT;
    int TRACK_POINT;  // TP
    int PATH_POINT;
    int NUM_DIGIT;  // 12, 15 #TP
    int currentIndex;
    double heading_threshold;
    bool current_direction;
    bool goal_direction;
    double velocity_factor;
    double curr_x, curr_y;
    // 2. trajectory selection param
    double GOAL_COST_GAIN;
    double SPEED_COST_GAIN;
    double OBS_COST_GAIN;
    double MAX_ACC;
    double targetSpeed;
    double velocity_resolution;
    double currVelocity;
    bool targetSpeedReceived;
    float goalRegionThreshold;
    Node dw, wc, cg;
    int half_path_point;
    bool half_path;
    // 3. vehicle param
    double LENGTH;
    double LR;
    double GOAL_THRESHOLD;
    bool track_point_recieved;
    std::string state;
    bool newPath, tracking;
    Node current_pose, localGoal, stopping_node, switch_direction, goal;
    int initial_index, final_index;
    KDT::KDTree<KDT::MyPoint> kdt;
    // local_planner localPlan;
    //
    flux_msgs::TrackingPoints wc_global, dw_global, cg_global;
    // geometry_msgs::PoseStamped localGoal;
    // geometry_msgs::PoseStamped current_pose;
    // geometry_msgs::PoseWithCovarianceStamped current_pose;
    std::vector<double> steering_angle;
    std::vector<Node> obstacles;
    std::vector<Node> localPath;
    std::vector<Node> first_half_path, gb_path, second_half_path, path_vel;
    Node esPose;
    sensor_msgs::LaserScan scanMsg;
    float steeringAngle;

    //std_msgs::Float32MultiArray speed_msg;
};
#endif
