#ifndef BOUSTROPHEDON_SERVER_LOCAL_PLANNER_H
#define BOUSTROPHEDON_SERVER_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

#include "boustrophedon_server/kdtree.h"
#include "boustrophedon_server/utilities.h"
#include "boustrophedon_server/bezier.h"
#include "boustrophedon_server/vel_prof.h"

class local_planner
{
public:
  local_planner();
  void driver();

private:
  void callback_current_pose_covariance(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
  void callback_current_pose_stamped(const geometry_msgs::PoseStampedConstPtr&);
  void callback_current_pose_stamped_vehicle(const geometry_msgs::PoseStampedConstPtr&);
  void callback_global_path(const nav_msgs::PathConstPtr&);
  void callback_switch_points(const geometry_msgs::PoseArrayConstPtr&);
  float check_distance(Node,Node);
  std::vector<Node> generate_segment(std::vector<Node>,Node);
  void gen_local_path_segment();
  void set_direction();
  void status_check();
  int genKDTree(int, KDT::KDTree<KDT::MyPoint>*, std::vector<Node>);
  void gen_local_path(int, int, std::vector<Node>*);
  void check_covered_percent(std::vector<Node>,std::vector<Node>);
  void publish_path(std::vector<Node>*);

  ros::NodeHandle nh;
  ros::Subscriber sub_current_pose_covariance, sub_current_pose_stamped, sub_global_path,sub_current_pose_stamped_vehicle,sub_switch_points;
  ros::Publisher pub_path, pub_speed;
  std_msgs::Float32 speed_msg;

  std::vector<Node> global_path_node,local_path, path, local_path_smooth, path_metric,
      global_path_segment, path_vel;
  Node direction_switch_rev, direction_switch_frwd,goal;
  gen_vel vp_ceres;
  Node curr_loc;
  KDT::KDTree<KDT::MyPoint> kdt;
  KDT::KDTree<KDT::MyPoint> global_kdt;

  BezierFit bezfit;

  float endX = 0;
  float endY = 0;
  float segments_covered = 0;
  float coverage_percentage = 0;

  int ind = 0;
  int global_ind = 0;
  int global_idx = 0;
  int i = 0;
  int begin = 0;
  int segment_size = 30;
  int endInd = 12312312;
  int global_endIdx = 12312312;
  int reverse_count = 0;

  bool path_received = false;
  bool kd_tree_gen = false;
  bool remove_path_point = false;
  bool segment_done = true;
  bool approaching_goal = false;
  bool homing = false;
  bool _reverse = false;
  bool direction = true;
  bool auto_init = true;
  bool reverse_set = false;
  

  std::string node_name = "local_planner";
  std::string vehicle_mode = "";
  std::string previous_mode = "";
};


#endif  // BOUSTROPHEDON_SERVER_LOCAL_PLANNER_H
