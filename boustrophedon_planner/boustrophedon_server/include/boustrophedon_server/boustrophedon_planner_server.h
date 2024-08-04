#ifndef SRC_BOUSTROPHEDON_PLANNER_SERVER_H
#define SRC_BOUSTROPHEDON_PLANNER_SERVER_H

#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <boustrophedon_msgs/ConvertPlanToPath.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include "boustrophedon_server/cgal_utils.h"
#include "boustrophedon_server/striping_planner.h"
#include "boustrophedon_server/outline_planner.h"
#include "cellular_decomposition/polygon_decomposer.h"
#include "boustrophedon_server/bezier.h"
#include "boustrophedon_server/utilities.h"

class BoustrophedonPlannerServer
{
public:
  BoustrophedonPlannerServer();

  void executePlanPathAction(const boustrophedon_msgs::PlanMowingPathGoalConstPtr& goal);

private:
  using Server = actionlib::SimpleActionServer<boustrophedon_msgs::PlanMowingPathAction>;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  Server action_server_;
  ros::ServiceServer conversion_server_;
  ros::Publisher initial_polygon_publisher_;
  ros::Publisher preprocessed_polygon_publisher_;
  ros::Publisher path_points_publisher_;
  ros::Publisher switch_points_publisher_;
  ros::Publisher polygon_points_publisher_;

  std::vector<Node> stripe_path, interpolated_path, erased_path;
  StripingPlanner striping_planner_;
  OutlinePlanner outline_planner_;
  BezierFit bez;

  bool repeat_boundary_{};
  bool outline_clockwise_{};
  bool skip_outlines_{};
  bool enable_orientation_{};
  int outline_layer_count_{};
  double stripe_separation_{};
  double intermediary_separation_{};
  double stripe_angle_{};
  bool travel_along_boundary_{};
  bool allow_points_outside_boundary_{};
  bool enable_half_y_turns_{};
  int points_per_turn_{};
  double turn_start_offset_{};
  bool reverse_{};
  tf::TransformListener transform_listener_{};
  bool publish_polygons_{};
  bool publish_path_points_{};

  ros::Subscriber map_subscriber_;
  nav_msgs::OccupancyGrid occ_map;

  bool convertStripingPlanToPath(boustrophedon_msgs::ConvertPlanToPath::Request& request,
                                 boustrophedon_msgs::ConvertPlanToPath::Response& response);
  boustrophedon_msgs::PlanMowingPathResult toResult(std::vector<NavPoint>&& path, const std::string& frame) const;
  Polygon fromBoundary(const geometry_msgs::PolygonStamped& boundary) const;
  Point fromPositionWithFrame(const geometry_msgs::PoseStamped& pose, const std::string& target_frame) const;
  bool checkPolygonIsValid(const Polygon& poly) const;
  double getStripeAngleFromOrientation(const geometry_msgs::PoseStamped& robot_position);
  geometry_msgs::PolygonStamped convertCGALPolygonToMsg(const Polygon& poly) const;
  void publishPathPoints(const std::vector<NavPoint>& path);
  void publishPolygonPoints(const Polygon& poly) const;
  void map_callback(const nav_msgs::OccupancyGridConstPtr&);
  bool check_collision(Node);
  void navpoint_to_node(const std::vector<NavPoint>&);
  void trace_path(bool, bool, int, int, bool, const std::vector<NavPoint>&);
  void go_home_forward(const std::vector<NavPoint>&);

  void execute_fishtail_turn(const std::vector<NavPoint>&);
  void convert_to_bezier(std::vector<Node>);
  void reduce_interpolation(std::vector<Node>);
  void check_collision_globally(std::vector<Node>);
  void convert_to_nav_msgs(std::vector<Node>);

  int map_width, map_height, i = 0, j = 0;
  float origin_x, origin_y, origin_orien, map_res;
  double a,b,radius;

  bool bottom_up = true, top_down = false, start = true;

  std::vector<Node> bottom_points_list, top_points_list,bezier_path,final_path;
  geometry_msgs::Pose switch_direction_point_to_forward, switch_direction_point_to_reverse;
};

#endif  // SRC_BOUSTROPHEDON_PLANNER_SERVER_H
