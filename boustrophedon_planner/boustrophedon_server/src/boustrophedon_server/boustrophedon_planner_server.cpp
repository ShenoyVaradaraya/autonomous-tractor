#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include "boustrophedon_server/boustrophedon_planner_server.h"

BoustrophedonPlannerServer::BoustrophedonPlannerServer()
  : private_node_handle_("~")
  , action_server_(node_handle_, "plan_path", boost::bind(&BoustrophedonPlannerServer::executePlanPathAction, this, _1),
                   false)
  , conversion_server_{ node_handle_.advertiseService("convert_striping_plan_to_path",
                                                      &BoustrophedonPlannerServer::convertStripingPlanToPath, this) }
{
  std::size_t error = 0;
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "repeat_boundary", repeat_boundary_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "outline_clockwise", outline_clockwise_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "skip_outlines", skip_outlines_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "outline_layer_count", outline_layer_count_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "stripe_separation", stripe_separation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "intermediary_separation", intermediary_separation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "stripe_angle", stripe_angle_));
  error += static_cast<std::size_t>(!rosparam_shortcuts::get("plan_path", private_node_handle_,
                                                             "enable_stripe_angle_orientation", enable_orientation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "travel_along_boundary", travel_along_boundary_));
  error += static_cast<std::size_t>(!rosparam_shortcuts::get(
      "plan_path", private_node_handle_, "allow_points_outside_boundary", allow_points_outside_boundary_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "enable_half_y_turns", enable_half_y_turns_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "points_per_turn", points_per_turn_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "turn_start_offset", turn_start_offset_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "publish_polygons", publish_polygons_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "publish_path_points", publish_path_points_));
  error += static_cast<std::size_t>(!rosparam_shortcuts::get("plan_path", private_node_handle_, "reverse", reverse_));
  rosparam_shortcuts::shutdownIfError("plan_path", error);

  if (intermediary_separation_ <= 0.0)
  {
    // doesn't make sense, or we don't want intermediaries. set it to double max so we can't make any intermediaries
    intermediary_separation_ = std::numeric_limits<double>::max();
  }

  if (enable_half_y_turns_ && outline_layer_count_ < 1)
  {
    if (allow_points_outside_boundary_)
    {
      ROS_WARN_STREAM("Current configuration will result in turns that go outside the boundary, but this has been "
                      "explicitly enabled");
    }
    else
    {
      // we can't do half-y-turns safely without an inner boundary layer, as the arc will stick outside of the boundary
      ROS_ERROR_STREAM("Cannot plan using half-y-turns if the outline_layer_count is less than 1! Boustrophedon "
                       "planner will not start.");
      return;
    }
  }

  striping_planner_.setParameters({ stripe_separation_, intermediary_separation_, travel_along_boundary_,
                                    enable_half_y_turns_, points_per_turn_, turn_start_offset_ });
  outline_planner_.setParameters(
      { repeat_boundary_, outline_clockwise_, skip_outlines_, outline_layer_count_, stripe_separation_ });

  action_server_.start();

  if (publish_polygons_)
  {
    initial_polygon_publisher_ = private_node_handle_.advertise<geometry_msgs::PolygonStamped>("initial_polygon", 1);
    preprocessed_polygon_publisher_ =
        private_node_handle_.advertise<geometry_msgs::PolygonStamped>("preprocessed_polygon", 1);
  }
  // mainly for use with plotJuggler, which wants the points to be put one at a time on the same topic
  if (publish_path_points_)
  {
    map_subscriber_ = private_node_handle_.subscribe("/map", 1, &BoustrophedonPlannerServer::map_callback, this);
    path_points_publisher_ = private_node_handle_.advertise<nav_msgs::Path>("/PP/GlobalPath", 1);
    switch_points_publisher_ = private_node_handle_.advertise<geometry_msgs::PoseArray>("/PP/switch_points", 1);
    polygon_points_publisher_ = private_node_handle_.advertise<geometry_msgs::PointStamped>("polygon_points", 1000);
  }
}

void BoustrophedonPlannerServer::map_callback(const nav_msgs::OccupancyGridConstPtr& map)
{
  occ_map = *map;
  map_width = map->info.width;
  map_height = map->info.height;
  map_res = map->info.resolution;

  origin_x = map->info.origin.position.x;
  origin_y = map->info.origin.position.y;
  origin_orien = 0.0;

  ROS_INFO_STREAM_ONCE("[boustrophedon_server_node]: Map Received");
}
void BoustrophedonPlannerServer::executePlanPathAction(const boustrophedon_msgs::PlanMowingPathGoalConstPtr& goal)
{
  std::string boundary_frame = goal->property.header.frame_id;
  if (enable_orientation_)
  {
    stripe_angle_ = getStripeAngleFromOrientation(goal->robot_position);
  }

  Polygon polygon = fromBoundary(goal->property);
  if (!checkPolygonIsValid(polygon))
  {
    action_server_.setAborted(Server::Result(), std::string("Boustrophedon planner does not work for polygons of "
                                                            "size "
                                                            "< 3."));
    return;
  }
  if (!polygon.is_simple())
  {
    action_server_.setAborted(Server::Result(), std::string("Boustrophedon planner only works for simple (non "
                                                            "self-intersecting) polygons."));
    return;
  }
  Point robot_position;
  try
  {
    robot_position = fromPositionWithFrame(goal->robot_position, goal->property.header.frame_id);
  }
  catch (const tf::TransformException& ex)
  {
    action_server_.setAborted(Server::Result(),
                              std::string("Boustrophedon planner failed with a tf exception: ") + ex.what());
    return;
  }
  std::vector<NavPoint> path;
  auto preprocess_transform = preprocessPolygon(polygon, robot_position, stripe_angle_);

  if (publish_polygons_)
  {
    initial_polygon_publisher_.publish(goal->property);
    preprocessed_polygon_publisher_.publish(convertCGALPolygonToMsg(polygon));
  }

  Polygon fill_polygon;
  if (!outline_planner_.addToPath(polygon, robot_position, path, fill_polygon))
  {
    action_server_.setAborted(Server::Result(), std::string("Boustrophedon planner failed because "
                                                            "outline_layer_count "
                                                            "was too large for the boundary."));
    return;
  }
  PolygonDecomposer polygon_decomposer{};
  // A print statement MUST be here, see issue #1586
  ROS_INFO_STREAM("Decomposing boundary polygon into sub-polygons...");
  polygon_decomposer.decompose(fill_polygon);
  std::vector<Polygon> sub_polygons = polygon_decomposer.getSubPolygons(robot_position);
  ROS_INFO_STREAM("Broke the boundary up into " << sub_polygons.size() << " sub-polygons");
  Polygon merged_polygon;
  Point start_position = robot_position;
  for (const auto& subpoly : sub_polygons)
  {
    // combine the new subpoly with the merged_polygon. If merged_polygon is empty, it returns the sub polygon
    merged_polygon = mergePolygons(merged_polygon, subpoly);

    // add the stripes to the path, using merged_polygon boundary to travel if necessary.
    striping_planner_.addToPath(merged_polygon, subpoly, robot_position, path);
  }
  if (travel_along_boundary_)
  {
    striping_planner_.addReturnToStart(merged_polygon, start_position, robot_position, path);
  }
  postprocessPolygonAndPath(preprocess_transform, polygon, path);
  if (publish_path_points_)  // if we care about visualizing the planned path in plotJuggler
  {
    publishPathPoints(path);
    publishPolygonPoints(polygon);
  }
  auto result = toResult(std::move(path), boundary_frame);
  action_server_.setSucceeded(result);
}

boustrophedon_msgs::PlanMowingPathResult BoustrophedonPlannerServer::toResult(std::vector<NavPoint>&& path,
                                                                              const std::string& frame) const
{
  boustrophedon_msgs::PlanMowingPathResult result;
  result.plan.points.reserve(path.size());
  result.plan.header.stamp = ros::Time::now();
  result.plan.header.frame_id = frame;

  for (const auto& point : path)
  {
    boustrophedon_msgs::StripingPoint stripe_point{};
    stripe_point.point.x = point.point.x();
    stripe_point.point.y = point.point.y();
    stripe_point.point.z = 0;
    stripe_point.type = static_cast<uint8_t>(point.type);
    result.plan.points.emplace_back(stripe_point);
  }
  return result;
}

Polygon BoustrophedonPlannerServer::fromBoundary(const geometry_msgs::PolygonStamped& boundary) const
{
  Polygon polygon;
  for (const auto& point : boundary.polygon.points)
  {
    polygon.push_back(Point(point.x, point.y));
  }
  return polygon;
}

Point BoustrophedonPlannerServer::fromPositionWithFrame(const geometry_msgs::PoseStamped& pose,
                                                        const std::string& target_frame) const
{
  geometry_msgs::PoseStamped transformed_pose;
  transform_listener_.transformPose(target_frame, pose, transformed_pose);
  return { transformed_pose.pose.position.x, transformed_pose.pose.position.y };
}

bool BoustrophedonPlannerServer::convertStripingPlanToPath(boustrophedon_msgs::ConvertPlanToPath::Request& request,
                                                           boustrophedon_msgs::ConvertPlanToPath::Response& response)
{
  response.path.header.frame_id = request.plan.header.frame_id;
  response.path.header.stamp = request.plan.header.stamp;

  std::transform(request.plan.points.begin(), request.plan.points.end(), response.path.poses.begin(),
                 [&](const boustrophedon_msgs::StripingPoint& point) {
                   geometry_msgs::PoseStamped pose;
                   pose.header.frame_id = request.plan.header.frame_id;
                   pose.header.stamp = request.plan.header.stamp;
                   pose.pose.position = point.point;
                   pose.pose.orientation.x = 0.0;
                   pose.pose.orientation.y = 0.0;
                   pose.pose.orientation.z = 0.0;
                   pose.pose.orientation.w = 1.0;
                   return pose;
                 });
  return true;
}

geometry_msgs::PolygonStamped BoustrophedonPlannerServer::convertCGALPolygonToMsg(const Polygon& poly) const
{
  geometry_msgs::PolygonStamped stamped_poly;

  for (auto it = poly.vertices_begin(); it != poly.vertices_end(); it++)
  {
    geometry_msgs::Point32 point;
    point.x = float(it->x());
    point.y = float(it->y());
    point.z = float(0.0);
    stamped_poly.polygon.points.push_back(point);
  }

  stamped_poly.header.frame_id = "map";
  stamped_poly.header.stamp = ros::Time::now();
  return stamped_poly;
}

bool BoustrophedonPlannerServer::checkPolygonIsValid(const Polygon& poly) const
{
  return !(poly.size() < 3);  // expand later if we find more cases of invalid polygons
}

// get the yaw from the robot_position part of the given goal
double BoustrophedonPlannerServer::getStripeAngleFromOrientation(const geometry_msgs::PoseStamped& robot_position)
{
  tf::Quaternion quat(robot_position.pose.orientation.x, robot_position.pose.orientation.y,
                      robot_position.pose.orientation.z, robot_position.pose.orientation.w);
  tf::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO_STREAM("Got Striping Angle from Orientation: " << yaw);
  // TODO: Recalibrate the IMU so that we can get rid of this constant below.
  return yaw + M_PI / 2;  // Adds PI / 2 to account for incorrect IMU calibration / reference vector
}

bool BoustrophedonPlannerServer::check_collision(Node n)
{
  cv::Mat gridmap = cv::imread("/home/varadaraya-shenoy/RnD1/ceres/src/map_server_thor/maps/maps/"
                               "ceres_oriented_holes.pgm");
  // cv::imshow("map",grid_map);
  // cv::waitKey(0);
  int index = floor((n.x - origin_x) / map_res) + floor((n.y - origin_y) / map_res) * map_width;
  return ((occ_map.data[index]) == 100);
}
// publishes the path points one at a time for visualization in plotJuggler
void BoustrophedonPlannerServer::publishPathPoints(const std::vector<NavPoint>& path)
{
  //std::cout << "Number of stripe points : " << path.size() << std::endl;

  std::vector<Node> bottom_points_list, top_points_list;
  for (NavPoint point : path)
  {
    Node stripe_point;
    stripe_point.x = point.point.x();
    stripe_point.y = point.point.y();
    stripe_point.orien = atan(point.point.y() / point.point.x());
    // stripe_path.push_back(stripe_point);
    // std::cout << stripe_point.x << ", " << stripe_point.y << std::endl;
    if ((stripe_point.y - 14.2 < 0.01))
    {
      bottom_points_list.push_back(stripe_point);
    }
    else
    {
      top_points_list.push_back(stripe_point);
    }
  }

  // std::cout << bottom_points_list.size() << ", " << top_points_list.size() << std::endl;
  // Node stripe_pt_end;
  // stripe_pt_end.x = 0.97;
  // stripe_pt_end.y = 82;
  // top_points_list.push_back(stripe_pt_end);
  // stripe_pt_end.x = 2.7;
  // stripe_pt_end.y = 5.68;
  // bottom_points_list.push_back(stripe_pt_end);

  int i = 0;
  int j = 0;
  bool bottom_up = true;
  bool top_down = false;
  bool start = true;
  while (i < 4)
  {
    if (bottom_up && !top_down)
    {
      stripe_path.push_back(bottom_points_list[i]);
      stripe_path.push_back(top_points_list[i]);
      if (start)
      {
        j = ((path.size() + 2) / 4);
        i = i + j;
      }
      else
      {
        j = ((path.size() + 4) / 4);
        i = i + j;
      }
      double a = (top_points_list[i - j].x + top_points_list[i].x) / 2;
      double b = (top_points_list[i - j].y + top_points_list[i].y) / 2;
      double radius = sqrt(pow((top_points_list[i].x - a), 2) + pow((top_points_list[i].y - b), 2));
      for (float theta = 180; theta > 0; theta--)
      {
        Node n;
        // std::cout << theta << std::endl;
        n.x = a + radius * cos(theta * M_PI / 180);
        n.y = b + radius * sin(theta * M_PI / 180);
        stripe_path.push_back(n);
      }
      bottom_up = false;
      top_down = true;
    }
    if (top_down && !bottom_up)
    {
      stripe_path.push_back(top_points_list[i]);
      stripe_path.push_back(bottom_points_list[i]);
      if (start)
      {
        j = ((path.size() + 2) / 4);
        i = i - j;
      }
      else
      {
        j = ((path.size() + 2) / 4);
        i = i - j;
      }

      double a1 = (bottom_points_list[i + j].x + bottom_points_list[i].x) / 2;
      double b1 = (bottom_points_list[i + j].y + bottom_points_list[i].y) / 2;
      double radius1 = sqrt(pow((bottom_points_list[i].x - a1), 2) + pow((bottom_points_list[i].y - b1), 2));
      for (float theta = 0; theta > -180; theta--)
      {
        Node n;
        // std::cout << theta << std::endl;
        n.x = a1 + radius1 * cos(theta * M_PI / 180);
        n.y = b1 + radius1 * sin(theta * M_PI / 180);
        stripe_path.push_back(n);
      }
      bottom_up = true;
      top_down = false;
      if (start)
      {
        i = 0;
        start = false;
      }
    }
    else
    {
      stripe_path.push_back(bottom_points_list[i]);
      bottom_up = true;
    }
  }

  if (!reverse_)
  {
    stripe_path.push_back(top_points_list[((path.size() - 2) / 4)]);
    double a = (top_points_list[((path.size() - 2) / 4)].x + top_points_list[top_points_list.size() - 1].x) / 2;
    double b = (top_points_list[((path.size() - 2) / 4)].y + top_points_list[top_points_list.size() - 1].y) / 2;
    double radius = sqrt(pow((top_points_list[top_points_list.size() - 1].x - a), 2) +
                         pow((top_points_list[top_points_list.size() - 1].y - b), 2));
    for (float theta = 180; theta > 0; theta--)
    {
      Node n;
      // std::cout << theta << std::endl;
      n.x = a + radius * cos(theta * M_PI / 180);
      n.y = b + radius * sin(theta * M_PI / 180);
      stripe_path.push_back(n);
    }

    stripe_path.push_back(top_points_list[top_points_list.size() - 1]);
    stripe_path.push_back(bottom_points_list[bottom_points_list.size() - 1]);

    double a5 =
        (bottom_points_list[((path.size() - 2) / 4)].x + bottom_points_list[bottom_points_list.size() - 1].x) / 2;
    double b5 =
        (bottom_points_list[((path.size() - 2) / 4)].y + bottom_points_list[bottom_points_list.size() - 1].y) / 2;
    double radius5 = sqrt(pow((bottom_points_list[bottom_points_list.size() - 1].x - a5), 2) +
                          pow((bottom_points_list[bottom_points_list.size() - 1].y - b5), 2));
    for (float theta = 0; theta > -90; theta--)
    {
      Node n;
      // std::cout << theta << std::endl;
      n.x = a5 + radius5 * cos(theta * M_PI / 180);
      n.y = b5 + radius5 * sin(theta * M_PI / 180);
      stripe_path.push_back(n);
    }

    for (unsigned int i = 0; i < 200; i++)
    {
      Node forward_n;
      forward_n.x = stripe_path[stripe_path.size() - 1].x - 0.05;
      forward_n.y = stripe_path[stripe_path.size() - 1].y;
      stripe_path.push_back(forward_n);
    }

    double a_new = (bottom_points_list[0].x + bottom_points_list[5].x) / 2;
    double b_new = (bottom_points_list[0].y + bottom_points_list[5].y) / 2;
    double radius_new = sqrt(pow((bottom_points_list[5].x - a_new), 2) + pow((bottom_points_list[5].y - b_new), 2));
    for (float theta = -90; theta > -180; theta--)
    {
      Node n;
      // std::cout << theta << std::endl;
      n.x = a_new + radius_new * cos(theta * M_PI / 180);
      n.y = b_new + radius_new * sin(theta * M_PI / 180);
      stripe_path.push_back(n);
    }
    stripe_path.push_back(bottom_points_list[0]);
  }

  else
  {
    geometry_msgs::PoseArray switch_points;
    switch_points.header.frame_id = "map";
    switch_points.header.stamp = ros::Time::now();
    stripe_path.push_back(top_points_list[((path.size() - 2) / 4)]);
    double a = (top_points_list[((path.size() - 2) / 4)].x + top_points_list[top_points_list.size() - 1].x) / 2;
    double b = (top_points_list[((path.size() - 2) / 4)].y + top_points_list[top_points_list.size() - 1].y) / 2;
    double radius = sqrt(pow((top_points_list[top_points_list.size() - 1].x - a), 2) +
                         pow((top_points_list[top_points_list.size() - 1].y - b), 2));
    for (float theta = 180; theta > 90; theta--)
    {
      Node n;
      // std::cout << theta << std::endl;
      n.x = a + radius * cos(theta * M_PI / 180);
      n.y = b + radius * sin(theta * M_PI / 180);
      stripe_path.push_back(n);
    }

    for (unsigned int i = 0; i < 100; i++)
    {
      Node forward_n;
      forward_n.x = stripe_path[stripe_path.size() - 1].x + 0.05;
      forward_n.y = stripe_path[stripe_path.size() - 1].y;
      stripe_path.push_back(forward_n);
    }


    switch_direction_point_to_reverse.position.x = stripe_path[stripe_path.size() - 20].x;
    switch_direction_point_to_reverse.position.y = stripe_path[stripe_path.size() - 20].y;
    switch_points.poses.push_back(switch_direction_point_to_reverse);

    for (unsigned int i = 0; i < 400; i++)
    {
      Node reverse_n;
      reverse_n.x = stripe_path[stripe_path.size() - 1].x - 0.05;
      reverse_n.y = stripe_path[stripe_path.size() - 1].y;
      stripe_path.push_back(reverse_n);
    }

    switch_direction_point_to_forward.position.x = stripe_path[stripe_path.size() - 10].x;
    switch_direction_point_to_forward.position.y = stripe_path[stripe_path.size() - 10].y;
    switch_points.poses.push_back(switch_direction_point_to_forward);

    switch_points_publisher_.publish(switch_points);

    double a_new = (top_points_list[0].x + top_points_list[5].x) / 2;
    double b_new = (top_points_list[0].y + top_points_list[5].y) / 2;
    double radius_new = sqrt(pow((top_points_list[5].x - a_new), 2) + pow((top_points_list[5].y - b_new), 2));
    for (float theta = 90; theta > 0; theta--)
    {
      Node n;
      // std::cout << theta << std::endl;
      n.x = a_new + radius_new * cos(theta * M_PI / 180);
      n.y = b_new + radius_new * sin(theta * M_PI / 180);
      stripe_path.push_back(n);
    }
    stripe_path.push_back(top_points_list[5]);
    stripe_path.push_back(bottom_points_list[5]);

    double a_new1 = (bottom_points_list[0].x + bottom_points_list[5].x) / 2;
    double b_new1 = (bottom_points_list[0].y + bottom_points_list[5].y) / 2;
    double radius_new1 = sqrt(pow((bottom_points_list[5].x - a_new1), 2) + pow((bottom_points_list[5].y - b_new1), 2));
    for (float theta = 0; theta > -180; theta--)
    {
      Node n;
      // std::cout << theta << std::endl;
      n.x = a_new1 + radius_new1 * cos(theta * M_PI / 180);
      n.y = b_new1 + radius_new1 * sin(theta * M_PI / 180);
      stripe_path.push_back(n);
    }
  }
  Node goal;
  goal.x = -24.7;
  goal.y = 11.7;
  stripe_path.push_back(goal);
  for (long unsigned int k = 0; k < stripe_path.size() - 1; k++)
  {
    // std::cout << stripe_path[k].x << "," << stripe_path[k+1].x << std::endl;
    bez.interpolate(stripe_path[k], stripe_path[k + 1], 120, interpolated_path);
  }
  std::vector<Node> bezier_path = bez.smooth(interpolated_path, 35, 35);
  std::vector<Node> final_path;
  for (int i = 0; i < bezier_path.size(); i++)
  {
    for (int j = i + 1; j < bezier_path.size(); j++)
    {
      double dist = sqrt(pow((bezier_path[j].x - bezier_path[i].x), 2) + pow((bezier_path[j].y - bezier_path[i].y), 2));
      if (dist >= 0.5)
      {
        final_path.push_back(bezier_path[j]);
        i = j;
        break;
      }
    }
  }

  // for (int i = 0; i < final_path.size(); i++)
  //{
  //   bool collision_check = check_collision(final_path[i]);
  //   if (collision_check)
  //   {

  //    erased_path.insert(erased_path.end(), final_path.begin() + i, final_path.end());
  //    final_path.erase(final_path.begin() + i, final_path.end());
  //  }
  //}

  // if (final_path.size() < erased_path.size())
  //{
  //   ROS_INFO("Erased path larger than remaining path");
  //   final_path.clear();
  //   final_path = erased_path;
  // }

  nav_msgs::Path path_coverage;
  path_coverage.header.frame_id = "map";
  path_coverage.header.stamp = ros::Time::now();
  for (Node point : final_path)
  {
    geometry_msgs::PoseStamped stripe_point;
    stripe_point.header.frame_id = "map";
    stripe_point.header.stamp = ros::Time::now();
    stripe_point.pose.position.x = point.x;
    stripe_point.pose.position.y = point.y;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, point.orien);
    stripe_point.pose.orientation = tf2::toMsg(quat);
    path_coverage.poses.push_back(stripe_point);
  }
  path_points_publisher_.publish(path_coverage);
  ros::Rate(10).sleep();
  ros::spinOnce();
}

// publishes the polygon points one at a time for visualization in plotJuggler
void BoustrophedonPlannerServer::publishPolygonPoints(const Polygon& poly) const
{
  for (auto it = poly.vertices_begin(); it != poly.vertices_end(); it++)
  {
    geometry_msgs::PointStamped point;
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    point.point.x = float(it->x());
    point.point.y = float(it->y());
    point.point.z = float(0.0);
    polygon_points_publisher_.publish(point);
    ros::spinOnce();
  }
}
