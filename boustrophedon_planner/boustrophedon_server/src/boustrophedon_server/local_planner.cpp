/**
 * @file local_planner.cpp
 * @author Varadaraya Ganesh Shenoy (varadaraya.shenoy@fluxauto.xyz)
 * @brief
 * @date 2023-03-25
 *
 *  Copyright © Flux Auto India Private Limited 2023
 *
 */

#include "boustrophedon_server/local_planner.h"

local_planner::local_planner()
{
  curr_loc.x = -12342245265;
  curr_loc.y = 0;
  curr_loc.orien = 0;
}
void local_planner::callback_current_pose_covariance(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& current_pose)
{
  curr_loc.x = current_pose->pose.pose.position.x;
  curr_loc.y = current_pose->pose.pose.position.y;
  tf::Pose tfPose;
  tf::poseMsgToTF(current_pose->pose.pose, tfPose);
  double yaw = tf::getYaw(tfPose.getRotation());
  curr_loc.orien = yaw;
}

void local_planner::callback_current_pose_stamped(const geometry_msgs::PoseStampedConstPtr& current_pose)
{
  curr_loc.x = current_pose->pose.position.x;
  curr_loc.y = current_pose->pose.position.y;
  tf::Pose tfPose;
  tf::poseMsgToTF(current_pose->pose, tfPose);
  double yaw = tf::getYaw(tfPose.getRotation());
  curr_loc.orien = yaw;
}

void local_planner::callback_current_pose_stamped_vehicle(const geometry_msgs::PoseStampedConstPtr& current_pose)
{
  curr_loc.x = current_pose->pose.position.x;
  curr_loc.y = current_pose->pose.position.y;
  tf::Pose tfPose;
  tf::poseMsgToTF(current_pose->pose, tfPose);
  double yaw = tf::getYaw(tfPose.getRotation());
  curr_loc.orien = yaw;
}

void local_planner::callback_switch_points(const geometry_msgs::PoseArrayConstPtr& sp)
{
  direction_switch_rev.x = sp->poses[0].position.x;
  direction_switch_rev.y = sp->poses[0].position.y;
  direction_switch_frwd.x = sp->poses[1].position.x;
  direction_switch_frwd.y = sp->poses[1].position.y;
}
void local_planner::callback_global_path(const nav_msgs::PathConstPtr& global_path)
{
  for (int i = 0; i < global_path->poses.size(); i++)
  {
    geometry_msgs::PoseStamped pt = global_path->poses[i];
    tf::Pose tfPose;
    tf::poseMsgToTF(pt.pose, tfPose);
    double yaw = tf::getYaw(tfPose.getRotation());
    Node n(pt.pose.position.x, pt.pose.position.y, yaw);
    n.velocity = pt.header.seq == 0 ? -1 : 1;
    global_path_node.push_back(n);
  }
  ros::param::get("/boustrophedon_server/reverse", _reverse);
  global_endIdx = genKDTree(global_idx, &global_kdt, global_path_node);
}

int local_planner::genKDTree(int ind, KDT::KDTree<KDT::MyPoint>* kdt, std::vector<Node> path)
{
  std::vector<KDT::MyPoint> kdt_pts;

  kdt_pts.push_back(KDT::MyPoint(path[ind].x, path[ind].y));

  int ind_max;
  int sgn = path[ind + 1].velocity;
  for (int i = ind + 1; i < path.size(); i++)
  {
    kdt_pts.push_back(KDT::MyPoint(path[i].x, path[i].y));
    ind_max = i;
  }

  kdt->build(kdt_pts);

  return ind_max;
}

void local_planner::gen_local_path(int ind, int endInd, std::vector<Node>* path)
{
  path->clear();
  for (int i = ind; i <= std::min(ind + TuningParams::nPtsLocalPath, endInd); i++)
  {
    Node n = global_path_segment[i];
    path->push_back(n);
  }
  path_vel.clear();
  for (int i = std::max(0, ind - TuningParams::nPtsLocalPathback);
       i <= std::min(ind + TuningParams::nPtsLocalPathfront, endInd); i++)
  {
    Node n = global_path_segment[i];
    path_vel.push_back(n);
  }
}

// Needs to be fixed
void local_planner::check_covered_percent(std::vector<Node> path, std::vector<Node> path_segment)
{
  float total_segments = std::ceil(path.size() / segment_size);
  segments_covered++;
  coverage_percentage = segments_covered / total_segments;
  coverage_percentage = coverage_percentage * 100;
  ROS_INFO_STREAM("[" << node_name << "]: Covered " << coverage_percentage << "% of global path");
}

void local_planner::publish_path(std::vector<Node>* path)
{
  nav_msgs::Path pathMsg;
  std_msgs::Float32MultiArray msg;
  pathMsg.header.frame_id = "map";
  pathMsg.header.stamp = ros::Time::now();
  for (int i = 0; i < path->size(); i++)
  {
    Node pt = path->at(i);
    geometry_msgs::PoseStamped ptMsg;
    ptMsg.header.frame_id = "map";
    ptMsg.header.stamp = ros::Time::now();
    ptMsg.header.seq = pt.velConstraint;
    ptMsg.pose.position.x = pt.x;
    ptMsg.pose.position.y = pt.y;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, pt.orien);
    ptMsg.pose.orientation = tf2::toMsg(quat);
    pathMsg.poses.push_back(ptMsg);
  }
  pub_path.publish(pathMsg);
}

float local_planner::check_distance(Node n1, Node n2)
{
  float dx = abs(n2.x - n1.x);
  float dy = abs(n2.y - n1.y);
  float cost = (dx + dy);  // * std::min(dx, dy);
  return cost;
}
void local_planner::driver()
{
  sub_current_pose_covariance =
      nh.subscribe("/initialpose", 10, &local_planner::callback_current_pose_covariance, this);
  sub_current_pose_stamped = nh.subscribe("/VM/Position", 10, &local_planner::callback_current_pose_stamped, this);
  sub_current_pose_stamped_vehicle =
      nh.subscribe("/ML/localizer_pose", 10, &local_planner::callback_current_pose_stamped_vehicle, this);
  sub_global_path = nh.subscribe("/PP/GlobalPath", 1, &local_planner::callback_global_path, this);
  sub_switch_points = nh.subscribe("/PP/switch_points", 1, &local_planner::callback_switch_points, this);
  pub_path = nh.advertise<nav_msgs::Path>("/PP/LocalPath", 1);
  pub_speed = nh.advertise<std_msgs::Float32>("/PP/Speed", 10);
  while (ros::ok())
  {
    if (global_path_node.size() != 0)
    {
      auto start_time = ros::Time::now();
      ROS_INFO_STREAM_ONCE("[" << node_name << "]: Received Global Path");
      Node goal = global_path_node[global_path_node.size() - 1];
      // std::cout << "Segment done : " << segment_done << std::endl;
      ros::param::get("/vehicle/mode", vehicle_mode);
      // if (vehicle_mode == "Manual")
      // {
      //   auto_init = true;
      // }
      // else
      // {
      if (curr_loc.x != -1234224526 && auto_init)
      {
        global_ind = global_kdt.nnSearch(KDT::MyPoint(curr_loc.x, curr_loc.y));
        auto_init = false;
      }
          // std::vector<Node> temp_global_path;
      //     for(unsigned int i = global_ind;i<global_path_node.size();i++)
      //     {
      //       temp_global_path.push_back(global_path_node[i]);
      //     }
      //     for(unsigned int i = 0;i<global_ind;i++)
      //     {
      //       temp_global_path.push_back(global_path_node[i]);
      //     }
      //     global_path_node.clear();
      //     global_path_node = temp_global_path;
      //     goal = global_path_node[global_path_node.size()-20];
      //   }
      // }
      if (segment_done && !approaching_goal)
      {
        for (int k = global_ind; k < global_ind + segment_size; k++)
        {
          global_path_segment.push_back(global_path_node[k]);
        }
        if (std::find(global_path_segment.begin(), global_path_segment.end(), goal) != global_path_segment.end())
        {
          approaching_goal = true;
          auto goal_index = std::find(global_path_segment.begin(), global_path_segment.end(), goal);
          int goal_index_int = goal_index - global_path_segment.begin();
          global_path_segment.erase(global_path_segment.begin() + goal_index_int, global_path_segment.end());
        }
        segment_done = false;
        // std::cout << global_path_segment.size() << ", " << global_path_node.size() << std::endl;
        endInd = genKDTree(ind, &kdt, global_path_segment);
        kd_tree_gen = true;
        ros::param::set("/end_of_trajectory", false);
      }
      else
      {
        // std::cout << "------------>" << ind << std::endl;
        if (kd_tree_gen && ind < endInd && curr_loc.x != -12342245265)
        {
          ind = kdt.nnSearch(KDT::MyPoint(curr_loc.x, curr_loc.y));
          gen_local_path(ind, endInd, &local_path);
          auto goal_index_local = std::find(local_path.begin(), local_path.end(), goal);

          if (goal_index_local != local_path.end())
          {
            int goal_index_int_local = goal_index_local - local_path.begin();
            local_path.erase(local_path.begin() + goal_index_int_local, local_path.end());
          }
          // local_path_smooth = bezfit.smooth(local_path, 2, 2);
          // check_covered_percent(endX,endY,local_path_smooth);
        }
        ros::param::set("/PP/direction", direction);
        if (_reverse)
        { 
          Node start_reverse(-15,24,0);
          std::cout << check_distance(curr_loc,start_reverse) << std::endl;
          if(check_distance(curr_loc,start_reverse) < 0.5)
          {
            reverse_count = 0;
            reverse_set = true;
          }
          else
          {
            if(!reverse_set)
              reverse_count = 100;
          }
          std::cout << ((kdt.nnSearch(KDT::MyPoint(direction_switch_rev.x, direction_switch_rev.y)))) << std::endl;
          if ((kdt.nnSearch(KDT::MyPoint(direction_switch_rev.x, direction_switch_rev.y))) < 2 &&
              direction && reverse_set)
          // std::cout << "Distance = " << (check_distance(curr_loc, direction_switch_rev)) << std::endl;
          // if ((check_distance(curr_loc, direction_switch_rev) < 0.5) && direction)
          {
            ros::param::set("/PP/direction", false);
            direction = false;
            reverse_count ++;
            reverse_set = false;
          }
          else if (((kdt.nnSearch(KDT::MyPoint(direction_switch_frwd.x, direction_switch_frwd.y)) < 2) &&
                    !direction))
          // else if ((check_distance(curr_loc, direction_switch_frwd) < 1.0) && !direction)
          {
            ros::param::set("/PP/direction", true);
            direction = true;
          }

          else
          {
            ros::param::set("/PP/direction", direction);
          }
        }
        if (begin == 0)
        {
          vp_ceres.start = true;
          begin++;
        }
        else
        {
          vp_ceres.start = false;
        }
        vp_ceres.velocity_points(path_vel, global_path_segment, curr_loc, false, false);
        speed_msg.data = vp_ceres.current_vel;
        publish_path(&local_path);
        pub_speed.publish(speed_msg);
        ros::Rate(10).sleep();
        if (ind >= global_path_segment.size() - 20 || approaching_goal)
        {
          if (approaching_goal)
          {
            if (ind >= global_path_segment.size() - 7)
            {
              ROS_INFO_STREAM_ONCE("[" << node_name << "]: REACHED GOAL");
              auto end_time = ros::Time::now();
              ROS_INFO_STREAM_ONCE("[" << node_name << "]: Field covered in " << start_time - end_time << "seconds");
              // ros::param::set("/path_done", true);
              ros::param::set("/end_of_trajectory", true);
              ros::param::set("/state", "GoalManager");
              homing = true;
              path_received = false;
              global_path_node.clear();
              kdt.clear();
              kd_tree_gen = false;
              begin = true;
              // ros::Duration(30).sleep();
            }
            else
            {
              segment_done = false;
            }
          }
          else
          {
            ros::param::set("/end_of_trajectory", false);
            check_covered_percent(global_path_node, global_path_segment);
            global_path_segment.clear();
            kdt.clear();
            global_ind += ind - 4;
            ind = 0;
            segment_done = true;
          }
        }
        else
        {
          segment_done = false;
        }
      }
    }
    else
    {
      ROS_INFO_STREAM_ONCE("[" << node_name << "]: Waiting for global path");
    }
    ros::spinOnce();
  }
}