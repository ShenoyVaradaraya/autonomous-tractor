#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "boustrophedon_msgs/PlanMowingPathAction.h"
#include "boustrophedon_server/boustrophedon_planner_server.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>

boustrophedon_msgs::PlanMowingPathGoal goal;
bool startReceived = false;
bool path_generated = false;
bool eot = false;
std::string vehicle_mode = "Manual";
std::string state_ = "NA";
int number_of_iterations = 0;
geometry_msgs::PoseStamped init_pose;
std::string node_name = "boustrophedon_planner_client_node";
// geometry_msgs::PolygonStamped farm;

// void propertyCallback(const geometry_msgs::PolygonStamped::ConstPtr& field)
//{
//   if (!farmReceived)
//   {
//     farm.polygon.points.clear();
//     farm = *field;
//     farmReceived = true;
//   }
// }
void initCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& robot_pose)
{
  init_pose.header = robot_pose->header;
  init_pose.pose.position.x = robot_pose->pose.pose.position.x;
  init_pose.pose.position.y = robot_pose->pose.pose.position.y;
  startReceived = true;
}
void vehiclePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& robot_pose)
{
  init_pose.header = robot_pose->header;
  init_pose.pose.position.x = robot_pose->pose.position.x;
  init_pose.pose.position.y = robot_pose->pose.position.y;
  startReceived = true;
}

void vehiclePoseCallbackSim(const geometry_msgs::PoseStamped::ConstPtr& robot_pose)
{
  init_pose.header = robot_pose->header;
  init_pose.pose.position.x = robot_pose->pose.position.x;
  init_pose.pose.position.y = robot_pose->pose.position.y;
  startReceived = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  // create the action client
  // true causes the client to spin its own thread
  //  ros::Subscriber fieldSub = nh.subscribe("/field/polygon", 1, propertyCallback);

  ros::Subscriber initSub = nh.subscribe("/initialpose", 1, initCallback);
  ros::Subscriber vehiclePoseSub = nh.subscribe("/ML/localizer_pose", 1, vehiclePoseCallback);
  ros::Subscriber vehiclePoseSubSim = nh.subscribe("/VM/Position", 1, vehiclePoseCallbackSim);

  while (ros::ok())
  {
    // ros::param::get("/path_done",path_generated);
    ros::param::get("/end_of_trajectory", eot);
    ros::param::get("/vehicle/mode", vehicle_mode);
    ROS_INFO_STREAM("[" << node_name << "]: ====================" << vehicle_mode << "============");
    if ((!path_generated || eot) && vehicle_mode == "Auto")
    {
      ROS_INFO_STREAM_ONCE("[" << node_name << "]: Waiting for Start");
      actionlib::SimpleActionClient<boustrophedon_msgs::PlanMowingPathAction> ac("plan_path", true);
      ROS_INFO_STREAM_ONCE("[" << node_name << "]: Waiting for action server to start.");
      // wait for the action server to start
      ac.waitForServer();  // will wait for infinite time

      ROS_INFO_STREAM_ONCE("[" << node_name << "]: Action server started, sending goal.");
      // send a goal to the action
      geometry_msgs::PolygonStamped property;
      property.header.frame_id = "map";

      geometry_msgs::Point32 point;
      point.z = 0;
      double length = 25;

      property.polygon.points.clear();
      point.x = -24.8;
      point.y = 10;
      property.polygon.points.push_back(point);
      point.x = -1.78;
      point.y = 10;
      property.polygon.points.push_back(point);
      point.x = -3.1;
      point.y = 48.5;
      property.polygon.points.push_back(point);
      point.x = -24.8;
      point.y = 48.5;
      property.polygon.points.push_back(point);

      geometry_msgs::PoseStamped robot_position;
      robot_position.header.frame_id = "map";
      robot_position.pose.orientation.x = 0;
      robot_position.pose.orientation.y = 0;
      robot_position.pose.orientation.z = 0.707168;
      robot_position.pose.orientation.w = 0.707168;
      robot_position.pose.position.x = -24.8;
      robot_position.pose.position.y = 13.4;
      robot_position.pose.position.z = 0;

      goal.property = property;
      goal.robot_position = robot_position;
      ac.sendGoal(goal);

      // //wait for the action to return
      bool finished_before_timeout = ac.waitForResult(ros::Duration(5000.0));
      if (finished_before_timeout)
      {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO_STREAM_ONCE("[" << node_name << "]: Action finished: %s" << state.toString().c_str());
        number_of_iterations++;
        path_generated = true;
        ROS_INFO_STREAM_ONCE("[" << node_name << "]: ==================== Field has been covered "
                                 << number_of_iterations << "times =====================");
      }
      else
        ROS_INFO_STREAM_ONCE("[" << node_name << "]: Action did not finish before the time out.");
    }

    ROS_INFO_STREAM_ONCE("[" << node_name << "]: Planner idle");
    ros::Rate(10).sleep();
    ros::spinOnce();
  }

  // exit
  return 0;
}