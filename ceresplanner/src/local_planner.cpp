#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "kdtree.h"


Node currLoc(-12342245265,0,0);
std::vector<Node> globalPath;
bool pathRecieved = false;
ros::Subscriber subGlobalPath, subLocalization;
ros::Publisher pubLocalPath;
int lastIndex;


void callbackGlobalPath(const nav_msgs::Path::ConstPtr& msg){
	if(!pathRecieved){
		for(int i = 0;i < msg->poses.size();i++){
			geometry_msgs::PoseStamped pt = msg->poses[i];
			tf::Pose tfPose;
			tf::poseMsgToTF(pt.pose, tfPose);
			double yaw = tf::getYaw(tfPose.getRotation());

			Node n(pt.pose.position.x, pt.pose.position.y, yaw);
			n.velocity = pt.header.seq == 0 ? -1 : 1;		// -1 for backware, 1 for forward
			globalPath.push_back(n);
			// std::cout<<globalPath.size()<<std::endl;
		}
		pathRecieved = true;
	}
}


void callbackLocalization(const geometry_msgs::PoseStamped::ConstPtr& msg){
	tf::Pose tfPose;
	tf::poseMsgToTF(msg->pose, tfPose);
	double yaw = tf::getYaw(tfPose.getRotation());

	currLoc.x = msg->pose.position.x;
	currLoc.y = msg->pose.position.y;
	currLoc.orien = yaw;
}

void callbackLocalizationInitialPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
	tf::Pose tfPose;
	tf::poseMsgToTF(msg->pose, tfPose);
	double yaw = tf::getYaw(tfPose.getRotation());

	currLoc.x = msg->pose.position.x;
	currLoc.y = msg->pose.position.y;
	currLoc.orien = yaw;
}

void callbackLocalizationInitialPose2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	tf::Pose tfPose;
	tf::poseMsgToTF(msg->pose.pose, tfPose);
	double yaw = tf::getYaw(tfPose.getRotation());

	currLoc.x = msg->pose.pose.position.x;
	currLoc.y = msg->pose.pose.position.y;
	currLoc.orien = yaw;
	std::cout<<currLoc.x<<" , "<<currLoc.y<<" , "<<currLoc.orien<<std::endl;
}

int genKDTree(int ind, KDT::KDTree<KDT::MyPoint>& kdt){
	std::vector<KDT::MyPoint> kdt_pts;
	kdt_pts.push_back(KDT::MyPoint(globalPath[ind].x, globalPath[ind].y));

	int ind_max;
	int sgn = globalPath[ind+1].velocity;		//direction of the nodes
	for(int i = ind+1;i < globalPath.size();i++){
		// if(globalPath[i].velocity*sgn < 0)
		// 	break;

		kdt_pts.push_back(KDT::MyPoint(globalPath[i].x, globalPath[i].y));
		ind_max = i;
	}

	kdt.build(kdt_pts);

	return ind_max;
}

void genLocalPath(int ind, int endInd, std::vector<Node>& path){
	path.clear();
	int sgn = globalPath[ind].velocity;

	for(int i = ind;i <= std::min(ind + TuningParams::nPtsLocalPath, endInd);i++){

		// Node n = sgn < 0 ? thTf.shiftBackToFront(globalPath[i]) : globalPath[i];

		Node n = globalPath[i];
		if(i == endInd){
			// std::cout<<"ad"<<std::endl;
			n.velConstraint = 0;			// 0 for 0 vel, 1 for maxVel;
		}
		else
			n.velConstraint = 1;

		path.push_back(n);
	}

}

void callBackObstacleDetecton(const std_msgs::Bool::ConstPtr& obs)
{
	obstacleDetected = obs->data;		
}

void publishPath(std::vector<Node>& path){

	nav_msgs::Path pathMsg;
	pathMsg.header.frame_id = "map";
	pathMsg.header.stamp = ros::Time::now();

	for(Node pt : path){
		geometry_msgs::PoseStamped ptMsg;
		ptMsg.header.frame_id = "map";
		ptMsg.header.stamp = ros::Time::now();
		ptMsg.header.seq = pt.velConstraint;
		// std::cout<<ptMsg.header.seq<<std::endl;
		ptMsg.pose.position.x = pt.x;
		ptMsg.pose.position.y = pt.y;

		tf2::Quaternion quat;
		quat.setRPY(0, 0, pt.orien);
		ptMsg.pose.orientation = tf2::toMsg(quat);

		pathMsg.poses.push_back(ptMsg);
	}

	pubLocalPath.publish(pathMsg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_planner");
	ros::NodeHandle nh;
	gen_vel velocity_planner;


	// subGlobalPath = nh.subscribe("/ThorPlanner/GlobalPath", 1, callbackGlobalPath);
	subMapData = nh.subscribe("/map_metadata", 1, callBackMapData);

	if(!nh.getParam("free_th", MapData::FreeThresh))
		throw std::invalid_argument("ERROR, Global Planner Node, couldn't load free_th ROS Param");

	if(!nh.getParam("occu_th", MapData::OccupiedThresh))
		throw std::invalid_argument("ERROR, Global Planner Node, couldn't load occu_th ROS Param");

	if(!nh.getParam("map_address", MapData::MapAddress))
		throw std::invalid_argument("ERROR, Global Planner Node, couldn't load map_address ROS Param");

	while(ros::ok()){
		ros::spinOnce();
		if(mapDataRecieved)
			break;
	}
	thTf.initialise();

	subLocalization = nh.subscribe("/Localization", 10, callbackLocalization);
	// subLocalization = nh.subscribe("/ndt_pose", 10, callbackLocalizationInitialPose);
	// subLocalization = nh.subscribe("/pose_stamped", 10, callbackLocalizationInitialPose);
	subLocalization = nh.subscribe("/initialpose", 10, callbackLocalizationInitialPose2);
	subGlobalPath = nh.subscribe("/ThorPlanner/GlobalPathNodeArray", 1, callbackGlobalPathNodeArray2);
	// subGlobalPath = nh.subscribe("/ThorPlanner/GlobalPathNodeArrayForward", 1, callbackGlobalPathNodeArray2);
	// subObstcle = nh.subscribe("/obstacle", 1, callBackObstacleDetecton);
	pubLocalPath = nh.advertise<nav_msgs::Path>("/ThorPlanner/LocalPath", 1);
	pubVelProf = nh.advertise<std_msgs::Float32MultiArray>("/speed", 1);

	while(ros::ok()){
		if(globalPath.size() != 0){
			std::cout<<"Received Global Path"<<std::endl;
			break;
		}
		else
			std::cout<<"Waiting for Global Path"<<std::endl;

		ros::spinOnce();
	}



	Node prevLoc(-127740,-121340,-131420);
	int ind = 0, prevEndInd = 0;
	lastIndex = ind;
	subObstcle = nh.subscribe("/obstacle", 1, callBackObstacleDetecton);
	int count = 0;
	std::cout<<"global path size : "<<globalPath.size()<<std::endl;
	Node goal = globalPath[globalPath.size() -1];
	while(ros::ok() && ind < globalPath.size())
	{
		ros::spinOnce();
		KDT::KDTree<KDT::MyPoint> kdt;
		
		int endInd = genKDTree(ind, kdt);

		subObstcle = nh.subscribe("/obstacle", 1, callBackObstacleDetecton);
		while(ros::ok() && ind < endInd && currLoc.x != -12342245265)
		{

			// count++;
			std::vector<Node> localPath, localPathSmoothed;
			ros::spinOnce();
			ind = kdt.nnSearch(KDT::MyPoint(currLoc.x, currLoc.y)) + prevEndInd;
			
			// std::cout<<"counter : "<<count<<std::endl;
			genLocalPath(ind, endInd, localPath);
			
			BezierFit bezfit;
			bezfit.bezierCurveFit(localPath, localPathSmoothed);
			prevLoc = currLoc;
			// velocity_planner.velocity_points(localPathSmoothed, goal);
			velocity_planner.velocity_points(localPathSmoothed);
			std_msgs::Float32MultiArray msg, MSG;
			msg.data.clear();
			// set up dimensions
			for(int i = 0; i<localPathSmoothed.size(); i++)
				{
				 	msg.data.push_back(localPathSmoothed[i].velocity);
				 	// MSG.data.push_back(localPathSmoothed[i].velocity);
				}
		
			

			pubVelProf.publish(msg);
			publishPath(localPathSmoothed);
			// pubVelProf.publish(pub_vec);
			if(ind == endInd)
				{
					prevEndInd = endInd;
					break;
				}
		ros::Rate(10).sleep();
		}
		
		// std::cout<<"location : "<<ind<<std::endl;

		kdt.clear();
		if(ind == globalPath.size()-1){
			std::cout<<"Reached Goal"<<std::endl;
			break;
		}
	}

	thTf.~TF();

	return 0;
}