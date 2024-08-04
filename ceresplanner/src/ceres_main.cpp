/**
 * @file main.cpp
 * @author Kumar Rahul Dinkar
 * @brief
 * @version 0.1
 * @date 2022-01-10
 * @copyright Copyright (c) 2022 FLUX AUTO
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fstream>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <string>

#include "bezier.h"
#include "ceresplanner.h"
#include "nav_msgs/Path.h"
#include "node.h"

/**
 * @brief Uses ceresPlanner object to plan coverage global path on the input
 * image
 *
 * @param argc
 * @param argv
 * @return int
 */

int main(int argc, char** argv) {
    ros::init(argc, argv, "PP_ceres");
    ros::NodeHandle n;
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("ceres/path", 1);

    cv::Mat image = cv::imread(
        "/home/varadaraya-shenoy/RnD1/ceres/src/ceresplanner/src/"
        "ceres_oriented.pgm",
        1);
    ceresPlanner temp;
    // std::cout<<"just before setInput()"<<std::endl;

    // Converts the input image into binary image
    cv::Mat threshold_image = temp.setInput(image);
    // std::cout<<"just before contourMapping()"<<std::endl;

    // Extracts the boundary points of the farmland with the input as binary
    // image
    std::vector<cv::Point> contourPoints = temp.contourMapping(threshold_image);
    // std::cout<<"just before hull"<<std::endl;

    /////////////////////////////////FILE-DUMPER/////////////////////////////////////////////

    std::ofstream outfile("contourPoints.txt");
    temp.dump_A(outfile, contourPoints);

    /////////////////////////////////////////////////////////////////////////////////////////

    // form lines spaced at the difference of 'h' along 'x'
    int parkingwidth =
        50;  // increase to enlarge the seperation between the lines.
    double gradientThreshold =
        5;  // This parameter is used for setting the threshold in change in y
    // tobe considered as whole in the image
    subregion plan =
        temp.straightPathDraw(contourPoints, parkingwidth, gradientThreshold);
    std::ofstream noutfile("contourPointsLines.txt");
    temp.dump_A(noutfile, plan);

    // for each line find its intersection by searching for the point with same
    // x-axis now from each intersection store two coordinates for each of the
    // line segments then pass it to skip design

    ////////////////////////////headland-creator//////////////////////////////

    int headLandSpace = 50;
    subregion plan_with_headland = temp.headLandCreator(plan, headLandSpace);
    std::ofstream nlinehoutfile("contourPointsLinesHeadLand.txt");
    temp.dump_A(nlinehoutfile, plan_with_headland);

    ////////////////////////////skip-design-creation/////////////////////////////

    designPattern DesPat = designPattern::MultiWayTwoSkip;
    std::vector<std::pair<int, int>> FinalPath =
        temp.skipTurnDesignedPath(plan_with_headland, DesPat);
    std::vector<Node> final_path;
    std::shared_ptr<std::vector<Node>> smooth_path =
        std::make_shared<std::vector<Node>>();

    transform tf(image);
    for (int i = 1; i < FinalPath.size(); i++) {
        Node n1(FinalPath[i - 1].first, FinalPath[i - 1].second, 0);
        Node n2(FinalPath[i].first, FinalPath[i].second, 0);
        Node ptShift1, ptShift2;
        tf.transformNode(n1, ptShift1);
        tf.transformNode(n2, ptShift2);
        float dist = sqrt(pow(ptShift2.x - ptShift1.x, 2) +
                          pow(ptShift2.y - ptShift1.y, 2));
        int nbr_points = static_cast<int>(dist / 2.0);
        for (int i = 0; i <= nbr_points; i++) {
            Node pt;
            pt.x = ptShift1.x + i * (ptShift2.x - ptShift1.x) / (nbr_points);
            pt.y = ptShift1.y + i * (ptShift2.y - ptShift1.y) / (nbr_points);
            pt.orien = atan2(static_cast<double>(ptShift2.y - ptShift1.y),
                             static_cast<double>(ptShift2.x - ptShift1.x));
            final_path.push_back(pt);
        }
    }
    *smooth_path = tf.smooth(final_path, 2, 2);
    // BezierFit bz;
    // bz.bezierCurveFit(final_path, smooth_path);
    // *smooth_path = final_path;
    while (ros::ok()) {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        cv::Mat img = threshold_image.clone();
        for (int i = 0; i < smooth_path->size(); i++) {
            Node pt = smooth_path->at(i);
            if (isnan(pt.x) || isnan(pt.y)) continue;
            geometry_msgs::PoseStamped pt_msg;
            pt_msg.header.frame_id = "map";
            pt_msg.header.stamp = ros::Time::now();
            pt_msg.header.seq = 0;
            pt_msg.pose.position.x = pt.x;
            pt_msg.pose.position.y = pt.y;
            tf2::Quaternion quat;
            quat.setRPY(0, 0, pt.orien);
            pt_msg.pose.orientation = tf2::toMsg(quat);
            path_msg.poses.push_back(pt_msg);
        }
        path_pub.publish(path_msg);
        ROS_INFO("path published");
        ros::Rate(1).sleep();
        ros::spinOnce();
    }
    std::ofstream fpoutfile("finalPath.txt");
    temp.dump_A(fpoutfile, *smooth_path);

    ////////////////////////////////////////////////////////////////////////////
    //  use this plan to visualise the line on python
    //  std::vector< std::vector<cv::Point> > hull =
    //  temp.convexHull(contours,threshold_image); std::cout<<"just before
    //  returning"<<std::endl;
    /////////////////////////////////FILE
    ///DUMPER/////////////////////////////////////////////
    //  std::ofstream outnextfile("hull.txt");
    //  temp.dump_A(outnextfile, hull);
    /////////////////////////////////////////////////////////////////////////////////////////
    ros::spin();
    return 0;
}