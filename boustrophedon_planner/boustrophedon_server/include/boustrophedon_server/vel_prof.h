/**
 * @file vel_prof.h
 * @author Varadaraya Ganesh Shenoy (varadaraya.shenoy@fluxauto.xyz)
 * @brief header file for velocity profile generation.
 * @version 1.0
 * @date 10/07/2021
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef HOME_ROCKET_ASGARD_SRC_PATH_PLANNER_INCLUDE_BOPT_WAREHOUSE_VEL_PROF_H_
#define HOME_ROCKET_ASGARD_SRC_PATH_PLANNER_INCLUDE_BOPT_WAREHOUSE_VEL_PROF_H_
#pragma once

#include <ros/ros.h>

#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

#include "boustrophedon_server/utilities.h"

/**
 * @file vel_prof.h
 * @authors Varadaraya Ganesh Shenoy and Saksham Kukreja
 * (varadaraya.shenoy@fluxauto.xyz , saksham.kukreja@fluxauto.xyz)
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version
 *
 * @section DESCRIPTION
 *
 * The gen_vel class represents the velocity profile for a robot without
 * feedback.
 *
 */

/**
 * @brief Speed Profile Class : gen_vel
 *
 * This is a simple class to generate velocity profile for a robot which
 * increases speed on straight paths and slows down on curves.
 */
class gen_vel {
 public:
    /**
     * @brief Construct that sets the velocity and stopping parameters
     *
     */
    gen_vel();

    /**
     * @brief Return a float value rounded off to 2 decimal places.
     *
     * @return float
     */
    float round2(float);

    /**
     * @brief Calculate distance between current pose and goal pose.
     *
     * @return float
     */
    float calc_dist(float, float, float, float);

    /**
     * @brief Returns the cost of a path that determines whether the path is
     * curved or not.
     *
     * @return std::vector<double>
     */
    std::vector<double> check_curve(std::vector<Node>);

    /**
     * @brief Main function which is called by the driver function to generate
     * the desired speed profile.
     *
     * @param path : local path
     * @param globalPath : global path
     * @param current_pose : current pose of the vehicle
     * @param curve_cnt : curved or not (boolean)
     * @param obs : obstacle detected or not (boolean)
     * @param recovery : recovery mode (boolean)
     */
    void assign_velocity(std::vector<Node> path,
                         std::vector<Node> globalPath,
                         Node current_pose, bool curve_cnt, bool obs,
                         bool recovery);

    /**
     * @brief Driver function for the gen_vel class
     *
     * @param path : local path (nearest_neighbor path)
     * @param dwa_path : dwa_path
     * @param globalPath : global path
     * @param current_pose : current_pose of the vehicle
     * @param obs : obstacle detected or not (boolean)
     * @param recovery : recovery mode (boolean)
     */
    void velocity_points(std::vector<Node> path,
                         std::vector<Node> globalPath,
                         Node current_pose, bool obs, bool recovery);

    /**
     * @brief Generate constant velocity for recovery mode
     *
     * @param path : local path
     * @param dist : distance to goal
     */
    void recovery_velPlanner(std::vector<Node> path, double dist);

    /**
     * @brief Check if the local path has shifted => vehicle is moving.
     *
     * @param endX : x-coordinate of the last point of local path
     * @param endY : y-coordinate of the last point of local path
     * @param path : local path
     */

    void checkShift(float endX, float endY, std::vector<Node> path);

    /**
     * @brief Gradually stop at goal
     *
     * @param dist : distance to goal
     * @param path : local path
     * @param current_vel : current velocity of the vehicle
     */
    void arrivingatGoal(double dist, std::vector<Node> path,
                        double current_vel);

    /**
     * @brief Create speed buffer.
     *
     * @param path : local path
     * @param current_vel : current velocity of the vehicle
     */
    void speedBackup(std::vector<Node> path, double current_vel);

    /**
     * @brief Generate speed profile based on curvature of path.
     *
     * @param path : local path
     * @param globalPath : global path
     * @param current_pose : current pose of vehicle
     * @param curve_cnt : curvature (boolean)
     * @param obs : obstacle (boolean )
     * @param dist : distance to goal
     */
    void speedProfile(std::vector<Node>,
                      std::vector<Node>, Node, bool, bool,
                      double);

    /** @param speed_arr : Speed Buffer */
    std::vector<double> speed_arr;

    /** @param stopParams : Tuple representing the stopping parameters to
     * generate a gradual velocity profile */
    std::tuple<float, float, float, int, int> stopParams;

    /** @param velParams : Tuple representing the velocity parameters to
     * generate desired speed profile */
    std::tuple<float, float, float, float, float, float> velParams;

    /**
     *  @param current_vel : current velocity
     *  @param vel :
     */
    float current_vel = 0, vel = 0.0;

    /** @param prevX : previous end x-coordinate of local path*/
    float prevX = 0;

    /** @param prevY : previous end y-coordinate of local path*/
    float prevY = 0;

    /** @param currX : current end x-coordinate of local path */
    float currX;

    /** @param currY : current end y-coordinate of local path */
    float currY;

    /** @param endX : new end x-coordinate of local path */
    float endX = 0;

    /** @param endY : new end y-coordinate of local path */
    float endY = 0;

    /** @param post_obs : to start gradually after stopping due to obstacle */
    int post_obs = 0;

    /** @param start: Gradual start */
    bool start = true;

    /** @param curve_cnt: Curve or not */
    bool curve_cnt = true;

    /** @param find_profile : Profile to be calculated or not */
    bool find_profile = true;

    /** @param start_flag : To start the vehicle from ros::param. */
    bool start_flag = false;
};
#endif
