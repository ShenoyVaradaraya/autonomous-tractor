/**
 * @file vel_prof.cpp
 * @author Varadaraya Ganesh Shenoy (varadaraya.shenoy@fluxauto.xyz)
 * @brief cpp file for velocity profile generation.
 * @version 1.0
 * @date 10/07/2021
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "boustrophedon_server/vel_prof.h"

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
gen_vel::gen_vel()
    : velParams(std::make_tuple(60.0, 10.0, 5.0, -2.5, 5.0, 3.0)),
      stopParams(std::make_tuple(3.0, 3.0, 0.1, 4, 2)) {}
float gen_vel::round2(float num) {
    num = std::ceil(num * 100);
    num = num / 100;

    return num;
}
std::vector<double> gen_vel::check_curve(std::vector<Node> path) {
    std::vector<double> curveVec(path.size(), 0);
    double X[3], Y[3];
    for (int i = 1; i + 1 < path.size(); i++) {
        X[0] = path[i - 1].x;  // Taking the (i-1)th x-waypoint
        X[1] = path[i].x;      // Taking the (i)th x-waypoint
        X[2] = path[i + 1].x;  // Taking the (i+1)th x-waypoint

        Y[0] = path[i - 1].y;  // Taking the (i-1)th y-waypoint
        Y[1] = path[i].y;      // Taking the (i)th y-waypoint
        Y[2] = path[i + 1].y;  // Taking the (i+1)th y-waypoint

        Eigen::Matrix3f Xmat;  // Defintion of X-matrix
        Eigen::MatrixXf Ymat(3, 1),
            coefMat(3, 1);  // Defintion of Y-matrix and Coefficient matrix

        Xmat << pow(X[0], 2), pow(X[0], 1), 1, pow(X[1], 2), pow(X[1], 1), 1,
            pow(X[2], 2), pow(X[2], 1), 1;

        // Forming the Y-matrix
        Ymat << Y[0], Y[1], Y[2];

        // Calculating the determinant, finding the coefficients and pushing the
        // a-coefficient into a vector
        
        if (abs(Xmat.determinant()) > 0.001) {
            coefMat = Xmat.inverse() * Ymat;
            curveVec.push_back(coefMat(0, 0));
        } else {
            curveVec.push_back(0);
        }
    }
    return curveVec;  // Returning the vector of a-coefficients for a local path
                      // segment
}

void gen_vel::assign_velocity(std::vector<Node> path,
                              std::vector<Node> globalPath,
                              Node current_pose, bool curve_cnt, bool obs,
                              bool recovery) {
    double x = current_pose.x - globalPath[globalPath.size() - 1].x;
    double y = current_pose.y - globalPath[globalPath.size() - 1].y;
    double dist = sqrt(abs((x * x) + (y * y)));

    if (recovery) {
        if (dist < 0.1) {
            for (int i = 0; i < path.size(); i++) {
                vel = 0;
                path[i].velocity = vel;
                speed_arr.push_back(vel);
            }
        } else {
            for (int i = 0; i < path.size(); i++) {
                vel = std::get<5>(velParams);  // Constant velocity
                path[i].velocity = vel;
            }
        }
    } else {
        speedProfile(path, globalPath, current_pose, curve_cnt, obs, dist);
    }
}

void gen_vel::velocity_points(std::vector<Node> path,
                              std::vector<Node> globalPath,
                              Node current_pose, bool obs, bool recovery) {
    curve_cnt = false;
    std::vector<double> curve = check_curve(path);
    //std::vector<double> curve2 = check_curve(*dwa_path);

    // Check if end-point of segment has shifted
    currX = path[path.size() - 1].x;
    currY = path[path.size() - 1].y;

    // If shifted, find profile otherwise publish previous planned velocity
    // points
    if (prevX != currX || prevY != currY) {
        find_profile = true;
        prevX = currX;
        prevY = currY;
    } else {
        find_profile = false;
    }

    // Check for curves for path from front point
    for (int i = 0; i < curve.size(); i++) {
        if (abs(curve[i]) > 0) {
            curve_cnt = true;  // if a-coefficient is greater than 0.001 , curve
                               // is encountered.
            break;
        }
    }
    // Assign velocity points based on curve flag
    if (curve_cnt) {
        assign_velocity(path, globalPath, current_pose, true, obs,
                        recovery);
    } else {
        assign_velocity(path, globalPath, current_pose, false, obs,
                        recovery);
    }
    curve.clear();
    speed_arr.clear();
}

void gen_vel::checkShift(float endX, float endY, std::vector<Node> path) {
    if (endX != path[0].x || endY != path[0].y) {
        find_profile = true;
        endX = path[0].x;  // X-Endpoint of local path segment
        endY = path[0].y;  // Y-Endpoint of local path segment
    } else {
        find_profile = false;
    }
}

void gen_vel::speedProfile(std::vector<Node> path,
                           std::vector<Node> globalPath,
                           Node current_pose, bool curve_cnt, bool obs,
                           double dist) {
    if (obs) {
        post_obs = 0;
        for (int i = 0; i < path.size(); i++) {
            path[i].velocity = 0;
        }
    } else {
        if (dist < std::get<1>(stopParams) ||
            path.size() < std::get<3>(stopParams)) {
            checkShift(endX, endY, path);
            if (find_profile) {
                if (dist < std::get<2>(stopParams) ||
                    path.size() <= std::get<4>(stopParams)) {
                    for (int i = 0; i < path.size(); i++) {
                        vel = 0;
                        path[i].velocity = vel;
                        speed_arr.push_back(vel);
                    }
                } else {
                    for (int i = 0; i < path.size(); i++) {
                        vel =
                            round2(current_vel -
                                   1.5 * (current_vel /
                                          path.size()));  // Slow deceleration
                        if (vel < std::get<0>(stopParams))
                            vel = std::get<0>(stopParams);
                        path[i].velocity = vel;
                        speed_arr.push_back(vel);
                    }
                    current_vel = speed_arr[1];
                }

            } else {
                for (int i = 0; i < path.size(); i++) {
                    path[i].velocity = speed_arr[i];
                }
                current_vel = speed_arr[1];
            }
        } else {
            if (start) {
                //std::cout << start << "----> " << std::endl;
                for (int i = 0; i <= path.size() - 1; i++) {
                    vel =
                        round2(((std::get<0>(velParams) - 0.0) / path.size()) *
                               (i + 1) * 1.0);  // Slow acceleration
                    path[i].velocity = vel;
                    //std::cout << path[i].velocity << "inside start" << std::endl;
                    speed_arr.push_back(vel);
                    current_vel = vel;
                }

                current_vel = speed_arr[0];
                //std::cout << current_vel << "---->>>>>>>>" << std::endl;

            } else {
                if (find_profile) {
                    for (int i = 0; i < path.size(); i++) {
                        if (curve_cnt) {
                            if (current_vel < std::get<2>(velParams)) {
                                vel = current_vel;  // Gradual decrease to min.
                                                    // velocity if curve
                                path[i].velocity = vel;
                                current_vel = vel;
                                speed_arr.push_back(vel);
                            } else {
                                vel = round2(
                                    current_vel +
                                    std::get<3>(velParams) *
                                        (current_vel /
                                         path.size()));  // Gradual decrease to
                                                          // min. velocity if
                                                          // curve
                                if (vel < std::get<2>(velParams))
                                    vel = std::get<2>(velParams);
                                path[i].velocity = vel;
                                current_vel = vel;
                                speed_arr.push_back(vel);
                            }

                        } else {
                            if (current_vel >= std::get<2>(velParams)) {
                                vel = round2(
                                    current_vel +
                                    abs(std::get<3>(velParams)) *
                                        (current_vel /
                                         path.size()));  // Gradual increase to
                                                          // max. velocity if
                                                          // straight
                                if (vel > std::get<1>(velParams))
                                    vel = std::get<1>(velParams);
                                path[i].velocity = vel;
                                current_vel = vel;
                                speed_arr.push_back(vel);
                            } else {
                                vel = round2(
                                    current_vel +
                                    std::get<4>(velParams) *
                                        (current_vel /
                                         path.size()));  // Gradual increase to
                                                          // max. velocity if
                                                          // straight
                                if (vel > std::get<1>(velParams))
                                    vel = std::get<1>(velParams);
                                path[i].velocity = vel;
                                current_vel = vel;
                                speed_arr.push_back(vel);
                            }
                        }
                        find_profile = false;
                    }
                    current_vel = speed_arr[1];
                } else {
                    for (int i = 0; i < path.size(); i++) {
                        path[i].velocity = speed_arr[i];
                    }
                }
            }
        }
    }
}
