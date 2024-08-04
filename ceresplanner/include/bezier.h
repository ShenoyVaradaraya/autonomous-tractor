/**
 * @file bezier.h
 * @author Path Planning team
 * @brief header file for fitting a bezier curve on the set of control points
 * @version 1.0
 * @date 21/02/2021
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INCLUDE_BOPT_WAREHOUSE_BEZIER_H_
#define INCLUDE_BOPT_WAREHOUSE_BEZIER_H_
#include <memory>
#include <vector>

#include "node.h"
typedef long double ullint;

/**
 * @brief class for fitting a bezier curve on control points
 *
 */
class BezierFit {
 public:
    /**
     * @brief constructor for class
     *
     */
    BezierFit() {}

    /**
     * @brief calculates the factorial.
     * @param a integer input.
     * @return unsigned long integer value
     */
    ullint fact(int a);

    /**
     * @brief helper function for fitting a curve on bezier control points
     * @param n integer type intput
     * @param a integer type intput
     * @return unsigned long integer value
     */
    ullint nCa(int n, int a);

    /**
     * @brief fits a curve on bezier control points
     * @param pts is vector of nodes
     * @param pts_fit is vector of nodes with bezier fitting
     */
    void bezierCurveFit(const std::vector<Node>& pts,
                        std::shared_ptr<std::vector<Node>> pts_fit);

    /**
     * @brief Helper function for recovery, psp and docking.
     * @param slopeVector is vector to be modified.
     * @param control_pts is vector of float.
     * @param total is costant.
     */
    void OneDbezierCurveFit(std::shared_ptr<std::vector<float>> slopeVector,
                            const std::vector<float>& control_pts,
                            const int& total = 500);
};

#endif  // INCLUDE_BOPT_WAREHOUSE_BEZIER_H_
