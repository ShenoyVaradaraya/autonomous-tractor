/**
 * @file bezier.cpp
 * @author Path Planning team
 * @brief cpp file for fitting a bezier curve on the set of control points
 * @version 1.0
 * @date 21/02/2021
 * @copyright Copyright (c) 2023
 *
 */

#include "bezier.h"

ullint BezierFit::fact(int a) {
    if (a < 0) throw std::invalid_argument("ERROR, a < 0");

    if (a == 0) {
        return 1;
    } else {
        ullint val = a;
        for (int i = a - 1; i > 0; i--) val *= i;

        return val;
    }
}

ullint BezierFit::nCa(int n, int a) {
    if (a > n) {
        throw std::invalid_argument("ERROR, a > n");
    } else {
        if (fact(n - a) * fact(a) == 0) return fact(n - a);
        return fact(n) / (fact(n - a) * fact(a));
    }
}

void BezierFit::bezierCurveFit(const std::vector<Node>& pts,
                               std::shared_ptr<std::vector<Node>> pts_fit) {
    pts_fit->clear();
    for (float i = 0; i <= pts.size() - 1; i++) {
        float t = i / static_cast<float>(pts.size() - 1);

        double x = 0, y = 0;
        for (int j = 0; j < pts.size(); j++) {
            x = x + nCa(pts.size() - 1, j) * pow(1 - t, pts.size() - j - 1) *
                        pow(t, j) * pts[j].x;
            y = y + nCa(pts.size() - 1, j) * pow(1 - t, pts.size() - j - 1) *
                        pow(t, j) * pts[j].y;
        }

        Node pt = pts[i];
        pt.x = x;
        pt.y = y;

        pts_fit->push_back(pt);
    }
    for (int i = 1; i < pts_fit->size(); i++)
        (*pts_fit)[i].orien =
            atan2(static_cast<double>((*pts_fit)[i].y - (*pts_fit)[i - 1].y),
                  static_cast<double>((*pts_fit)[i].x - (*pts_fit)[i - 1].x));
}

void BezierFit::OneDbezierCurveFit(
    std::shared_ptr<std::vector<float>> slopeVector,
    const std::vector<float>& control_pts, const int& total) {
    int n = control_pts.size() - 1;
    double step = 1.0 / (total - 1);
    for (int j = 0; j < total; j++) {
        float t = step * j;
        float sum = 0;
        int i = 0;
        for (auto it2 = control_pts.begin(); it2 != control_pts.end(); ++it2) {
            sum += (nCa(n, i) * pow((1 - t), (n - i)) * pow(t, i) * (*it2));
            i++;
        }
        slopeVector->push_back(sum);
    }
}
