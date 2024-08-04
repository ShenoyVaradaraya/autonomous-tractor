#ifndef INCLUDE_CERES_PLANNER_NODE_
#define INCLUDE_CERES_PLANNER_NODE_

#include <iostream>
#include <opencv2/opencv.hpp>
const float PI = 3.14;

class Node
{
    public:
    Node(){}
    Node(float x_, float y_, float orien_):x(x_), y(y_), orien(orien_){}
    float x,y,orien;
};

class transform {
    public:
    transform(cv::Mat& image);
    cv::Mat img;
    float map_res;
    float map_rows;
    float OX;
    float OY;
    float OOrien;
    float origin_x;
    float origin_y;
    float origin_o;
    void transformNode(Node pt, Node& pt_transformed);
    std::vector<Node> update_path(std::vector<Node> path,
                                           int turn_location, int num_points_st,
                                           int num_points_lt);
    std::vector<int> find_turns(std::vector<Node> path);
    std::vector<Node> smooth(std::vector<Node> path, int stPt,
                                      int endPt);
};
#endif //  INCLUDE_CERES_PLANNER_NODE_
