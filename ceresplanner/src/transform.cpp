#include "node.h"
#include "bezier.h"

transform::transform(cv::Mat& image):img(image){
    map_res = 0.05;
    map_rows = img.rows;
    OX = -30;
    OY = -2;
    OOrien = 0;
    origin_x = -OX * cos(OOrien) -
               OY * sin(OOrien);
    origin_y = -OX * (-sin(OOrien)) -
               OY * cos(OOrien);
    origin_o = -OOrien;
}
void transform::transformNode(Node pt, Node& pt_transformed) {

    Node ptMapBotOrigin(pt.x * map_res, (map_rows - pt.y) * map_res, -pt.orien);
    pt_transformed.x = (ptMapBotOrigin.x - origin_x) * cos(origin_o) +
                (ptMapBotOrigin.y - origin_y) * sin(origin_o);
    pt_transformed.y = -(ptMapBotOrigin.x - origin_x) * sin(origin_o) +
                (ptMapBotOrigin.y - origin_y) * cos(origin_o);
    pt_transformed.orien = ptMapBotOrigin.orien - origin_o;
}
std::vector<Node> transform::update_path(std::vector<Node> path,
                                           int turn_location, int num_points_st,
                                           int num_points_lt) {
    int start_point = std::max(turn_location - num_points_st, 0);
    int last_point = std::min(static_cast<int>(path.size() - 1),
                              turn_location + num_points_lt);
    std::vector<Node> smpth;
    std::shared_ptr<std::vector<Node>> smpth1 =
        std::make_shared<std::vector<Node>>();

    BezierFit bezfit;
    for (int i = start_point; i <= last_point; i++) {
        smpth.push_back(path[i]);
    }
    bezfit.bezierCurveFit(smpth, smpth1);
    int count = 0;
    for (int i = start_point; i <= last_point; i++) {
        path[i] = (*smpth1)[count];
        count++;
    }
    return path;
}

std::vector<int> transform::find_turns(std::vector<Node> path) {
    std::vector<int> turn_points;
    turn_points.clear();
    for (int i = 1; i <= path.size(); i++) {
        if (abs(path[i].orien - path[i - 1].orien) >= PI / 4) {
            turn_points.push_back(i);
        }
    }
    return turn_points;
}

std::vector<Node> transform::smooth(std::vector<Node> path, int stPt,
                                      int endPt) {
    std::vector<int> turn_points = find_turns(path);
    for (int i = 0; i < turn_points.size(); i++) {
        path = update_path(path, turn_points[i], stPt, endPt);
    }
    for (int i = 1; i < path.size(); i++) {
        path[i].orien = atan2(static_cast<double>(path[i].y - path[i - 1].y),
                              static_cast<double>(path[i].x - path[i - 1].x));
    }
    return path;
}
