#ifndef BEZIER_H_
#define BEZIER_H_
#include "utilities.h"
typedef long double ullint;

class BezierFit
{
public:
  BezierFit(){};
  ullint fact(int);
  ullint nCa(int, int);
  void bezierCurveFit(std::vector<Node>& pts, std::shared_ptr<std::vector<Node>>& pts_fit);
  void interpolate(Node n1, Node n2, int nbr_points, std::vector<Node>& points);
  std::vector<int> find_turns(std::vector<Node>);
  std::vector<Node> smooth(std::vector<Node>, int, int);
  std::vector<Node> update_path(std::vector<Node>, int, int, int);
};

#endif
