#include "boustrophedon_server/bezier.h"

ullint BezierFit::fact(int a)
{
  if (a < 0)
    throw std::invalid_argument("ERROR, a < 0");

  if (a == 0)
    return 1;
  else
  {
    ullint val = a;
    for (int i = a - 1; i > 0; i--)
      val *= i;

    return val;
  }
}

ullint BezierFit::nCa(int n, int a)
{
  if (a > n)
    throw std::invalid_argument("ERROR, a > n");
  else
  {
    if (fact(n - a) * fact(a) == 0)
      return fact(n - a);
    return fact(n) / (fact(n - a) * fact(a));
  }
}

void BezierFit::bezierCurveFit(std::vector<Node>& pts, std::shared_ptr<std::vector<Node>>& pts_fit)
{
  pts_fit->clear();
  for (float i = 0; i <= pts.size() - 1; i++)
  {
    float t = i / float(pts.size() - 1);

    double x = 0, y = 0;
    for (int j = 0; j < pts.size(); j++)
    {
      x = x + nCa(pts.size() - 1, j) * pow(1 - t, pts.size() - j - 1) * pow(t, j) * pts[j].x;
      y = y + nCa(pts.size() - 1, j) * pow(1 - t, pts.size() - j - 1) * pow(t, j) * pts[j].y;
    }

    Node pt = pts[i];
    pt.x = x;
    pt.y = y;

    pts_fit->push_back(pt);
  }
  for (int i = 1; i < pts_fit->size(); i++)
    (*pts_fit)[i].orien = atan2(double((*pts_fit)[i].y - (*pts_fit)[i - 1].y), double((*pts_fit)[i].x - (*pts_fit)[i - 1].x));
}

void BezierFit::interpolate(Node n1, Node n2, int nbr_points, std::vector<Node>& points)
{
  for (int i = 0; i <= nbr_points; i++)
  {
    Node pt;
    pt.x = n1.x + i * (n2.x - n1.x) / (nbr_points);
    pt.y = n1.y + i * (n2.y - n1.y) / (nbr_points);
    points.push_back(pt);
  }
}
std::vector<int> BezierFit::find_turns(std::vector<Node> path)
{
  std::vector<int> turn_points;
  turn_points.clear();
  for (int i = 1; i <= path.size(); i++)
  {
    if (abs(path[i].orien - path[i - 1].orien) >= PI / 4)
    {
      turn_points.push_back(i);
    }
  }
  return turn_points;
}
std::vector<Node> BezierFit::smooth(std::vector<Node> path, int stPt, int endPt)
{
  std::vector<int> turn_points = find_turns(path);
  for (int i = 0; i < turn_points.size(); i++)
  {
    path = update_path(path, turn_points[i], stPt, endPt);
  }

  for (int i = 1; i < path.size(); i++)
  {
    path[i].orien =
        atan2(static_cast<double>(path[i].y - path[i - 1].y), static_cast<double>(path[i].x - path[i - 1].x));
  }
  return path;
}

std::vector<Node> BezierFit::update_path(std::vector<Node> path, int turn_location, int num_points_st,
                                         int num_points_lt)
{
  int start_point = std::max(turn_location - num_points_st, 0);
  int last_point = std::min(static_cast<int>(path.size() - 1), turn_location + num_points_lt);
  std::vector<Node> smpth;
  std::shared_ptr<std::vector<Node>> smpth1 = std::make_shared<std::vector<Node>>();
  BezierFit bezfit;
  for (int i = start_point; i <= last_point; i++)
  {
    smpth.push_back(path[i]);
  }
  bezfit.bezierCurveFit(smpth, smpth1);
  int count = 0;
  for (int i = start_point; i <= last_point; i++)
  {
    path[i] = (*smpth1)[count];
    count++;
  }
  return path;
}