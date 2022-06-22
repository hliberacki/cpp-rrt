#include <matplotlibcpp.h>
#include "rrt.hpp"

#include <iostream>

namespace plt = matplotlibcpp;

void draw_path(const planning::pathFinding::PointsList& path)
{
    std::vector<double> xs, ys;
    for(auto node: path) {
      xs.push_back(node->mCords.x);
      ys.push_back(node->mCords.y);
    }
  plt::plot(xs, ys, "-r");
}

void drawTree(const planning::pathFinding::PointsList& tree)
{
  for (auto node: tree) {
    if (node->parent) {
      plt::plot({node->mCords.x, node->parent->mCords.x}, {node->mCords.y, node->parent->mCords.y}, "-b");
    }
  }
}

int main()
{
  planning::pathFinding::Point start;
  start.x = 0;
  start.y = 3;
  start.z = 0;

  planning::pathFinding::Point goal;
  goal.x = 98;
  goal.y = 89;
  goal.z = 0;

  planning::pathFinding::Rrt pathFinder(start, goal, 2000, 3, 0.05);

  auto final_path = pathFinder.plan();
  if (!final_path) {
    std::cout << "path not found\n";
    return 0;
  }

  plt::figure_size(1200, 780);
  plt::xlim(-2, 100);
  plt::ylim(-2, 100);
  plt::plot({start.x}, {start.y}, "xr");
  plt::plot({goal.x}, {goal.y}, "xr");
  drawTree(pathFinder.getAllPoints());
  draw_path(*final_path);
  plt::tight_layout();
  plt::show();

  return 0;
}
