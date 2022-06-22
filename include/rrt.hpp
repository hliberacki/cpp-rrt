#pragma once

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <optional>
#include <vector>
#include <memory>
#include <array>

namespace planning::pathFinding {

using Point = geometry_msgs::msg::Point;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;


/**
 * @brief Node represents each element of RRT tree
 */
struct Node {
  Point mCords;
  std::shared_ptr<Node> parent;
};

using PointsList = std::vector<std::shared_ptr<Node>>;

double pointsDistance(Point p1, Point p2);

class Rrt {
public:

  Rrt(Point startPos, Point GoalPos, double maxIter, double maxExtendLength,
      double mGoalSampleRate);

  std::optional<PointsList> plan();
  std::shared_ptr<Node> steer(std::shared_ptr<Node> fromNode, Node toNode);
  Node randomNode();
  // bool isInCollision(Point from, Point to, OccupancyGrid map);
  std::shared_ptr<Node> nearestNode(const Point newPoint);
  PointsList path();
  PointsList getAllPoints() {return mPoints;}

private:
  PointsList mPoints;
  Point mStart;
  Node mGoal;

  std::array<double, 2> mBoundsX;
  std::array<double, 2> mBoundsY;
  std::array<double, 2> mBoundsZ;

  // Maximum ammount of Nodes in Tree - Vertexes
  double mMaxVertexes;
  /**
   * Longest distance between nodes. In literature it's usally described
   * by ETA or Detla. The smaller the distance is the more nodes and curves
   * we will have in a final path. With small extend length, there shall be more
   * max vertexes allowed.
   */
  double mMaxExtendLength;
  double mGoalSampleRate;

};

}