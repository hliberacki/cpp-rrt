#include "rrt.hpp"
#include <random>
#include <cmath>

namespace planning::pathFinding {

double pointsDistance(Point p1, Point p2) {
  // TODO: check if there is already a library for that (it must be!)
  double deltax = p2.x - p1.x;
  double deltay = p2.y - p1.y;
  double deltaz = p2.z - p1.z;
  // Simple calculation of distance between points
  return std::sqrt(std::pow(deltax, 2) + std::pow(deltay, 2) + std::pow(deltaz, 2));
}

Rrt::Rrt(Point startPos, Point goalPos, double maxVertex = 500,
      double maxExtendLength = 3.0,
      double mGoalSampleRate = 0.05)
: mPoints {std::make_shared<Node>(Node{startPos, nullptr})}
, mStart{startPos}
, mGoal{Node{goalPos ,nullptr}}
, mMaxVertexes {maxVertex}
, mMaxExtendLength {maxExtendLength}
, mGoalSampleRate {mGoalSampleRate}
{
  // TODO: make it better and configurable
  mBoundsX = {0, 100};
  mBoundsY = {0, 100};
  mBoundsZ = {0, 100};
}

std::shared_ptr<Node> Rrt::nearestNode(const Point newPoint)
{
  double distance = pointsDistance(mPoints[0]->mCords, newPoint);
  auto nearest = mPoints[0];

  for(auto node : mPoints) {
    double currentDistance = pointsDistance(node->mCords, newPoint);
    if (currentDistance < distance) {
      distance = currentDistance;
      nearest = node;
    }
  }
  return nearest;
}

Node Rrt::randomNode(/*Add bounds - for dynamic areas*/)
{
  std::random_device randDev;
  std::mt19937 generator(randDev());
  std::uniform_real_distribution<> Xdistribution(mBoundsX[0], mBoundsX[1]);
  std::uniform_real_distribution<> Ydistribution(mBoundsY[0], mBoundsY[1]);
  // TODO: enable 3d points
  Point p;
  p.x = Xdistribution(generator);
  p.y = Ydistribution(generator);
  p.z = 0;
  //displayPoint(p);
  return Node{p, nullptr};
}

std::shared_ptr<Node> Rrt::steer(std::shared_ptr<Node> fromNode, Node toNode)
{
  std::shared_ptr newNode = std::make_shared<Node>(toNode);

  auto distance = pointsDistance(fromNode->mCords, toNode.mCords);
  if (distance > mMaxExtendLength)
  {
    // In this situation new point is located too far from the nearest point in the tree
    // it shall not be discarded but rather moved closer into the tree - respecting
    // his "orientation".
    Point newPoint;
    newPoint.x = fromNode->mCords.x - toNode.mCords.x;
    newPoint.y = fromNode->mCords.y - toNode.mCords.y;
    newPoint.z = fromNode->mCords.z - toNode.mCords.z;

    // Rescale each dimension to fit mMaxExtendLength with correct point orientation
    newNode->mCords.x = fromNode->mCords.x - newPoint.x / distance * mMaxExtendLength;
    newNode->mCords.y = fromNode->mCords.y - newPoint.y / distance * mMaxExtendLength;
    newNode->mCords.z = fromNode->mCords.z - newPoint.z / distance * mMaxExtendLength;
  }

  newNode->parent = fromNode;
  return newNode;
}

PointsList Rrt::path()
{
  PointsList result;
  auto currentNode = mPoints.back();
  while (currentNode->parent != nullptr) {
    result.push_back(currentNode);
    currentNode = currentNode->parent;
  }
  result.push_back(mPoints[0]);

  return result;
}

std::optional<PointsList> Rrt::plan()
{
  for (auto vertexCount = 0; vertexCount < mMaxVertexes; ++vertexCount) {
    auto randNode = randomNode();
    std::shared_ptr<Node> nearest = nearestNode(randNode.mCords);
    std::shared_ptr<Node> newNode = steer(nearest, randNode);
    //if (!isInCollusion(nearest, newNode))
    // TODO add collision
    mPoints.push_back(newNode);

    if (pointsDistance(mPoints.back()->mCords, mGoal.mCords) <=mMaxExtendLength) {
      auto lastNode = steer(mPoints.back(), mGoal);
      //if (!isInCollusion(lastNode, mPoints.back()))
      // TODO add collision
      mPoints.push_back(lastNode);
      return path();
    }
  }

  return {};
}


}