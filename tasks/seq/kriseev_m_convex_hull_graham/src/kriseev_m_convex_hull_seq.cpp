// Copyright 2024 Kriseev Mikhail
#include "seq/kriseev_m_convex_hull_graham/include/kriseev_m_convex_hull_seq.hpp"

#include <algorithm>
#include <limits>

bool KriseevMTaskSeq::ConvexHullTask::pre_processing() {
  if (taskData->inputs_count.at(0) != taskData->inputs_count.at(1)) {
    return false;
  }
  auto *pointsX = reinterpret_cast<double *>(taskData->inputs.at(0));
  auto *pointsY = reinterpret_cast<double *>(taskData->inputs.at(1));
  points = std::vector<std::pair<double, double>>();
  points.reserve(taskData->inputs_count[0]);
  for (size_t i = 0; i < taskData->inputs_count[0]; ++i) {
    points.emplace_back(pointsX[i], pointsY[i]);
  }

  return true;
}

bool KriseevMTaskSeq::ConvexHullTask::validation() {
  if (points.size() < 3) {
    return false;
  }
  for (uint32_t i = 0; i < points.size(); ++i) {
    if (points[i].first == std::numeric_limits<double>::quiet_NaN() ||
        points[i].second == std::numeric_limits<double>::quiet_NaN() ||
        points[i].first == std::numeric_limits<double>::infinity() ||
        points[i].second == std::numeric_limits<double>::infinity()) {
      return false;
    }
  }
  return true;
}

bool KriseevMTaskSeq::ConvexHullTask::run() {
  Point origin =
      *std::min_element(points.begin(), points.end(), [this](auto &a, auto &b) -> bool { return a.second < b.second; });
  std::sort(points.begin(), points.end(), [this, origin](auto &a, auto &b) -> bool {
    double angleA = angle(origin, a);
    double angleB = angle(origin, b);
    if (angleA < angleB) {
      return true;
    } else if (angleA > angleB) {
      return false;
    } else {
      double dxA = a.first - origin.first;
      double dyA = a.second - origin.second;
      double dxB = b.first - origin.first;
      double dyB = b.second - origin.second;
      return dxA * dxA + dyA * dyA < dxB * dxB + dyB * dyB;
    }
  });
  std::vector<Point> hull;
  hull.reserve(points.size());
  hull.push_back(points[0]);
  hull.push_back(points[1]);
  for (uint32_t i = 2; i < points.size() - 1; ++i) {
    if (checkOrientation(points[i], hull.back(), points[i + 1])) {
      hull.push_back(points[i]);
    }
  }
  if (checkOrientation(points.back(), hull.back(), points[0])) {
    hull.push_back(points.back());
  }
  this->finalHull = hull;
  return true;
}

bool KriseevMTaskSeq::ConvexHullTask::post_processing() {
  double *resultsX = reinterpret_cast<double *>(taskData->outputs[0]);
  double *resultsY = reinterpret_cast<double *>(taskData->outputs[1]);
  for (size_t i = 0; i < finalHull.size(); ++i) {
    resultsX[i] = finalHull[i].first;
    resultsY[i] = finalHull[i].second;
  }
  taskData->outputs_count[0] = finalHull.size();
  taskData->outputs_count[1] = finalHull.size();
  return true;
}

double KriseevMTaskSeq::ConvexHullTask::angle(const Point &origin, const Point &point) {
  double dx = point.first - origin.first;
  double dy = point.second - origin.second;
  if (dx == 0.0 && dy == 0.0) {
    return 0.0;
  }
  // Borrowed code: https://stackoverflow.com/a/14675998/14116050
  if (dy >= 0)
    return (dx >= 0 ? dy / (dx + dy) : 1 - dx / (-dx + dy));
  else
    return (dx < 0 ? 2 - dy / (-dx - dy) : 3 + dx / (dx - dy));
}

bool KriseevMTaskSeq::ConvexHullTask::checkOrientation(const Point &origin, const Point &a, const Point &b) {
  double dxA = a.first - origin.first;
  double dyA = a.second - origin.second;
  double dxB = b.first - origin.first;
  double dyB = b.second - origin.second;
  // Using cross product to check orientation
  return dxA * dyB - dyA * dxB < 0;
}
