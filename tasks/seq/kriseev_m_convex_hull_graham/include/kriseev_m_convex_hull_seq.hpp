// Copyright 2024 Kriseev Mikhail
#pragma once

#include <vector>

#include "core/task/include/task.hpp"
namespace KriseevMTaskSeq {

typedef std::pair<double, double> Point;

class ConvexHullTask : public ppc::core::Task {
 public:
  // Task data layout:
  // inputs:
  //   [0]: double * pointsX
  //   [1]: double * pointsY
  //  outputs:
  //   [0]: double * pointsX
  //   [1]: double * pointsY

  explicit ConvexHullTask(std::shared_ptr<ppc::core::TaskData> taskData_) : Task(std::move(taskData_)) {}
  bool pre_processing() override;
  bool validation() override;
  bool run() override;
  bool post_processing() override;

 private:
  std::vector<Point> points;
  double angle(const Point& origin, const Point& point);
  bool checkOrientation(const Point& origin, const Point& a, const Point& b);
  std::vector<Point> finalHull;
};
}  // namespace KriseevMTaskSeq
