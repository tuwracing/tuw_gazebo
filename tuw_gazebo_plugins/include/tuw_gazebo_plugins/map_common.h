#pragma once

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <string>
#include <vector>

namespace gazebo {

typedef struct {
  double distanceMin;

  double distanceMax;

  double fovHorizontalDeg;

  double fovVerticalDeg;

} DetectionConfig;

namespace TrafficCone {
  const double coneRadius = 0.2;

  const double coneColorUnknown = 0;
  const double coneColorBlue = 1;
  const double coneColorYellow = 2;
  const double coneColorRed = 3;
}

double getConeColorShapeVariable(physics::ModelPtr model);

std::vector<physics::ModelPtr> getConesInWorld(const physics::WorldPtr &world);
}

