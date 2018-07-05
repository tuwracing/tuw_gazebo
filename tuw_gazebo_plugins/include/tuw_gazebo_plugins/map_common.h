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

bool isInRangeOf(const ignition::math::Vector3d &conePos,
                 physics::ModelPtr model, const DetectionConfig &cdConfig);

std::vector<physics::ModelPtr> getConesInWorld(const physics::WorldPtr &world);

std::vector<physics::ModelPtr>
getConesInWorldSeenBy(const physics::WorldPtr &world,
                      const physics::ModelPtr &robot,
                      const DetectionConfig &cdConfig);
}

