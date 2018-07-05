#include "tuw_gazebo_plugins/map_common.h"

namespace gazebo {

double getConeColorShapeVariable(physics::ModelPtr cone) {
  if (cone->GetScopedName(false).find("cone_blue") != std::string::npos ||
      cone->GetScopedName(false).find("cone_l") != std::string::npos) {
    return TrafficCone::coneColorBlue;
  } else if (cone->GetScopedName(false).find("cone_yellow") != std::string::npos ||
             cone->GetScopedName(false).find("cone_r") != std::string::npos) {
    return TrafficCone::coneColorYellow;
  } else if (cone->GetScopedName(false).find("cone") != std::string::npos) {
    return TrafficCone::coneColorRed;
  }
  return TrafficCone::coneColorUnknown;
}

bool isInRangeOf(const ignition::math::Vector3d &conePos,
                 physics::ModelPtr model, const DetectionConfig &cdConfig) {
  ignition::math::Pose3d pose = model->WorldPose();
  ignition::math::Vector3d pos = pose.Pos();

  // TODO sensor mounting position by configuration, not hard-coded
  double sensorX = 0.5;
  double sensorZ = 0.9;
  pos.X(pos.X() + sensorX);
  pos.Z(pos.Z() + sensorZ);

  ignition::math::Quaternion<double> rot = pose.Rot();
  double yaw = rot.Yaw();

  ignition::math::Vector3d relative = conePos - pos;

  double len = relative.Length();
  if (len > cdConfig.distanceMax) {
    return false;
  }
  if (len < cdConfig.distanceMin) {
    return false;
  }

  double thetaToCone = atan2(relative.Y(), relative.X());

  double maxDeg = cdConfig.fovHorizontalDeg / 2;
  double distRad = std::abs(thetaToCone - yaw);
  double distDeg = distRad * 180 / M_PI;

  if (distDeg > maxDeg) {
    return false;
  }

  return true;
}

std::vector<physics::ModelPtr> getConesInWorld(const physics::WorldPtr &world) {
  std::vector<physics::ModelPtr> cones;

  physics::Model_V models = world->Models();
  int nbrModels = models.size();

  for (int i(0); i < nbrModels; i++) {
    auto model = models[i];
    if (model->GetName().substr(0, 4) == "cone") {
      cones.push_back(model);
    }
  }
  return cones;
}

std::vector<physics::ModelPtr> getConesInWorldSeenBy(
    const physics::WorldPtr &world, const physics::ModelPtr &robot,
    const DetectionConfig &cdConfig) {
  auto cones = getConesInWorld(world);
  std::vector<physics::ModelPtr> filtered;
  for (int i(0); i < cones.size(); i++) {
    if (isInRangeOf(cones[i]->WorldPose().Pos(), robot, cdConfig)) {
      filtered.push_back(cones[i]);
    }
  }
  return filtered;
}
}
