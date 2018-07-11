#include "tuw_gazebo_plugins/map_common.h"

namespace gazebo {

namespace math = ignition::math;

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
}
