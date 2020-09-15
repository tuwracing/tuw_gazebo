#include <tuw_gazebo_plugins/cone_detection_sim.h>
#include <cstdlib>
#include <ctime>
#include <random>

namespace gazebo
{
  namespace math = ignition::math;

ConeDetectionSim::ConeDetectionSim() : detection_prob_distribution(0.0, 1.0) {}

ConeDetectionSim::~ConeDetectionSim()
{
  pub_queue_.reset();
  pub_.shutdown();
}

void ConeDetectionSim::ParseOpt()
{
  gazebo_ros_->getParameter<std::string>(options_.nodeHandleName,
                                         "nodeHandleName", "");

  gazebo_ros_->getParameter<std::string>(options_.topicName, "topicName",
                                         "cones_gt");

  gazebo_ros_->getParameter<std::string>(options_.frameId, "frameId",
                                         "base_link");

  gazebo_ros_->getParameter<double>(options_.detectionConfig.distanceMin,
                                    "distanceMin", 0.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.distanceMax,
                                    "distanceMax", 0.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.fovHorizontalDeg,
                                    "fovHorizontalDeg", 0.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.fovVerticalDeg,
                                    "fovVerticalDeg", 0.0);

  if (gazebo_ros_->Sdf()->HasElement("detection_noise"))
  {
    auto noise = gazebo_ros_->Sdf()->GetElement("detection_noise");
    noiseX_.loadParam(noise);
    noiseY_.loadParam(noise);
  }
}

void ConeDetectionSim::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  parent_ = parent;
  gazebo_ros_ = GazeboRosPtr(new GazeboRos(parent, sdf, "ConeDetectionSim"));
  gazebo_ros_->isInitialized();
  ParseOpt();

  rosNode_.reset(new ros::NodeHandle(options_.nodeHandleName));
  pub_ = rosNode_->advertise<tuw_object_msgs::ObjectDetection>(
      options_.topicName, 1);
  ROS_INFO("%s: Advertising on %s", gazebo_ros_->info(), options_.topicName.c_str());

  pub_multi_queue_.startServiceThread();
  pub_queue_ = pub_multi_queue_.addPub<tuw_object_msgs::ObjectDetection>();

  update_connection_ = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&ConeDetectionSim::Update, this));

  reconfigureServer_ = std::make_shared<
      dynamic_reconfigure::Server<tuw_gazebo_plugins::ConeDetectionSimConfig>>(
      ros::NodeHandle("~/" + gazebo_ros_->getPluginName()));
  reconfigureFnc_ =
      boost::bind(&ConeDetectionSim::callbackConfig, this, _1, _2);
  reconfigureServer_->setCallback(reconfigureFnc_);
}

void ConeDetectionSim::callbackConfig(
    tuw_gazebo_plugins::ConeDetectionSimConfig &_config, uint32_t _level)
{
  config_ = _config;
  ROS_DEBUG("%s: callbackConfig!", gazebo_ros_->info());
}

void ConeDetectionSim::setVisualDetectionCovariance(
    tuw_object_msgs::ObjectWithCovariance &owc)
{
  double pxHeight = 360.0;
  double pxWidth = 1280.0;

  // measured average pixel error of visual cone detection
  double pxErr = 5.0;
  // TODO by configuration, maximum range of the camera
  double maxRange = 20;
  // horizontal fov of the camera

  math::Angle fov;
  fov.Degree(90.);
  math::Angle bearingErrPerPxAtMaxRange;
  bearingErrPerPxAtMaxRange.Degree(10.);

  // + 1 due to rounding to nearest pixel
  double rangeNoise = maxRange / pxHeight * (pxErr + 1);
  double bearingNoise = fov.Radian() / pxWidth * (pxErr + 1);

  double fps = 30;
  double dt = 1 / fps;
  double frameGrabDelayRangeNoise = dt * 5; // TODO use speed measurement

  double corr = bearingErrPerPxAtMaxRange.Radian() / maxRange;

  owc.covariance_pose.resize(9);
  std::fill(owc.covariance_pose.begin(), owc.covariance_pose.end(), 0);

  double rang = config_.sig_range * (rangeNoise + frameGrabDelayRangeNoise);
  double brng = config_.sig_bearing * bearingNoise;

  // (0) (1)  -
  // (3) (4)  -
  //  -   -  (8)
  owc.covariance_pose[0] = rang;
  owc.covariance_pose[1] = config_.sig_correlation * corr;
  owc.covariance_pose[3] = config_.sig_correlation * corr;
  owc.covariance_pose[4] = brng;
  owc.covariance_pose[8] = 1.0;
}

static bool isInRangeOf(
  const math::Vector3d &conePos,
  const math::Pose3d &sensorPose,
  const DetectionConfig &cdConfig) {
    
  math::Vector3d relative = conePos - sensorPose.Pos();
  double len = relative.Length();
  if (len > cdConfig.distanceMax) {
    return false;
  }
  if (len < cdConfig.distanceMin) {
    return false;
  }

  math::Quaternion<double> rot = sensorPose.Rot();
  double yaw = rot.Yaw();
  double thetaToCone = atan2(relative.Y(), relative.X());

  double maxDeg = cdConfig.fovHorizontalDeg / 2;
  double distRad = std::abs(thetaToCone - yaw);
  double distDeg = distRad * 180 / M_PI;

  if (distDeg > maxDeg) {
    return false;
  }

  return true;
}

void ConeDetectionSim::Update()
{
  common::Time current_time = parent_->GetWorld()->SimTime();
  double dt = (current_time - last_update_time_).Double();

  update_period_ = 1.0 / config_.fps;
  if (dt < update_period_)
  {
    return;
  }
  last_update_time_ = current_time;

  cones_ = getConesInWorld(parent_->GetWorld());
  tuw_object_msgs::ObjectDetection od;

  math::Pose3d robotPose = parent_->WorldPose();

  for (physics::ModelPtr cone : cones_)
  {
    auto pos = robotPose.Pos();
    auto rot = robotPose.Rot();
    if (!isInRangeOf(cone->WorldPose().Pos(), robotPose, options_.detectionConfig)) {
      continue;
    }

    if (detection_prob_distribution(generator) <= config_.p_detection)
    {
      math::Vector3d relConePos = cone->WorldPose().Pos() - robotPose.Pos();
      relConePos = robotPose.Rot().Inverse().RotateVector(relConePos);

      double x = noiseX_.sim(relConePos.X(), dt);
      double y = noiseY_.sim(relConePos.Y(), dt);

      tuw_object_msgs::ObjectWithCovariance owc;
      tuw_object_msgs::Object object;

      object.ids.clear();
      object.ids_confidence.clear();

      object.ids.push_back(1);
      object.ids_confidence.push_back(1);

      object.pose.position.x = x;
      object.pose.position.y = y;
      object.pose.position.z = 0;
      object.pose.orientation.x = 0;
      object.pose.orientation.y = 0;
      object.pose.orientation.z = 0;
      object.pose.orientation.w = 1;

      object.shape = tuw_object_msgs::Object::SHAPE_TRAFFIC_CONE;
      object.shape_variables.resize(2);
      object.shape_variables[0] = 0.2;
      object.shape_variables[1] = getConeColorShapeVariable(cone);

      setVisualDetectionCovariance(owc);
      owc.object = object;

      od.objects.push_back(owc);
    }
  }

  od.header.stamp = ros::Time(current_time.sec, current_time.nsec);
  od.header.frame_id = options_.frameId;

  od.distance_min = options_.detectionConfig.distanceMin;
  od.distance_max = options_.detectionConfig.distanceMax;
  od.fov_horizontal = options_.detectionConfig.fovHorizontalDeg;
  od.fov_vertical = options_.detectionConfig.fovVerticalDeg;

  od.type = tuw_object_msgs::ObjectDetection::OBJECT_TYPE_TRAFFIC_CONE;
  od.sensor_type =
      tuw_object_msgs::ObjectDetection::SENSOR_TYPE_GENERIC_MONOCULAR_VISION;

  pub_.publish(od);

  ROS_DEBUG("%i cones detected and published", (int)od.objects.size());
}

GZ_REGISTER_MODEL_PLUGIN(ConeDetectionSim)
} // namespace gazebo
