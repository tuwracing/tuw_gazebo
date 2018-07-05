#include <cstdlib>
#include <ctime>
#include <tuw_gazebo_plugins/gazebo_ros_map.h>

namespace gazebo
{

namespace math = ignition::math;

GazeboRosMap::GazeboRosMap() {}

GazeboRosMap::~GazeboRosMap()
{
  pubQueue_map_.reset();
  pubQueue_robot_.reset();
  pub_.shutdown();
}

void GazeboRosMap::ParseOpt()
{
  gazebo_ros_->getParameter<std::string>(options_.nodeHandleName,
                                         "nodeHandleName", "");

  gazebo_ros_->getParameter<std::string>(options_.topicName, "topicName",
                                         "map_gt");

  gazebo_ros_->getParameter<std::string>(options_.frameId, "frameId", "base_link");

  gazebo_ros_->getParameter<double>(options_.updateRate, "updateRate", 10.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.distanceMin,
                                    "distanceMin", 0.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.distanceMax,
                                    "distanceMax", 0.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.fovHorizontalDeg,
                                    "fovHorizontalDeg", 0.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.fovVerticalDeg,
                                    "fovVerticalDeg", 0.0);

  gazebo_ros_->getParameter<double>(options_.objectCovariancePos,
                                    "objectCovariancePos", 0.1);

  options_.referenceFrame = ReferenceFrame::ROBOT;
  if (gazebo_ros_->hasParameter("referenceFrame"))
  {
    std::string referenceFrame;
    gazebo_ros_->getParameter<std::string>(referenceFrame,
                                           "referenceFrame", "robot");
    if (boost::iequals(referenceFrame, "map"))
    {
      options_.referenceFrame = ReferenceFrame::MAP;
    }
    else if (!boost::iequals(referenceFrame, "robot"))
    {
      ROS_WARN("%s: referenceFrame must be either 'robot' or 'map'", gazebo_ros_->info());
    }
  }

  update_period_ = 1.0 / options_.updateRate;
}

void GazeboRosMap::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  parent_ = parent;
  gazebo_ros_ = GazeboRosPtr(new GazeboRos(parent, sdf, "Map"));
  gazebo_ros_->isInitialized();
  ParseOpt();

  rosNode_.reset(new ros::NodeHandle(options_.nodeHandleName));
  pub_ = rosNode_->advertise<tuw_object_msgs::ObjectDetection>(options_.topicName, 1);
  ROS_INFO("%s: Advertising on %s", gazebo_ros_->info(), options_.topicName.c_str());

  pub_multi_queue_.startServiceThread();

  if (options_.referenceFrame == ReferenceFrame::ROBOT)
  {
    pubQueue_robot_ = pub_multi_queue_.addPub<tuw_object_msgs::ObjectDetection>();
  }
  else
  {
    pubQueue_map_ = pub_multi_queue_.addPub<tuw_object_msgs::ObjectWithCovarianceArray>();
  }

  update_connection_ = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&GazeboRosMap::Update, this));
}

void GazeboRosMap::toObjectWithCovariance(
    physics::ModelPtr &cone,
    tuw_object_msgs::ObjectWithCovariance &owc)
{
  tuw_object_msgs::Object object;

  object.ids.clear();
  object.ids_confidence.clear();
  object.ids.push_back(1);
  object.ids_confidence.push_back(1);

  object.pose.position.x = cone->WorldPose().Pos().X();
  object.pose.position.y = cone->WorldPose().Pos().Y();
  object.pose.position.z = 0;
  object.pose.orientation.x = 0;
  object.pose.orientation.y = 0;
  object.pose.orientation.z = 0;
  object.pose.orientation.w = 1;

  object.shape = tuw_object_msgs::Object::SHAPE_TRAFFIC_CONE;
  object.shape_variables.resize(2);
  object.shape_variables[0] = gazebo::TrafficCone::coneRadius;
  object.shape_variables[1] = getConeColorShapeVariable(cone);

  owc.covariance_pose.resize(9);
  for (size_t i = 0; i < 9; i++)
  {
    owc.covariance_pose[i] = 0.0;
  }
  owc.covariance_pose[0] = options_.objectCovariancePos;
  owc.covariance_pose[4] = options_.objectCovariancePos;
  owc.covariance_pose[8] = options_.objectCovariancePos;

  owc.object = object;
}

void GazeboRosMap::Update()
{
  common::Time current_time = parent_->GetWorld()->SimTime();
  double dt = (current_time - last_update_time_).Double();
  if (dt < update_period_)
  {
    return;
  }

  last_update_time_ = current_time;
  cones_ = getConesInWorld(parent_->GetWorld());
  math::Pose3d robotPose = parent_->WorldPose();
  auto stamp = ros::Time(current_time.sec, current_time.nsec);

  if (options_.referenceFrame == ReferenceFrame::ROBOT)
  {

    tuw_object_msgs::ObjectDetection od;
    for (physics::ModelPtr &cone : cones_)
    {
      // filter any cone that is too far away from the robot
      math::Vector3d posDiff = cone->WorldPose().Pos() - robotPose.Pos();
      if (posDiff.Length() > options_.detectionConfig.distanceMax)
      {
        continue;
      }

      // add object to detection in robot space
      tuw_object_msgs::ObjectWithCovariance owc;
      toObjectWithCovariance(cone, owc);

      math::Vector3d posInRobotSpace = robotPose.Rot().RotateVectorReverse(posDiff);
      owc.object.pose.position.x = posInRobotSpace.X();
      owc.object.pose.position.y = posInRobotSpace.Y();

      od.objects.push_back(owc);
    }
    od.header.stamp = stamp;
    od.header.frame_id = options_.frameId;
    pub_.publish(od);
  }
  else
  {
    tuw_object_msgs::ObjectWithCovarianceArray owca;
    for (physics::ModelPtr &cone : cones_)
    {
      // add object to array in map space
      tuw_object_msgs::ObjectWithCovariance owc;
      toObjectWithCovariance(cone, owc);
      owca.objects.push_back(owc);
    }
    owca.header.stamp = stamp;
    owca.header.frame_id = options_.frameId;
    pub_.publish(owca);
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMap)
} // namespace gazebo
