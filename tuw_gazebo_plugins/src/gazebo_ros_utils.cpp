/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its 
 *     contributors may be used to endorse or promote products derived from 
 *     this software without specific prior written permission.
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <tuw_gazebo_plugins/gazebo_ros_utils.h>

#include <sdf/sdf.hh>
#include <tf/transform_listener.h>

#include <ros/package.h>

#include <ros/console.h>

using namespace gazebo;

const char *GazeboRos::info() const
{
    return info_text.c_str();
}
boost::shared_ptr<ros::NodeHandle> &GazeboRos ::node()
{
    return rosnode_;
}
const boost::shared_ptr<ros::NodeHandle> &GazeboRos ::node() const
{
    return rosnode_;
}

std::string GazeboRos ::resolveTF(const std::string &name)
{
    return tf::resolve(tf_prefix_, name);
}
const std::string GazeboRos ::getPluginName() const
{
    return plugin_;
}
const std::string GazeboRos ::getNamespace() const
{
    return namespace_;
}
void GazeboRos::readCommonParameter()
{
    ROS_DEBUG_NAMED(plugin_, "-------Starting plugin %s!-------", info());

    std::string logger_name = std::string(ROSCONSOLE_NAME_PREFIX) + "." + plugin_;
    if (sdf_->HasElement("rosDebugLevel"))
    {
        std::string debugLevel;
        getParameter<std::string>(debugLevel, "rosDebugLevel", "na", false);
        ROS_DEBUG_NAMED(plugin_, "%s: <rosDebugLevel> = %s", info(), debugLevel.c_str());

        auto logLevel = ros::console::levels::Debug;
        if (boost::iequals(debugLevel, std::string("Info")))
        {
            logLevel = ros::console::levels::Info;
        }
        else if (boost::iequals(debugLevel, std::string("Warn")))
        {
            logLevel = ros::console::levels::Warn;
        }
        else if (boost::iequals(debugLevel, std::string("Error")))
        {
            logLevel = ros::console::levels::Error;
        }
        else if (boost::iequals(debugLevel, std::string("Fatal")))
        {
            logLevel = ros::console::levels::Fatal;
        }
        if (ros::console::set_logger_level(logger_name, logLevel))
        {
            ros::console::notifyLoggerLevelsChanged();
        }
    }
    else
    {
        if (ros::console::set_logger_level(logger_name, ros::console::levels::Info))
        {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    if (tf_prefix_.empty())
    {
        tf_prefix_ = namespace_;
        boost::trim_right_if(tf_prefix_, boost::is_any_of("/"));
    }
    ROS_INFO_NAMED(plugin_, "%s: <tf_prefix> = %s", info(), tf_prefix_.c_str());
}

bool GazeboRos::hasParameter(const char *_tag_name) {
    return sdf_->HasElement(_tag_name);
}

void GazeboRos ::getParameterBoolean(bool &_value, const char *_tag_name, const bool &_default)
{
    _value = _default;
    if (!sdf_->HasElement(_tag_name))
    {
        ROS_DEBUG_NAMED(plugin_, "%s: missing <%s> default is %s", info(), _tag_name, (_default ? "true" : "false"));
    }
    else
    {
        getParameterBoolean(_value, _tag_name);
    }
}
void GazeboRos ::getParameterBoolean(bool &_value, const char *_tag_name)
{
    if (sdf_->HasElement(_tag_name))
    {
        std::string value = sdf_->GetElement(_tag_name)->Get<std::string>();
        if (boost::iequals(value, std::string("true")) || boost::iequals(value, std::string("1")))
        {
            _value = true;
        }
        else if (boost::iequals(value, std::string("false")) || boost::iequals(value, std::string("0")))
        {
            _value = false;
        }
        else
        {
            ROS_WARN_NAMED(plugin_, "%s: <%s> must be either true or false. Was %s", info(), _tag_name, value.c_str());
        }
    }
    ROS_DEBUG_NAMED(plugin_, "%s: <%s> = %s", info(), _tag_name, (_value ? "true" : "false"));
}

physics::JointPtr GazeboRos::getJoint(physics::ModelPtr &_parent, const char *_tag_name, const std::string &_joint_default_name)
{
    std::string joint_name;
    getParameter<std::string>(joint_name, _tag_name, _joint_default_name);
    physics::JointPtr joint = _parent->GetJoint(joint_name);
    if (!joint)
    {
        char error[200];
        snprintf(error, 200, "%s: couldn't get wheel hinge joint named %s", info(), joint_name.c_str());
        gzthrow(error);
    }
    return joint;
}

void GazeboRos::isInitialized()
{
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED(plugin_, info() << "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                               << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
}
