// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file KinematicsROSPlugin.hh
/// \brief Publishes the constant flow velocity in ROS messages and creates a
/// service to alter the flow model in runtime

#ifndef __KINEMATICS_ROS_PLUGIN_HH__
#define __KINEMATICS_ROS_PLUGIN_HH__

#include <map>
#include <string>

// Gazebo plugin
#include <kinematics_plugins/KinematicsPlugin.hh>

#include <boost/scoped_ptr.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <kinematics_ros_plugins_msgs/SetCurrentModel.h>
#include <kinematics_ros_plugins_msgs/GetCurrentModel.h>
#include <kinematics_ros_plugins_msgs/SetCurrentVelocity.h>
#include <kinematics_ros_plugins_msgs/SetCurrentDirection.h>
#include <kinematics_ros_plugins_msgs/SetOriginSphericalCoord.h>
#include <kinematics_ros_plugins_msgs/GetOriginSphericalCoord.h>

namespace kinematics_ros
{
  class KinematicsROSPlugin : public gazebo::KinematicsPlugin
  {
    /// \brief Class constructor
    public: KinematicsROSPlugin();

    /// \brief Class destructor
    public: virtual ~KinematicsROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::WorldPtr _world,
        sdf::ElementPtr _sdf);

    /// \brief Service call to update the parameters for the velocity
    /// Gauss-Markov process model
    public: bool UpdateCurrentVelocityModel(
        kinematics_ros_plugins_msgs::SetCurrentModel::Request& _req,
        kinematics_ros_plugins_msgs::SetCurrentModel::Response& _res);

    /// \brief Service call to update the parameters for the horizontal angle
    /// Gauss-Markov process model
    public: bool UpdateCurrentHorzAngleModel(
        kinematics_ros_plugins_msgs::SetCurrentModel::Request& _req,
        kinematics_ros_plugins_msgs::SetCurrentModel::Response& _res);

    /// \brief Service call to update the parameters for the vertical angle
    /// Gauss-Markov process model
    public: bool UpdateCurrentVertAngleModel(
        kinematics_ros_plugins_msgs::SetCurrentModel::Request& _req,
        kinematics_ros_plugins_msgs::SetCurrentModel::Response& _res);

    /// \brief Service call to read the parameters for the velocity
    /// Gauss-Markov process model
    public: bool GetCurrentVelocityModel(
        kinematics_ros_plugins_msgs::GetCurrentModel::Request& _req,
        kinematics_ros_plugins_msgs::GetCurrentModel::Response& _res);

    /// \brief Service call to read the parameters for the horizontal angle
    /// Gauss-Markov process model
    public: bool GetCurrentHorzAngleModel(
        kinematics_ros_plugins_msgs::GetCurrentModel::Request& _req,
        kinematics_ros_plugins_msgs::GetCurrentModel::Response& _res);

    /// \brief Service call to read the parameters for the vertical angle
    /// Gauss-Markov process model
    public: bool GetCurrentVertAngleModel(
        kinematics_ros_plugins_msgs::GetCurrentModel::Request& _req,
        kinematics_ros_plugins_msgs::GetCurrentModel::Response& _res);

    /// \brief Service call to update the mean value of the flow velocity
    public: bool UpdateCurrentVelocity(
        kinematics_ros_plugins_msgs::SetCurrentVelocity::Request& _req,
        kinematics_ros_plugins_msgs::SetCurrentVelocity::Response& _res);

    /// \brief Service call to update the mean value of the horizontal angle
    public: bool UpdateHorzAngle(
        kinematics_ros_plugins_msgs::SetCurrentDirection::Request& _req,
        kinematics_ros_plugins_msgs::SetCurrentDirection::Response& _res);

    /// \brief Service call to update the mean value of the vertical angle
    public: bool UpdateVertAngle(
        kinematics_ros_plugins_msgs::SetCurrentDirection::Request& _req,
        kinematics_ros_plugins_msgs::SetCurrentDirection::Response& _res);

    /// \brief Publishes ROS topics
    private: void OnUpdateCurrentVel();

    /// \brief All underwater world services
    private: std::map<std::string, ros::ServiceServer> worldServices;

    /// \brief Pointer to this ROS node's handle.
    private: boost::scoped_ptr<ros::NodeHandle> rosNode;

    /// \brief Connection for callbacks on update world.
    private: gazebo::event::ConnectionPtr rosPublishConnection;

    /// \brief Publisher for the flow velocity in the world frame
    private: ros::Publisher flowVelocityPub;

    /// \brief Period after which we should publish a message via ROS.
    private: gazebo::common::Time rosPublishPeriod;

    /// \brief Last time we published a message via ROS.
    private: gazebo::common::Time lastRosPublishTime;
  };
}

#endif  // __KINEMATICS_ROS_PLUGIN_HH__
