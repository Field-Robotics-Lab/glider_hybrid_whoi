// Copyright (c) 2020 The Dave Simulator Authors.
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

/// \file Direct kinematics ROS plugin for a ROS node

#ifndef __DIRECT_KINEMATICS_ROS_PLUGIN_HH__
#define __DIRECT_KINEMATICS_ROS_PLUGIN_HH__

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>

#include <map>
#include <string>

namespace direct_kinematics_ros
{

  class DirectKinematicsROSPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: DirectKinematicsROSPlugin();

    /// \brief Destructor
    public: virtual ~DirectKinematicsROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::ModelPtr _model,
                      sdf::ElementPtr _sdf);

    /// \brief Initialize Module.
    public: virtual void Init();

    /// \brief Reset Module.
    public: virtual void Reset();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: virtual void Update(const gazebo::common::UpdateInfo &_info);

    /// \brief Convey model state from gazebo topic to outside
    // protected: virtual void ConveyModelState();

    /// \brief Convey model state from gazebo topic to outside
    protected: virtual void ConveyCommand(const frl_vehicle_msgs::
                                        UwGliderCommand::ConstPtr &_msg);

    /// \brief Calculate the model state from received command
    protected: virtual gazebo_msgs::ModelState calculateCommand
                (const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg);

    /// \brief Pointer to the model structure
    protected: gazebo::physics::ModelPtr model;

    /// \brief Pointer to this ROS node's handle.
    private: boost::scoped_ptr<ros::NodeHandle> rosNode;

    /// \brief ROS Subscribers from outside
    private: ros::Subscriber commandSubscriber;

    /// \brief ROS Publishers to gazebo_msgs topic
    private: ros::Publisher commandPublisher;

    /// \brief ROS Subscriber from gazebo topic
    private: std::map<std::string, ros::Subscriber> gazeboSubscriber;

    /// \brief ROS Publishers to outside
    private: std::map<std::string, ros::Publisher> statusPublishers;

    /// \brief Time at gazebo simulation
    protected: gazebo::common::Time time;

    /// \brief Command starting time recorder
    protected: gazebo::common::Time cmd_start_time;
  };
}

#endif  // __DIRECT_KINEMATICS_ROS_PLUGIN_HH__
