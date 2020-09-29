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

/// \file kinematics_plugins.hh
/// \brief Plugin that for the vehicle kinematics

#ifndef __KINEMATICS_PLUGIN_HH__
#define __KINEMATICS_PLUGIN_HH__

#include <map>
#include <cmath>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <kinematics_plugins/GaussMarkovProcess.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief Class for the vehicle kinematics
  class KinematicsPlugin : public ModelPlugin
  {
    /// \brief Class constructor
    public: KinematicsPlugin();

    /// \brief Class destructor
    public: virtual ~KinematicsPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
        sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: void Update(const common::UpdateInfo &_info);

    /// \brief Publish current velocity and the pose of its frame
    protected: void PublishCurrentVelocity();

    /// \brief Update event
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the model
    protected: physics::ModelPtr model;

    /// \brief Pointer to sdf
    protected: sdf::ElementPtr sdf;

    /// \brief True if the sea surface is present
    protected: bool hasSurface;

    /// \brief Pointer to a node for communication
    protected: transport::NodePtr node;

    /// \brief Map of publishers
    protected: std::map<std::string, transport::PublisherPtr>
      publishers;

    /// \brief Current velocity topic
    protected: std::string currentVelocityTopic;

    /// \brief Namespace for topics and services
    protected: std::string ns;

    /// \brief Gauss-Markov process instance for the current velocity
    protected: GaussMarkovProcess currentVelModel;

    /// \brief Gauss-Markov process instance for horizontal angle model
    protected: GaussMarkovProcess currentHorzAngleModel;

    /// \brief Gauss-Markov process instance for vertical angle model
    protected: GaussMarkovProcess currentVertAngleModel;

    /// \brief Last update time stamp
    protected: common::Time lastUpdate;

    /// \brief Current linear velocity vector
    protected: ignition::math::Vector3d currentVelocity;
  };
}

#endif  // __KINEMATICS_PLUGIN_HH__
