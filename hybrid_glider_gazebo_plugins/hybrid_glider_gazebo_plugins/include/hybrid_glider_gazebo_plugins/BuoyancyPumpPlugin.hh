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

/// \file BuoyancyPumpPlugin.hh
/// \brief Model plugin for description of a glider's Buoyancy pump.

#ifndef __HYBRID_GLIDER_GAZEBO_PLUGINS_BUOYANCYPUMP_PLUGIN_HH__
#define __HYBRID_GLIDER_GAZEBO_PLUGINS_BUOYANCYPUMP_PLUGIN_HH__

#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>

#include <gazebo/msgs/msgs.hh>
#include <hybrid_glider_gazebo_plugins/Dynamics.hh>

#include "Double.pb.h"

using namespace gazebo;

namespace hybridglider
{

/// \brief Debuoyancypumpition of a pointer to the floating point message
typedef const boost::shared_ptr<const hybrid_glider_gazebo_plugins_msgs::msgs::Double>
ConstDoublePtr;

class BuoyancyPumpPlugin : public ModelPlugin
{
    /// \brief Constructor
    public: BuoyancyPumpPlugin();

    /// \brief Destructor
    public: virtual ~BuoyancyPumpPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Callback for the input topic subscriber
    protected: void UpdateInput(ConstDoublePtr &_msg);

    /// \brief BuoyancyPump dynamic model
    protected: std::shared_ptr<Dynamics> dynamics;

    /// \brief Update event
    protected: event::ConnectionPtr updateConnection;

    /// \brief Gazebo node
    protected: transport::NodePtr node;

    /// \brief Subscriber to the reference signal topic.
    protected: transport::SubscriberPtr commandSubscriber;

    /// \brief Publisher to the output buoyancy pump pos topic
    protected: transport::PublisherPtr pumpPosPublisher;

    /// \brief Latest input command.
    protected: double inputCommand;

    /// \brief BuoyancyPump ID
    protected: int buoyancypumpID;

    /// \brief Topic prefix
    protected: std::string topicPrefix;

    /// \brief Latest buoyancypump pump pos in [cm].
    protected: double pumpPos;

    /// \brief Time stamp of latest thrust force
    protected: common::Time pumpPosStamp;
};
}

#endif
