// Copyright (c) 2016 The hybrid_glider_ Simulator Authors.
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

#include <hybrid_glider_gazebo_plugins/BuoyancyPumpPlugin.hh>
#include <hybrid_glider_gazebo_plugins/Def.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

GZ_REGISTER_MODEL_PLUGIN(hybridglider::BuoyancyPumpPlugin)

using namespace gazebo;

namespace hybridglider
{

/////////////////////////////////////////////////
BuoyancyPumpPlugin::BuoyancyPumpPlugin() : inputCommand(0), pumpPos(0), buoyancypumpID(-1)
{
}

/////////////////////////////////////////////////
BuoyancyPumpPlugin::~BuoyancyPumpPlugin()
{
  if (this->updateConnection)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->updateConnection.reset();
#else
    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
#endif
  }
}

/////////////////////////////////////////////////
void BuoyancyPumpPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initializing the transport node
  this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION >= 8
  this->node->Init(_model->GetWorld()->Name());
#else
  this->node->Init(_model->GetWorld()->GetName());
#endif

  // BuoyancyPump ID
  GZ_ASSERT(_sdf->HasElement("buoyancypumpID"), "Could not find buoyancypumpID parameter.");
  this->buoyancypumpID = _sdf->Get<int>("buoyancypumpID");
  GZ_ASSERT(this->buoyancypumpID >= 0, "BuoyancyPump ID must be greater or equal than zero");

  // Root string for topics
  std::stringstream strs;
  strs << "/" << _model->GetName() << "/buoyancypumps/" << this->buoyancypumpID << "/";
  this->topicPrefix = strs.str();

  // Subscribe to input signal topic
  this->commandSubscriber =
    this->node->Subscribe(this->topicPrefix + "input",
        &BuoyancyPumpPlugin::UpdateInput,
        this);

  // Advertise the output topic
  this->pumpPosPublisher =
      this->node->Advertise<
      hybrid_glider_gazebo_plugins_msgs::msgs::Double>(this->topicPrefix + "output");


  // Subscribe to the input signal topic
  this->commandSubscriber = this->node->Subscribe(this->topicPrefix + "input",
                                                &BuoyancyPumpPlugin::UpdateInput,
                                                this);

  // Connect the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&BuoyancyPumpPlugin::OnUpdate,
                    this, _1));
}

/////////////////////////////////////////////////
void BuoyancyPumpPlugin::Init()
{
}

/////////////////////////////////////////////////
void BuoyancyPumpPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  GZ_ASSERT(!std::isnan(this->inputCommand),
            "nan in this->inputCommand");

  // Update dynamics model:
  this->pumpPos = this->dynamics->update(this->inputCommand,
                                       _info.simTime.Double());

  // Publish position:
  msgs::Vector3d pumpPosMsg;
  msgs::Set(&pumpPosMsg, ignition::math::Vector3d(this->pumpPos, 0., 0.));
  this->pumpPosPublisher->Publish(pumpPosMsg);
}

/////////////////////////////////////////////////
void BuoyancyPumpPlugin::UpdateInput(ConstDoublePtr &_msg)
{
  this->inputCommand = _msg->value();
}
}