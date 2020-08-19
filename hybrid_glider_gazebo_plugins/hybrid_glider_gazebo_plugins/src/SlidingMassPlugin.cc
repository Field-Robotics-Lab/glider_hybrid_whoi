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

#include <hybrid_glider_gazebo_plugins/SlidingMassPlugin.hh>
#include <hybrid_glider_gazebo_plugins/Def.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

GZ_REGISTER_MODEL_PLUGIN(hybridglider::SlidingMassPlugin)

using namespace gazebo;

namespace hybridglider
{

/////////////////////////////////////////////////
SlidingMassPlugin::SlidingMassPlugin() : inputCommand(0), massPos(0), slidingmassID(-1)
{
}

/////////////////////////////////////////////////
SlidingMassPlugin::~SlidingMassPlugin()
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
void SlidingMassPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initializing the transport node
  this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION >= 8
  this->node->Init(_model->GetWorld()->Name());
#else
  this->node->Init(_model->GetWorld()->GetName());
#endif

  // SlidingMass ID
  GZ_ASSERT(_sdf->HasElement("slidingmassID"), "Could not find slidingmassID parameter.");
  this->slidingmassID = _sdf->Get<int>("slidingmassID");
  GZ_ASSERT(this->slidingmassID >= 0, "SlidingMass ID must be greater or equal than zero");

  // Root string for topics
  std::stringstream strs;
  strs << "/" << _model->GetName() << "/slidingmasses/" << this->slidingmassID << "/";
  this->topicPrefix = strs.str();

  // Subscribe to input signal topic
  this->commandSubscriber =
    this->node->Subscribe(this->topicPrefix + "input",
        &SlidingMassPlugin::UpdateInput,
        this);

  // Advertise the output topic
  this->massPosPublisher =
      this->node->Advertise<
      hybrid_glider_gazebo_plugins_msgs::msgs::Double>(this->topicPrefix + "output");


  // Subscribe to the input signal topic
  this->commandSubscriber = this->node->Subscribe(this->topicPrefix + "input",
                                                &SlidingMassPlugin::UpdateInput,
                                                this);

  // Connect the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&SlidingMassPlugin::OnUpdate,
                    this, _1));
}

/////////////////////////////////////////////////
void SlidingMassPlugin::Init()
{
}

/////////////////////////////////////////////////
void SlidingMassPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  GZ_ASSERT(!std::isnan(this->inputCommand),
            "nan in this->inputCommand");

  // Update dynamics model:
  this->massPos = this->dynamics->update(this->inputCommand,
                                       _info.simTime.Double());

  // Publish position:
  msgs::Vector3d massPosMsg;
  msgs::Set(&massPosMsg, ignition::math::Vector3d(this->massPos, 0., 0.));
  this->massPosPublisher->Publish(massPosMsg);
}

/////////////////////////////////////////////////
void SlidingMassPlugin::UpdateInput(ConstDoublePtr &_msg)
{
  this->inputCommand = _msg->value();
}
}