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

#include <frl_vehicle_msgs/UwGliderCommand.h>
#include <frl_vehicle_msgs/UwGliderStatus.h>

#include <gazebo_msgs/ModelState.h>

#include <direct_kinematics_ros_plugins/DirectKinematicsROSPlugin.hh>

#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>

namespace direct_kinematics_ros
{
/////////////////////////////////////////////////
DirectKinematicsROSPlugin::DirectKinematicsROSPlugin()
{
}

/////////////////////////////////////////////////
DirectKinematicsROSPlugin::~DirectKinematicsROSPlugin()
{
  this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::Load(gazebo::physics::ModelPtr _model,
                             sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  GZ_ASSERT(_model != NULL, "Invalid model pointer");
  GZ_ASSERT(_sdf != NULL, "Invalid SDF element pointer");
  
  gzmsg << "Initializing ROS node for direct kinematics..." << std::endl;
  
  this->model = _model;
  this->rosNode.reset(new ros::NodeHandle(""));

  // -- Convey command from outside to gazebo model -- //
  // Advertise for command publisher
  this->commandPublisher =
    this->rosNode->advertise<gazebo_msgs::ModelState>
    ("/gazebo/set_model_state", 10);

  // Subscribe command from outside
  this->commandSubscriber =
    this->rosNode->subscribe<frl_vehicle_msgs::UwGliderCommand>
    (this->model->GetName() + "/direct_kinematics/UwGliderCommand", 10,
    boost::bind(&DirectKinematicsROSPlugin::ConveyCommand,
    this, _1));

  gzmsg << "Vehicle Command Topic name : " << this->model->GetName() 
    + "/direct_kinematics/UwGliderCommand" << std::endl;





}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::Init()
{
    // Nothing here
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::Reset()
{ }

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  // Update time
  this->time = this->model->GetWorld()->SimTime();
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::ConveyCommand(
  const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
{
  // Calculate command
  gazebo_msgs::ModelState command_state = this->calculateCommand(_msg);
  // publish command model state to gazebo/set_model_state topic
  this->commandPublisher.publish(command_state);
}


/////////////////////////////////////////////////
gazebo_msgs::ModelState DirectKinematicsROSPlugin::calculateCommand(
                const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
{
  gzmsg << _msg->target_pitch_value << std::endl;
    
  // Calculate command
  gazebo_msgs::ModelState command_msg;
  command_msg.model_name = this->model->GetName();
  command_msg.pose.position.x = 0.0;
  command_msg.pose.position.y = 0.0;
  command_msg.pose.position.z = 0.0;
  command_msg.pose.orientation.x = 0.0;
  command_msg.pose.orientation.y = 0.0;
  command_msg.pose.orientation.z = 0.0;
  command_msg.twist.linear.x = 0.0;
  command_msg.twist.linear.y = 0.0;
  command_msg.twist.linear.z = 0.0;
  command_msg.twist.angular.x = 0.0;
  command_msg.twist.angular.y = 0.0;
  command_msg.twist.angular.z = 0.0;
  command_msg.reference_frame = "world";

  // publish command model state to gazebo/set_model_state topic
  this->commandPublisher.publish(command_msg);
}


GZ_REGISTER_MODEL_PLUGIN(DirectKinematicsROSPlugin)
}
