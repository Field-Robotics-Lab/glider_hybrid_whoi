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
  // CSV log write stream close
  writeLog.close();
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

  // Read link names from SDF configuration
  if (_sdf->HasElement("base_link_name"))
  {
    this->base_link_name = _sdf->Get<std::string>("base_link_name");
  }
  else
  {
    this->base_link_name = "base_link";
  }
  gzmsg << "DirectKinematicsPlugin: Base_link name : "
        << this->base_link_name << std::endl;

  if (_sdf->HasElement("rudder_link_name"))
  {
    this->rudder_link_name = _sdf->Get<std::string>("rudder_link_name");
    this->rudderExist = true;
    gzmsg << "DirectKinematicsPlugin: rudder_link name : "
        << this->rudder_link_name << std::endl;
  }
  else
  {
    this->rudderExist = false;
  }
  // get writeLog Flag
  if (_sdf->HasElement("writeLog"))
  {
    this->writeLogFlag = _sdf->Get<bool>("writeLog");
    gzmsg << "World position of the base_link is saved at"
        << "/tmp/HybridGliderLog.csv" << std::endl;
  }

  // -- Convey command from outside to gazebo model -- //
  // Advertise for command publisher
  bool service_ready = false;
  service_ready = ros::service::exists("/gazebo/set_model_state", true);
  if (!service_ready)
  {
    gzmsg << "set_model_state service does not exists" << std::endl;
  }
  this->commandPublisher["Model"] = this->rosNode->serviceClient    
                <gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  if (this->rudderExist)
  {
    this->commandPublisher["Rudder"] = this->rosNode->serviceClient      
                <gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
  }

  // Subscribe command from outside
  this->commandSubscriber =
    this->rosNode->subscribe<frl_vehicle_msgs::UwGliderCommand>
    (this->model->GetName() + "/direct_kinematics/UwGliderCommand", 10,
    boost::bind(&DirectKinematicsROSPlugin::ConveyCommands,
    this, _1));

  gzmsg << "Vehicle Command Topic name : " << this->model->GetName() 
    + "/direct_kinematics/UwGliderCommand" << std::endl;

  this->nedTransform.header.frame_id = this->model->GetName() + "/base_link";
  this->nedTransform.child_frame_id = this->model->GetName() + "/base_link_ned";
  this->nedTransform.transform.translation.x = 0;
  this->nedTransform.transform.translation.y = 0;
  this->nedTransform.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(M_PI, 0, 0);
  this->nedTransform.transform.rotation.x = quat.x();
  this->nedTransform.transform.rotation.y = quat.y();
  this->nedTransform.transform.rotation.z = quat.z();
  this->nedTransform.transform.rotation.w = quat.w();
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

  // CSV log write stream
  this->writeCSVLog();

  this->nedTransform.header.stamp = ros::Time::now();
  this->tfBroadcaster.sendTransform(this->nedTransform);
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::ConveyCommands(
  const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
{
  this->ConveyModelCommand(_msg);

  if (this->rudderExist)
  {
    this->ConveyRudderVisualCommand(_msg);
  }
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::ConveyModelCommand(
             const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
{
  // --------------------------------------- //
  // ------------ Pitch command ------------ //
  // --------------------------------------- //
  gazebo_msgs::SetModelState pitch_cmd_msg;
  int _pitch_cmd_type = _msg->pitch_cmd_type;
  pitch_cmd_msg.request.model_state.model_name = this->model->GetName();
  pitch_cmd_msg.request.model_state.reference_frame =
            this->model->GetName() + "/" + this->base_link_name;
  // 0 : battery position, 1: target once , 2: target servo
  if (_pitch_cmd_type == 0)  // battery position control
  {
    // nothing now
  }
  else if (_pitch_cmd_type == 1)  // target once
  {
    ignition::math::Quaterniond target_pitch;
    target_pitch.Euler(0.0, _msg->target_pitch_value, 0.0);
    pitch_cmd_msg.request.model_state.pose.orientation.x = target_pitch.X();
    pitch_cmd_msg.request.model_state.pose.orientation.y = target_pitch.Y();
    pitch_cmd_msg.request.model_state.pose.orientation.z = target_pitch.Z();
    pitch_cmd_msg.request.model_state.pose.orientation.w = target_pitch.W();
    // A table lookup is used to compute the desired
    // battery position and no corrections are made.
  }
  else if (_pitch_cmd_type == 2)  // target servo
  {
    // nothing now
  }
  else
  {
    gzmsg << "WRONG PITCH COMMAND TYPE "
        << "(0 : battery position, 1: target once, 2: target servo)" 
        << std::endl;
  }
  // publish command model state to gazebo/set_model_state topic
  this->commandPublisher["Model"].call(pitch_cmd_msg);

        bool result = pitch_cmd_msg.response.success;
        if (!result)
        {
            gzmsg << "failed " << std::endl;
            gzmsg << "failed " << std::endl;
            gzmsg << "failed " << std::endl;
            gzmsg << "failed " << std::endl;
            gzmsg << "failed " << std::endl;
            gzmsg << "failed " << std::endl;
            gzmsg << "failed " << std::endl;
            gzmsg << "failed " << std::endl;
            ROS_WARN("service call to set_model_state failed!");

        }
        else
        {
            gzmsg << "DONE " << std::endl;
            gzmsg << "DONE " << std::endl;
            gzmsg << "DONE " << std::endl;
            gzmsg << "DONE " << std::endl;
            gzmsg << "DONE " << std::endl;
            gzmsg << "DONE " << std::endl;
            gzmsg << "DONE " << std::endl;
            gzmsg << "DONE " << std::endl;
            gzmsg << "DONE " << std::endl;
            ROS_INFO("Done");

        }
//   // --------------------------------------- //
//   // ------------ Rudder control ----------- //
//   // --------------------------------------- //
//   gazebo_msgs::SetModelState rudder_cmd_msg;
//   int _rudder_control_mode = _msg->rudder_control_mode;
//   rudder_cmd_msg.request.model_state.model_name = this->model->GetName();
//   ignition::math::Quaterniond _target_heading;
//   // 0 : control heading, 1: control angle
//   if (_rudder_control_mode == 0)  // control heading
//   {
//     _target_heading.Euler(0.0, 0.0, _msg->target_heading);
//     rudder_cmd_msg.request.model_state.reference_frame = "world";
//   }
//   else if (_rudder_control_mode == 1)  // control angle
//   {
//     int _rudder_angle = _msg->rudder_angle;
//     rudder_cmd_msg.request.model_state.reference_frame =
//             this->model->GetName() + "/" + this->base_link_name;
//     // Rudder angle control
//     // 0: center, 1: port, 2: staboard, 3: direct
//     double rudderAngleZero = 0.0;
//     double rudderAnglePort = M_PI/3.0;
//     double rudderAngleStbd = -M_PI/3.0;
//     double rudderAngleTarget = _msg->target_rudder_angle;
//     if (_rudder_angle == 0)  // center
//     {
//       _target_heading.Euler(0.0, 0.0, rudderAngleZero);
//     }
//     else if (_rudder_angle == 1)  // port
//     {
//       _target_heading.Euler(0.0, 0.0, rudderAnglePort);
//     }
//     else if (_rudder_angle == 2)  // staboard
//     {
//       _target_heading.Euler(0.0, 0.0, rudderAngleStbd);
//     }
//     else if (_rudder_angle == 3)  // direct
//     {
//       _target_heading.Euler(0.0, 0.0, rudderAngleTarget);
//     }
//     else
//     {
//       gzmsg << "WRONG RUDDER ANGLE COMMAND "
//         << "(0: center, 1: port, 2: staboard, 3: direct)" 
//         << std::endl;
//     }
//     rudder_cmd_msg.request.model_state.pose.orientation.x = _target_heading.X();
//     rudder_cmd_msg.request.model_state.pose.orientation.y = _target_heading.Y();
//     rudder_cmd_msg.request.model_state.pose.orientation.z = _target_heading.Z();
//     rudder_cmd_msg.request.model_state.pose.orientation.w = _target_heading.W();
//   }
//   else
//   {
//     gzmsg << "WRONG RUDDER COMMAND TYPE "
//         << "(0 : control heading, 1: control angle)" 
//         << std::endl;
//   }
//   // publish command model state to gazebo/set_model_state topic
//   this->commandPublisher["Model"].call(rudder_cmd_msg);

//   // --------------------------------------- //
//   // ------- Motor/thruster command -------- //
//   // --------------------------------------- //
//   gazebo_msgs::SetModelState motor_cmd_msg;
//   int _motor_cmd_type = _msg->motor_cmd_type;
//   motor_cmd_msg.request.model_state.model_name = this->model->GetName();
//   // 0 : voltage command, 1: power command
//   if (_motor_cmd_type == 0)  // voltage command
//   {
//     // nothing now
//   }
//   else if (_motor_cmd_type == 1)  // power command
//   {
//     motor_cmd_msg.request.model_state.twist.linear.x = _msg->target_motor_cmd;
//     motor_cmd_msg.request.model_state.reference_frame =
//         this->model->GetName() + "/" + this->base_link_name;
//   }
//   else
//   {
//     gzmsg << "WRONG MOTOR/THRUSTER COMMAND TYPE "
//         << "(0 : voltage command, 1: power command)"
//         << std::endl;
//   }
//   // publish command model state to gazebo/set_model_state topic
//   this->commandPublisher["Model"].call(motor_cmd_msg);


//   // -------------------------------------- //
//   // ------- Buoyancy pump command -------- //
//   // -------------------------------------- //
//   gazebo_msgs::SetModelState pump_cmd_msg;
//   pump_cmd_msg.request.model_state.model_name = this->model->GetName();
//   pump_cmd_msg.request.model_state.twist.linear.z =
//                                 _msg->target_pumped_volume*9.81;
//   pump_cmd_msg.request.model_state.reference_frame = "world";
//   // publish command model state to gazebo/set_model_state topic
//   this->commandPublisher["Model"].call(pump_cmd_msg);
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::ConveyRudderVisualCommand(
  const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
{
//   // --------------------------------------- //
//   // -------- Rudder visual control -------- //
//   // --------------------------------------- //
//   gazebo_msgs::LinkState rudder_cmd_msg;
//   rudder_cmd_msg.request.model_state.reference_frame =
//         this->model->GetName() + "/" + this->rudder_link_name;
//   int _rudder_control_mode = _msg->rudder_control_mode;
//   rudder_cmd_msg.request.model_state.link_name = 
//         this->model->GetLink(this->rudder_link_name)->GetName();
//   // 0 : control heading, 1: control angle
//   if (_rudder_control_mode == 0)  // control heading
//   {
//     // nothing here, controlled using at ModelCommand
//   }
//   else if (_rudder_control_mode == 1)  // control angle
//   {
//     // Rudder angle control
//     // 0: center, 1: port, 2: staboard, 3: direct
//     int _rudder_angle = _msg->rudder_angle;
//     if (_rudder_angle == 0)  // center
//     {
//       rudder_cmd_msg.request.model_state.pose.orientation.z = 0;
//     }
//     else if (_rudder_angle == 1)  // port
//     {
//       rudder_cmd_msg.request.model_state.pose.orientation.z = + M_PI/3.0;
//     }
//     else if (_rudder_angle == 2)  // staboard
//     {
//       rudder_cmd_msg.request.model_state.pose.orientation.z = - M_PI/3.0;
//     }
//     else if (_rudder_angle == 3)  // direct
//     {
//       rudder_cmd_msg.request.model_state.pose.orientation.z = _msg->target_rudder_angle;
//     }
//     else
//     {
//       gzmsg << "WRONG RUDDER ANGLE COMMAND "
//         << "(0: center, 1: port, 2: staboard, 3: direct)" 
//         << std::endl;
//     }
//   }
//   else
//   {
//     gzmsg << "WRONG RUDDER COMMAND TYPE "
//         << "(0 : control heading, 1: control angle)" 
//         << std::endl;
//   }
//   // publish command model state to gazebo/set_model_state topic
//   this->commandPublisher["Rudder"].publish(rudder_cmd_msg);
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::writeCSVLog()
{
// CSV log write stream
if (this->writeLogFlag)
{
  double time = this->time.Double();
  if (writeCounter == 0)
  {
    writeLog.open("/tmp/HybridGliderLog.csv");
    writeLog << "# Hybrid Glider Plugin Log\n";
    writeLog << "# t,x,y,z,p,q,r\n";
    writeLog.close();
    writeCounter = writeCounter + 1;
  }
  if (floor(time * 10) == time * 10)
  {
    ignition::math::Pose3d pose = 
            this->model->GetLink(this->base_link_name)->WorldPose();
    ignition::math::Vector3<double> vP = pose.Pos();
    ignition::math::Vector3<double> vR(0.0, 0.0, 0.0);
    ignition::math::Vector4<double> q(0.0, 0.0, 0.0, 0.0);
    q.X() = pose.Rot().X();
    q.Y() = pose.Rot().Y();
    q.Z() = pose.Rot().Z();
    q.W() = pose.Rot().W();
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.X() * q.Y() + q.Z() * q.W());
    double cosr_cosp = 1 - 2 * (q.Y() * q.Y() + q.Z() * q.Z());
    vR.X() = std::atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double sinp = 2 * (q.X() * q.Z() - q.W() * q.Y());
    if (std::abs(sinp) >= 1)
    vR.Y() = std::copysign(M_PI / 2, sinp);  // use 90 deg if out
    else
    vR.Y() = std::asin(sinp);
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.X() * q.W() + q.Y() * q.Z());
    double cosy_cosp = 1 - 2 * (q.Z() * q.Z() + q.W() * q.W());
    vR.Z() = std::atan2(siny_cosp, cosy_cosp);

    writeLog.open("/tmp/HybridGliderLog.csv", std::ios_base::app);
    writeLog << time << ","
            << vP.X() << "," << vP.Y() << "," << vP.Z() << ","
            << vR.X() << "," << vR.Y() << "," << vR.Z() << "\n";
    writeLog.close();
  }
}
}
GZ_REGISTER_MODEL_PLUGIN(DirectKinematicsROSPlugin)
}
