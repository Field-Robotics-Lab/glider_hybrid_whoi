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


#include <direct_kinematics_ros_plugins/DirectKinematicsROSPlugin.hh>

#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

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
  this->updateConnection.reset();
  
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
  
  this->model = _model;
  // this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->rosNode.reset(new ros::NodeHandle(""));

  // Read link names from SDF configuration
  if (_sdf->HasElement("base_link_name"))
  {
    this->base_link_name = _sdf->Get<std::string>("base_link_name");
  }
  else
  {
    this->base_link_name = "base_link";  // defult name
  }

  // if (_sdf->HasElement("rudder_link_name"))
  // {
  //   this->rudder_link_name = _sdf->Get<std::string>("rudder_link_name");
  //   this->rudderExist = true;
  //   gzmsg << "DirectKinematicsPlugin: rudder_link name : "
  //       << this->rudder_link_name << std::endl;
  // }
  // else
  // {
  //   this->rudderExist = false;
  // }

  // -- Read model state from gazebo model -- //
  // Advertise for command publisher
  bool service_ready = false;
  service_ready = ros::service::exists("/gazebo/get_model_state",true);
  if (!service_ready) 
  {
    ROS_INFO("get_model_state service does not exists");
  }
  this->stateSubscriber["Model"] =
    this->rosNode->serviceClient
    <gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  // -- Convey command from outside to gazebo model -- //
  // Advertise for command publisher
  this->commandPublisher["Model"] =
    this->rosNode->serviceClient
    <gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  // if (this->rudderExist)
  // {
  //   this->commandPublisher["Rudder"] =
  //     this->rosNode->serviceClient
  //     <gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
  // }

  // Subscribe command from outside
  ros::SubscribeOptions so =
  ros::SubscribeOptions::create<frl_vehicle_msgs::UwGliderCommand>(
      this->model->GetName() + "/direct_kinematics/UwGliderCommand",
      1,
      boost::bind(&DirectKinematicsROSPlugin::ConveyCommands, this, _1),
      ros::VoidPtr(), &this->commandSubQueue);
  this->commandSubscriber = this->rosNode->subscribe(so);
  // Spin up the queue helper thread.
  this->commandSubQueueThread = std::thread(std::bind(
      &DirectKinematicsROSPlugin::commandSubThread, this));

  // Sensor subscribings
  this->sensorSubscribers["DVLOnOff"]=
    this->rosNode->subscribe<std_msgs::Bool>(
    this->model->GetName() + "/dvl/state", 10,
    boost::bind(&DirectKinematicsROSPlugin::UpdateDVLSensorOnOff,
    this, _1));

  this->sensorSubscribers["DVL"]=
    this->rosNode->subscribe<uuv_sensor_ros_plugins_msgs::DVL>(
    this->model->GetName() + "/dvl", 10,
    boost::bind(&DirectKinematicsROSPlugin::UpdateDVLSensorData,
    this, _1));
    
  this->sensorSubscribers["GPSOnOff"]=
    this->rosNode->subscribe<std_msgs::Bool>(
    this->model->GetName() + "/dvl/state", 10,
    boost::bind(&DirectKinematicsROSPlugin::UpdateGPSSensorOnOff,
    this, _1));

  this->sensorSubscribers["GPS"]=
    this->rosNode->subscribe<sensor_msgs::NavSatFix>(
    this->model->GetName() + "/gps", 10,
    boost::bind(&DirectKinematicsROSPlugin::UpdateGPSSensorData,
    this, _1));
    
  // Publisher for glider status
  this->statePublisher =
      this->rosNode->advertise<frl_vehicle_msgs::UwGliderStatus>(
        this->model->GetName() + "/direct_kinematics/UwGliderStatus", 10);


  // THIS PART IS HARD CODED FOR NOW for link names, can't get link names
  // for (gazebo::physics::Link_V::const_iterator iter = this->model->GetLinks().begin();
  //      iter != this->model->GetLinks().end(); ++iter)
  // {
  //   gzmsg << (*iter)->GetName() << std::endl;
  // }
  // gzmsg << this->model->GetWorld()->ModelByName(this->model->GetName())->GetLink("right_propeller_link")->GetName() << std::endl;

  this->nedTransform["base_link"].header.frame_id = this->model->GetName() + "/base_link";
  this->nedTransform["base_link"].child_frame_id = this->model->GetName() + "/base_link_ned";
  this->nedTransform["base_link"].transform.translation.x = 0;
  this->nedTransform["base_link"].transform.translation.y = 0;
  this->nedTransform["base_link"].transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(M_PI, 0, 0);
  this->nedTransform["base_link"].transform.rotation.x = quat.x();
  this->nedTransform["base_link"].transform.rotation.y = quat.y();
  this->nedTransform["base_link"].transform.rotation.z = quat.z();
  this->nedTransform["base_link"].transform.rotation.w = quat.w();

  // Connect the update event callback
  this->Connect();

  // Initiated
  gzmsg << std::endl;
  gzmsg << "##################################################" << std::endl;
  gzmsg << "#######  DIRECT KINEMATICS CONTROL PLUGIN  #######" << std::endl;
  gzmsg << "##################################################" << std::endl;
  gzmsg << "Vehicle Model name      : " << this->model->GetName() << std::endl;
  gzmsg << "Vehicle Base_link name  : " << this->base_link_name << std::endl;
  gzmsg << "--------------------------------------------------" << std::endl;
  gzmsg << "Command Topic (frl_msg type)" << std::endl;
  gzmsg << ":\t" + this->model->GetName()
          + "/direct_kinematics/UwGliderCommand" << std::endl;
  gzmsg << "State Topic (frl_msg type)" << std::endl;
  gzmsg << ":\t" + this->model->GetName() 
          + "/direct_kinematics/UwGliderStatus" << std::endl;
  gzmsg << "##################################################" << std::endl;
  gzmsg << std::endl;

  // get writeLog Flag
  if (_sdf->HasElement("writeLog"))
  {
    this->writeLogFlag = _sdf->Get<bool>("writeLog");
    gzmsg << "World position of the base_link is saved at"
        << "/tmp/DirectKinematicsLog.csv" << std::endl;
    remove("/tmp/DirectKinematicsLog.csv");
  }
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::Connect()
{
  // Connect the update event
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DirectKinematicsROSPlugin::Update,
                  this, _1));
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::Init()
{
  // Nothing here
  this->writeCounter = 0;
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::Reset()
{ 
  // Nothing here 
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::Update(const gazebo::common::UpdateInfo &)
{
  // Update time
  this->time = this->model->GetWorld()->SimTime();

  // Update model state
  this->updateModelState();

  // Send status
  this->ConveyModelState();

  // CSV log write stream
  this->writeCSVLog();

  this->nedTransform["base_link"].header.stamp = ros::Time::now();
  this->tfBroadcaster["base_link"].sendTransform(this->nedTransform["base_link"]);
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::updateModelState()
{
  gazebo_msgs::GetModelState model_state_msg;
  model_state_msg.request.model_name = this->model->GetName();
  this->stateSubscriber["Model"].call(model_state_msg);
  this->modelState.pose = model_state_msg.response.pose;
  this->modelState.twist = model_state_msg.response.twist;


}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::commandSubThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->commandSubQueue.callAvailable(ros::WallDuration(timeout));
  }
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::ConveyCommands(
  const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
{
  this->ConveyModelCommand(_msg);

  // if (this->rudderExist)
  // {
  //   this->ConveyRudderVisualCommand(_msg);
  // }

  // Print gzmsg
  gzmsg << "[DIRECT KINEMATICS] Control msg detected! at " 
        << this->time << " s" << std::endl;
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::ConveyModelCommand(
             const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
{
  // --------------------------------------- //
  // ----------- PITCH COMMAND ------------- //
  // --------------------------------------- //
  gazebo_msgs::SetModelState pitch_cmd_msg;
  int _pitch_cmd_type = _msg->pitch_cmd_type;
  pitch_cmd_msg.request.model_state.model_name = this->model->GetName();
  pitch_cmd_msg.request.model_state.pose = this->modelState.pose;
  // 1 : battery position, 2: target once , 3: target servo
  if (_pitch_cmd_type == 1)  // battery position control
  {
    // nothing now
  }
  else if (_pitch_cmd_type == 2)  // target once
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
  else if (_pitch_cmd_type == 3)  // target servo
  {
    // nothing now
  }
  else if (_pitch_cmd_type == 0)  // nothing
  {
    // default
  }
  else
  {
    gzmsg << "WRONG PITCH COMMAND TYPE "
        << "(1 : battery position, 2: target once, 3: target servo)"
        << std::endl;
  }
  pitch_cmd_msg.request.model_state.twist = this->modelState.twist;
  // publish command model state to gazebo/set_model_state topic
  this->commandPublisher["Model"].call(pitch_cmd_msg);
  this->modelState = pitch_cmd_msg.request.model_state;

  // --------------------------------------- //
  // ----------- RUDDER CONTROL ------------ //
  // --------------------------------------- //
  gazebo_msgs::SetModelState rudder_cmd_msg;
  int _rudder_control_mode = _msg->rudder_control_mode;
  rudder_cmd_msg.request.model_state.model_name = this->model->GetName();

  // Pass original state
  rudder_cmd_msg.request.model_state.pose = this->modelState.pose;
  rudder_cmd_msg.request.model_state.twist = this->modelState.twist;

  // Convert original orientation state to ignition variable
  double xOrientation = this->modelState.pose.orientation.x;
  double yOrientation = this->modelState.pose.orientation.y;
  double zOrientation = this->modelState.pose.orientation.z;
  double wOrientation = this->modelState.pose.orientation.w;
  ignition::math::Quaterniond orientation;
  orientation.Set(xOrientation, yOrientation, zOrientation, wOrientation);
  ignition::math::Vector3d orientation_euler = orientation.Euler();

  // Define _target_heading accordingly
  ignition::math::Quaterniond _target_heading;
  // 1 : control heading, 2: control angle
  if (_rudder_control_mode == 1)  // control heading
  {
    // NOT IMPLEMENTED YET 
  }
  else if (_rudder_control_mode == 2)  // control angle
  {
    int _rudder_angle = _msg->rudder_angle;
    // Rudder angle control
    // 1: center, 2: port, 3: staboard, 4: direct
    double rudderAngleZero = 0.0;
    double rudderAnglePort = M_PI/3.0;
    double rudderAngleStbd = -M_PI/3.0;
    double rudderAngleTarget = _msg->target_rudder_angle;
    if (_rudder_angle == 1)  // center
    {
      _target_heading.Euler(orientation_euler.X(), 
                            orientation_euler.Y(), 
                            orientation_euler.Z()-M_PI + rudderAngleZero);
      this->rudderAngle = rudderAngleZero;
    }
    else if (_rudder_angle == 2)  // port
    {
      _target_heading.Euler(orientation_euler.X(), 
                            orientation_euler.Y(), 
                            orientation_euler.Z()-M_PI + rudderAnglePort);
      this->rudderAngle = rudderAnglePort;
    }
    else if (_rudder_angle == 3)  // staboard
    {
      _target_heading.Euler(orientation_euler.X(), 
                            orientation_euler.Y(), 
                            orientation_euler.Z()-M_PI + rudderAngleStbd);
      this->rudderAngle = rudderAngleStbd;
    }
    else if (_rudder_angle == 4)  // direct
    {
      _target_heading.Euler(orientation_euler.X(), 
                            orientation_euler.Y(), 
                            orientation_euler.Z()-M_PI + rudderAngleTarget);
      this->rudderAngle = rudderAngleTarget;
    }
    else if (_rudder_angle == 0)  // nothing
    {
      // default
    }
    else
    {
      gzmsg << "WRONG RUDDER ANGLE COMMAND "
        << "(1: center, 2: port, 3: staboard, 4: direct)" 
        << std::endl;
    }
    rudder_cmd_msg.request.model_state.pose.orientation.x = _target_heading.X();
    rudder_cmd_msg.request.model_state.pose.orientation.y = _target_heading.Y();
    rudder_cmd_msg.request.model_state.pose.orientation.z = _target_heading.Z();
    rudder_cmd_msg.request.model_state.pose.orientation.w = _target_heading.W();
  }
  else if (_rudder_control_mode == 0)  // nothing
  {
    // default
  }
  else 
  {
    gzmsg << "WRONG RUDDER COMMAND TYPE "
        << "(1 : control heading, 2: control angle)" 
        << std::endl;
  }
  // publish command model state to gazebo/set_model_state topic
  this->commandPublisher["Model"].call(rudder_cmd_msg);
  this->modelState = rudder_cmd_msg.request.model_state;

  // --------------------------------------- //
  // ------- MOTOR/THRUSTER COMMAND -------- //
  // -------- local frame control ---------- //
  // --------------------------------------- //
  // Velocity control (may be needed later considering drags)
  gazebo_msgs::SetModelState motor_cmd_msg;
  int _motor_cmd_type = _msg->motor_cmd_type;
  motor_cmd_msg.request.model_state.model_name = this->model->GetName();
  motor_cmd_msg.request.model_state.reference_frame =
        this->model->GetName() + "/" + this->base_link_name;
  // 1 : voltage command, 2: power command
  if (_motor_cmd_type == 1)  // voltage command
  {
    // nothing now
  }
  else if (_motor_cmd_type == 2)  // power command
  {
    motor_cmd_msg.request.model_state.twist.linear.x
          = _msg->target_motor_cmd;
    this->motorPower = _msg->target_motor_cmd;
  }
  else if (_motor_cmd_type == 0)  // nothing
  {
    motor_cmd_msg.request.model_state.reference_frame = "world";
    motor_cmd_msg.request.model_state = this->modelState;
  }
  else
  {
    motor_cmd_msg.request.model_state.reference_frame = "world";
    motor_cmd_msg.request.model_state = this->modelState;
    gzmsg << "WRONG MOTOR/THRUSTER COMMAND TYPE "
        << "(1 : voltage command, 2: power command)"
        << std::endl;
  }
  // publish command model state to gazebo/set_model_state topic
  this->commandPublisher["Model"].call(motor_cmd_msg);

  // -------------------------------------- //
  // ------- Buoyancy pump command -------- //
  // -------------------------------------- //
  // Force
  // double fluid_density = 1024.0;
  // double gravityAccel = 9.81;
  // double _pumped_volume = _msg->target_pumped_volume;
  // ignition::math::Vector3d buoyantForce(0.0, 0.0,
  //              _pumped_volume*gravityAccel*fluid_density);
  //              gzmsg << buoyantForce << std::endl;
  // this->model->GetLink(this->model->GetName() + 
  //          "/" + this->base_link_name)->SetForce(buoyantForce);
  // Velocity
  // gazebo_msgs::SetModelState pump_cmd_msg;
  // pump_cmd_msg.request.model_state.model_name = this->model->GetName();
  // pump_cmd_msg.request.model_state.pose = pose;
  // pump_cmd_msg.request.model_state.twist.linear.z =
  //                               _msg->target_pumped_volume*9.81;
  // pump_cmd_msg.request.model_state.reference_frame = "world";
  // publish command model state to gazebo/set_model_state topic
  // this->commandPublisher["Model"].call(pump_cmd_msg);
}

// /////////////////////////////////////////////////
// void DirectKinematicsROSPlugin::ConveyRudderVisualCommand(
//   const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
// {
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
// }

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::ConveyModelState()
{
  // Calculate pose
  this->modelXYZ = 
      ignition::math::Vector3d(this->modelState.pose.position.x,
                               this->modelState.pose.position.y,
                               this->modelState.pose.position.z);
  this->modelRPY = calculateRPY(this->modelState.pose.orientation.x,
                                this->modelState.pose.orientation.y,
                                this->modelState.pose.orientation.z,
                                this->modelState.pose.orientation.w);

  frl_vehicle_msgs::UwGliderStatus status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.latitude = this->sensorLatitude;
  status_msg.longitude = this->sensorLongitude;
  status_msg.roll = this->modelRPY.X();
  status_msg.pitch = this->modelRPY.Y();
  status_msg.yaw = this->modelRPY.Z();
  status_msg.heading = this->modelRPY.Z();
  status_msg.depth = this->modelXYZ.Z();
  status_msg.altitude = this->sensorAltitude;
  status_msg.motor_power = this->motorPower; 
  status_msg.rudder_angle = this->rudderAngle;
  // status_msg.battery_position = 
  // status_msg.pumped_volume = 
  
  this->statePublisher.publish(status_msg);
}



/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::writeCSVLog()
{
// CSV log write stream
if (this->writeLogFlag)
{
  double time = this->time.Double();
  if (this->writeCounter == 0)
  {
    writeLog.open("/tmp/DirectKinematicsLog.csv");
    writeLog << "# Hybrid Glider Plugin Log\n";
    writeLog << "# t,x,y,z,p,q,r,altitude\n";
    writeLog.close();
    this->writeCounter = this->writeCounter + 1;
  }
  if (floor(time * 10) == time * 10)
  {
    writeLog.open("/tmp/DirectKinematicsLog.csv", std::ios_base::app);
    writeLog << time << ","
            << this->modelXYZ.X() << "," << this->modelXYZ.Y() << "," 
            << this->modelXYZ.Z() << "," << this->modelRPY.X() << ","
            << this->modelRPY.Y() << "," << this->modelRPY.Z() << ","
            << this->sensorAltitude << "\n";
    writeLog.close();
  }
}
}

/////////////////////////////////////////////////
ignition::math::Vector3<double> DirectKinematicsROSPlugin::calculateRPY
                                (double x, double y, double z, double w)
{
  ignition::math::Vector3<double> vR;
  ignition::math::Vector4<double> q(0.0, 0.0, 0.0, 0.0);
  q.X() = x; q.Y() = y; q.Z() = z; q.W() = w;
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.X() * q.Y() + q.Z() * q.W());
  double cosr_cosp = 1 - 2 * (q.Y() * q.Y() + q.Z() * q.Z());
  vR.X() = std::atan2(sinr_cosp, cosr_cosp);
  // pitch (y-axis rotation)
  double sinp = 2 * (q.X() * q.Z() - q.W() * q.Y());
  if (std::abs(sinp) >= 1)
  vR.Y() = - std::copysign(M_PI / 2, sinp);  // use 90 deg if out
  else
  vR.Y() = - std::asin(sinp);
  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.X() * q.W() + q.Y() * q.Z());
  double cosy_cosp = 1 - 2 * (q.Z() * q.Z() + q.W() * q.W());
  vR.Z() = std::atan2(siny_cosp, cosy_cosp) - M_PI;
  // SUBTRACTION M_PI needs investigation
  return vR;
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::UpdateDVLSensorOnOff
                      (const std_msgs::Bool::ConstPtr &_msg)
{
  this->DVLOnOff = _msg->data;
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::UpdateDVLSensorData
                      (const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr &_msg)
{
  if (this->DVLOnOff)
    this->sensorAltitude = _msg->altitude;
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::UpdateGPSSensorOnOff
                      (const std_msgs::Bool::ConstPtr &_msg)
{
  this->GPSOnOff = _msg->data;
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::UpdateGPSSensorData
                      (const sensor_msgs::NavSatFix::ConstPtr &_msg)
{
  if (this->GPSOnOff)
    this->sensorLatitude = _msg->latitude;
    this->sensorLongitude = _msg->longitude;
}

GZ_REGISTER_MODEL_PLUGIN(DirectKinematicsROSPlugin)
}
