/*
 * Copyright 2020 Naval Postgraduate School
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


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

  if (_sdf->HasElement("fluid_density"))
    this->fluidDensity = _sdf->Get<double>("fluid_density");
  else
    this->fluidDensity = 1024.0;

  if (_sdf->HasElement("gravity"))
    this->gravity = _sdf->Get<double>("gravity");
  else
    this->gravity = 9.81;

  // If fluid topic is available, subscribe to it
  if (_sdf->HasElement("flow_velocity_topic"))
  {
    std::string flowTopic = _sdf->Get<std::string>("flow_velocity_topic");
    GZ_ASSERT(!flowTopic.empty(),
              "Fluid velocity topic tag cannot be empty");
    gzmsg << "Subscribing to current velocity topic: " << flowTopic
          << std::endl;
    this->flowSubscriber = this->rosNode->subscribe<geometry_msgs::TwistStamped>
          (flowTopic, 10 ,boost::bind(&DirectKinematicsROSPlugin::UpdateFlowVelocity, this, _1));
  }
  if (_sdf->HasElement("use_global_ocean_current"))
    this->useGlobalCurrent = _sdf->Get<bool>("use_global_ocean_current");
  else
    this->useGlobalCurrent = false;

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
    this->model->GetName() + "/gps/state", 10,
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

  // Check dual props and advertize publishers for visual effects
  if (this->model->GetJoint(this->model->GetName() + "/thruster_1_joint"))
  {
    this->dualProps = true;
    this->propVisualPubs["left"] =
        this->rosNode->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(
          this->model->GetName() + "/thrusters/0/input", 10);
    this->propVisualPubs["right"] =
        this->rosNode->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(
          this->model->GetName() + "/thrusters/1/input", 10);
  }
  else
  {
    this->dualProps = false;
    this->propVisualPubs["single"] =
        this->rosNode->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(
          this->model->GetName() + "/thrusters/0/input", 10);
  }

  // ---------------- READ Kinematics coefficients ---------------- //
  if (_sdf->HasElement("f_pitch_battpos_cal_m"))
    this->f_pitch_battpos_cal_m = _sdf->Get<double>("f_pitch_battpos_cal_m");
  else
    this->f_pitch_battpos_cal_m = 1.2565;
  if (_sdf->HasElement("f_pitch_battpos_cal_b"))
    this->f_pitch_battpos_cal_b = _sdf->Get<double>("f_pitch_battpos_cal_b");
  else
    this->f_pitch_battpos_cal_b = -0.055;
  if (_sdf->HasElement("Area"))
    this->Area = _sdf->Get<double>("Area");
  else
    this->Area = 0.0345;
  if (_sdf->HasElement("C_D"))
    this->C_D = _sdf->Get<double>("C_D");
  else
    this->C_D = 0.2534;
  if (_sdf->HasElement("C_L"))
    this->C_L = _sdf->Get<double>("C_L");
  else
    this->C_L = 0.4160;
  if (_sdf->HasElement("f_thruster_voltage_v1"))
    this->f_thruster_voltage_v1 = _sdf->Get<double>("f_thruster_voltage_v1");
  else
    this->f_thruster_voltage_v1 = 1.8953;
  if (_sdf->HasElement("f_thruster_voltage_v2"))
    this->f_thruster_voltage_v2 = _sdf->Get<double>("f_thruster_voltage_v2");
  else
    this->f_thruster_voltage_v2 = -1.995;
  if (_sdf->HasElement("f_thruster_voltage_v3"))
    this->f_thruster_voltage_v3 = _sdf->Get<double>("f_thruster_voltage_v3");
  else
    this->f_thruster_voltage_v3 = 1.8701;
  if (_sdf->HasElement("f_thruster_power_w1"))
    this->f_thruster_power_w1 = _sdf->Get<double>("f_thruster_power_w1");
  else
    this->f_thruster_power_w1 = -0.020919;
  if (_sdf->HasElement("f_thruster_power_w2"))
    this->f_thruster_power_w2 = _sdf->Get<double>("f_thruster_power_w2");
  else
    this->f_thruster_power_w2 = 1.4699;
  if (_sdf->HasElement("f_thruster_power_w3"))
    this->f_thruster_power_w3 = _sdf->Get<double>("f_thruster_power_w3");
  else
    this->f_thruster_power_w3 = 0.97895;

  // Coordinate transform functions : base_link
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

  // Initiate variables
  this->prev_pitch = 0.0;
  this->prev_yaw = 0.0;
  this->prev_motorPower = 0.0;

  // Free surface detection
  this->buoyancyFlag = true; // Initialize buoyancy engine
  this->link = this->model->GetLink(this->model->GetName() + "/" + this->base_link_name);
  this->boundingBox = link->BoundingBox();
  this->isSubmerged = true;

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

  // Apply ocean current
  this->applyOceanCurrent();

  // Send status
  this->ConveyModelState();

  // Check submergence
  this->CheckSubmergence();

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
  model_state_msg.request.relative_entity_name = "world";
  this->stateSubscriber["Model"].call(model_state_msg);
  this->modelState.model_name = this->model->GetName();
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
  // Convey commands to functions
  this->ConveyKinematicsCommands(_msg);

  // Print gzmsg
  gzmsg << "Control msg detected! at "
        << this->time << " s" << std::endl;
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::ConveyKinematicsCommands(
             const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
{
  // ---------------------------------------- //
  // ----------- Previous state ------------- //
  // ---------------------------------------- //
  gazebo_msgs::SetModelState cmd_msg;
  cmd_msg.request.model_state.model_name = this->model->GetName();
  cmd_msg.request.model_state = this->modelState;

  // --------------------------------------- //
  // ----------- PITCH COMMAND ------------- //
  // --------------------------------------- //
  double target_pitch = this->prev_pitch;
  switch(_msg->pitch_cmd_type)
  {
    case frl_vehicle_msgs::UwGliderCommand::PITCH_CMD_BATT_POS:
    {
      this->battpos = _msg->target_pitch_value/39.3701*1000;  // [m to inch]
      double target_pitch_rad
                = this->f_pitch_battpos_cal_m * this->battpos + this->f_pitch_battpos_cal_b;
      target_pitch = target_pitch_rad;
      break;
    }

    case frl_vehicle_msgs::UwGliderCommand::PITCH_CMD_TARGET_ONCE:
    // In the kinematic case, _ONCE and _SERVO are equivalent
    case frl_vehicle_msgs::UwGliderCommand::PITCH_CMD_TARGET_SERVO:
      target_pitch = _msg->target_pitch_value;
      break;

    case frl_vehicle_msgs::UwGliderCommand::PITCH_CMD_NONE:
      target_pitch = this->prev_pitch;
      break;

    default:
      gzmsg << "WRONG PITCH COMMAND TYPE "
          << "(1 : battery position, 2: target once, 3: target servo)"
          << std::endl;
      break;
  }
  this->prev_pitch = target_pitch;

  // --------------------------------------- //
  // ----------- RUDDER CONTROL ------------ //
  // --------------------------------------- //
  double target_yaw = this->prev_yaw;
  switch(_msg->rudder_control_mode)
  {
    case frl_vehicle_msgs::UwGliderCommand::RUDDER_CONTROL_HEADING:
      target_yaw = M_PI/2 - _msg->target_heading;
      break;

    case frl_vehicle_msgs::UwGliderCommand::RUDDER_CONTROL_ANGLE:
    {
      double rudderAngleZero = 0.0;
      double rudderAnglePort = M_PI/6.0;
      double rudderAngleStbd = -M_PI/6.0;
      switch(_msg->rudder_angle)
      {
        case frl_vehicle_msgs::UwGliderCommand::RUDDER_ANGLE_CENTER:
          target_yaw = this->prev_yaw + rudderAngleZero;
          break;

        case frl_vehicle_msgs::UwGliderCommand::RUDDER_ANGLE_PORT:
          target_yaw = this->prev_yaw + rudderAnglePort;
          break;

        case frl_vehicle_msgs::UwGliderCommand::RUDDER_ANGLE_STBD:
          target_yaw = this->prev_yaw + rudderAngleStbd;
          break;

        case frl_vehicle_msgs::UwGliderCommand::RUDDER_ANGLE_DIRECT:
          target_yaw = this->prev_yaw + _msg->target_rudder_angle;
          break;

        default:
          target_yaw = this->prev_yaw;
          gzmsg << "WRONG RUDDER ANGLE COMMAND "
            << "(1: center, 2: port, 3: staboard, 4: direct)"
            << std::endl; break;
      }
      break;
    }

    case frl_vehicle_msgs::UwGliderCommand::RUDDER_CONTROL_NONE:
      target_yaw = this->prev_yaw;
      break;

    default:
      gzmsg << "WRONG RUDDER COMMAND TYPE "
          << "(1 : control heading, 2: control angle)"
          << std::endl;
      break;
  }
  this->prev_yaw = target_yaw;

  // --------------------------------------- //
  // ------- MOTOR/THRUSTER COMMAND -------- //
  // --------------------------------------- //
  this->motorPower = this->prev_motorPower;
  switch(_msg->motor_cmd_type)
  {
    case frl_vehicle_msgs::UwGliderCommand::MOTOR_CMD_VOLTAGE:
      this->calcThrusterForce(_msg->motor_cmd_type,
                              _msg->target_motor_cmd);
      this->prev_motorPower = this->motorPower;
      break;

    case frl_vehicle_msgs::UwGliderCommand::MOTOR_CMD_POWER:
      this->calcThrusterForce(_msg->motor_cmd_type,
                              _msg->target_motor_cmd);
      this->prev_motorPower = this->motorPower;
      break;

    case frl_vehicle_msgs::UwGliderCommand::MOTOR_CMD_NONE:
      this->motorPower = this->prev_motorPower;
      break;

    default:
      gzmsg << "WRONG THRUSTER COMMAND TYPE "
          << "(1 : voltage control, 2: power control)"
          << std::endl;
      break;

  }
  double v_thrust = sqrt((this->prev_motorPower)/(fluidDensity*Area*C_D));

  // ---------------------------------------- //
  // ------- Caclulate flight model --------- //
  // ----- using pumped volume and AoA ------ //
  // ---------------------------------------- //
  double m_0;
  double alpha, xi, theta;
  double vx,vz,v_kernel;

  if (_msg->target_pumped_volume != 0.0)
  {
    this->prev_pumpVol = _msg->target_pumped_volume;
  }

  if (this->prev_pumpVol != 0.0)
  {
    m_0 = this->prev_pumpVol/1000000.0*this->fluidDensity;

    if (target_pitch > 0.0)
    {
      alpha = -3.0/180.0*M_PI;
    }
    else
    {
      alpha = +3.0/180.0*M_PI;
    }

    xi = target_pitch - alpha;
    if (m_0 > 0)
    {
      v_kernel = sqrt((2*m_0*this->gravity)/(this->fluidDensity*Area));
      vz = v_kernel*C_D/C_L/sqrt(C_L);
      if (-sin(xi)/C_D > 0)
        vx = v_kernel*sqrt(-sin(xi)/C_D)*cos(xi);
      else
        vx = 0.0;
    }
    else
    {
      v_kernel = sqrt((-2*m_0*this->gravity)/(this->fluidDensity*Area));
      vz = -v_kernel*C_D/C_L/sqrt(C_L);
      if (sin(xi)/C_D > 0)
        vx = v_kernel*sqrt(sin(xi)/C_D)*cos(xi);
      else
        vx = 0.0;
    }
  }

  // -------------------------------------- //
  // -------   Send message call   -------- //
  // -------------------------------------- //
  ignition::math::Quaterniond target_orientation;
  target_orientation.Euler(0.0, target_pitch, target_yaw);
  cmd_msg.request.model_state.pose.orientation.x = target_orientation.X();
  cmd_msg.request.model_state.pose.orientation.y = target_orientation.Y();
  cmd_msg.request.model_state.pose.orientation.z = target_orientation.Z();
  cmd_msg.request.model_state.pose.orientation.w = target_orientation.W();
  if (this->prev_pumpVol != 0.0)
  {
    double buoyancy_vel_x = vx*cos(target_pitch)+vz*sin(target_pitch);
    double buoyancy_vel_z = vx*sin(target_pitch)+vz*cos(target_pitch);
    cmd_msg.request.model_state.twist.linear.x = buoyancy_vel_x*cos(target_yaw) + v_thrust*cos(target_pitch)*cos(target_yaw);
    cmd_msg.request.model_state.twist.linear.y = buoyancy_vel_x*sin(target_yaw) + v_thrust*cos(target_pitch)*sin(target_yaw);
    cmd_msg.request.model_state.twist.linear.z = vx*sin(target_pitch)+vz*cos(target_pitch) - v_thrust*sin(target_pitch);
  }
  else
  {
    cmd_msg.request.model_state.twist.linear.x = v_thrust*cos(target_pitch)*cos(target_yaw);
    cmd_msg.request.model_state.twist.linear.y = v_thrust*cos(target_pitch)*sin(target_yaw);
    cmd_msg.request.model_state.twist.linear.z = -v_thrust*tan(target_pitch);
  }
  this->commandPublisher["Model"].call(cmd_msg);
  this->modelState = cmd_msg.request.model_state;

  // Save model velocity right after commend published
  this->modelVel = ignition::math::Vector3d(
                    this->link->WorldLinearVel().X(),
                    this->link->WorldLinearVel().Y(),
                    this->link->WorldLinearVel().Z());
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::applyOceanCurrent()
{
  this->link->SetLinearVel(ignition::math::Vector3d(
            this->modelVel.X() + this->flowVelocity.Y(),
            this->modelVel.Y() + this->flowVelocity.X(),
            this->modelVel.Z() + this->flowVelocity.Z()));
}

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
  status_msg.roll = this->modelRPY.Z();
  status_msg.pitch = this->modelRPY.Y();
  status_msg.yaw = this->modelRPY.X();
  status_msg.heading = M_PI/2 - this->modelRPY.X();
  if (status_msg.heading < 0)
  {
    status_msg.heading += 2 * M_PI;
  }
  status_msg.depth = - this->modelXYZ.Z();
  status_msg.altitude = this->sensorAltitude;
  status_msg.motor_power = this->motorPower;
  status_msg.rudder_angle = this->rudderAngle;
  status_msg.nav_sat_fix.status.status = this->GPSOnOff;
  status_msg.nav_sat_fix.latitude = this->sensorLatitude;
  status_msg.nav_sat_fix.longitude = this->sensorLongitude;
  status_msg.nav_sat_fix.altitude = this->sensorAltitude;
  status_msg.pumped_volume = this->prev_pumpVol;
  status_msg.battery_position = this->battpos;

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
    int prec = std::numeric_limits<double>::digits10+2; // generally 17
    writeLog.open("/tmp/DirectKinematicsLog.csv", std::ios_base::app);
    writeLog << std::setprecision(prec) << time << ","
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
  vR.Z() = std::atan2(siny_cosp, cosy_cosp) + M_PI;
  // yaw value needs investigation
  if (vR.Z() > 6.283185)
    vR.Z() = vR.Z() - 2*M_PI;
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

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::PropRotate(double thrustPower)
{
  uuv_gazebo_ros_plugins_msgs::FloatStamped input_message;
  input_message.header.stamp = ros::Time::now();
  input_message.data = thrustPower;
  if (this->dualProps)
  {
    this->propVisualPubs["left"].publish(input_message);
    this->propVisualPubs["right"].publish(input_message);
  }
  else
  {
    this->propVisualPubs["single"].publish(input_message);
  }
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::UpdateFlowVelocity
      (const geometry_msgs::TwistStamped::ConstPtr &_msg)
{
  if (this->useGlobalCurrent)
  {
    this->flowVelocity.X() = _msg->twist.linear.x;
    this->flowVelocity.Y() = _msg->twist.linear.y;
    this->flowVelocity.Z() = _msg->twist.linear.z;
  }
}

/////////////////////////////////////////////////
ignition::math::Vector3d DirectKinematicsROSPlugin::ToNED(ignition::math::Vector3d _vec)
{
  ignition::math::Vector3d output = _vec;
  output.Y() = -1.0 * output.Y();
  output.Z() = -1.0 * output.Z();
  return output;
}

/////////////////////////////////////////////////
ignition::math::Vector3d DirectKinematicsROSPlugin::FromNED(ignition::math::Vector3d _vec)
{
  return this->ToNED(_vec);
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::calcThrusterForce(int cmd_type, double cmd_value)
{
  // 1 : voltage command, 2: power command
  if (cmd_type == 1)  // voltage command
  {
    double v1 = this->f_thruster_voltage_v1;
    double v2 = this->f_thruster_voltage_v2;
    double v3 = this->f_thruster_voltage_v3;
    this->motorPower = v1*(cmd_value*cmd_value)+ v2*cmd_value + v3;
    if (cmd_value == 0.0)
      this->motorPower = 0.0;
    this->motorPower = this->motorPower * 2.0;  // Dual thrusters
    // rotate propellers for visual effets
    this->PropRotate(this->motorPower);
  }
  else if (cmd_type == 2)  // power command
  {
    double w1 = this->f_thruster_power_w1;
    double w2 = this->f_thruster_power_w2;
    double w3 = this->f_thruster_power_w3;
    this->motorPower = w1*(cmd_value*cmd_value)+ w2*cmd_value + w3;
    this->motorPower = this->motorPower * 2.0;  // Dual thrusters
    if (cmd_value == 0.0)
      this->motorPower = 0.0;
    // rotate propellers for visual effets
    this->PropRotate(this->motorPower);
  }
  else
  {
    gzmsg << "WRONG MOTOR/THRUSTER COMMAND TYPE "
        << "(1 : voltage command, 2: power command)"
        << std::endl;
  }
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::CheckSubmergence()
{
  double height = this->boundingBox.ZLength();
  double z = this->link->WorldPose().Pos().Z();
  bool previousState = this->isSubmerged;

  if (previousState)
    this->lastPose = this->link->WorldPose();

  // Submerged vessel
  if (z + height / 2 > 0 && z < 0)
  {
    this->isSubmerged = false;
  }
  else if (z + height / 2 < 0)
  {
    this->isSubmerged = true;
  }

  if (!this->isSubmerged)
  {
    if (prev_pitch < 0.0 || this->buoyancyForce.Z() > 0.0 || prev_pumpVol > 0.0)
    {
      ignition::math::Quaterniond quat;
      quat.Euler(0.0, 0.0, prev_yaw);
      this->lastPose.Rot() = quat;
      this->link->SetWorldPose(this->lastPose);
      this->link->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
      this->link->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
      this->link->ResetPhysicsStates();
    }
  }
}


GZ_REGISTER_MODEL_PLUGIN(DirectKinematicsROSPlugin)
}
