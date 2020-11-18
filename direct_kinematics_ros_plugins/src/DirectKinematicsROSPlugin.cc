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


  // Coordinate transform functions
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

  this->link = this->model->GetLink(this->model->GetName() + "/" + this->base_link_name);
  // Read dynacmis flags and parameters from SDF configuration
  if (_sdf->HasElement("dynamics"))
  {
    sdf::ElementPtr sdfDynamics = _sdf->GetElement("dynamics");
    this->r_w = sdfDynamics->Get<double>("ballast_radius");
    this->l_h = sdfDynamics->Get<double>("hull_length");
    this->r_h = sdfDynamics->Get<double>("hull_radius");
    this->m_h = sdfDynamics->Get<double>("hull_mass");
    this->m_s = sdfDynamics->Get<double>("shifter_mass");
    this->x_s_o = sdfDynamics->Get<double>("default_mass_position");
    this->x_w_o = sdfDynamics->Get<double>("default_ballast_position");
    this->x_s_o_max = sdfDynamics->Get<double>("max_mass_position");
    this->vol_w_max = (sdfDynamics->Get<double>("max_ballast_volume"))/(M_PI*(this->r_w)*(this->r_w));
    this->cog = Str2Vector(sdfDynamics->Get<std::string>("center_of_gravity"));
    this->cob = Str2Vector(sdfDynamics->Get<std::string>("center_of_buoyancy"));

    // Load linear damping coefficients, if provided. Otherwise, the linear
    // damping matrix is set to zero
    if (sdfDynamics->HasElement("linear_damping"))
      this->DLin = Str2Matrix6(sdfDynamics->Get<std::string>("linear_damping"));
    else
      gzmsg << "Using linear damping NULL" << std::endl;

    // Initial calculation
    this->massPos = sdfDynamics->Get<double>("initial_mass_position");
    this->pumpPos = sdfDynamics->Get<double>("initial_ballast_position");
    this->motorPower = 0.0;

    // Initialize filtered acceleration & last velocity
    this->filteredAcc.setZero();
    this->lastVelRel.setZero();
  }

  this->prev_pitch = 0.0;
  this->prev_yaw = 0.0;
  this->prev_motorPower = 0.0;

  if (_sdf->HasElement("use_dynamics"))
    this->dynamics = _sdf->Get<bool>("use_dynamics");
  else
    this->dynamics = false;     // Initiate without dynamics on

  this->buoyancyFlag = true; // Initialize buoyancy engine

  // Free surface detection
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

  // Update dynamics
  if (this->dynamics)
    this->CalculateDynamics(this->massPos, this->pumpPos, this->motorPower);

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
  // // Determine kinematics or dynamics
  // int _pitch_cmd_type = _msg->pitch_cmd_type;
  // if (_pitch_cmd_type == 1)  // battery position control
  // {
  //   this->dynamics = true;
  // }
  // else if (_pitch_cmd_type == 2 || _pitch_cmd_type == 3)
  // {
  //   this->dynamics = false;
  // }

  // Convey commands to functions accordingly (Kinematics, dynamics)
  if(this->dynamics)
    this->ConveyDynamicsCommands(_msg);
  else
    this->ConveyKinematicsCommands(_msg);

  // if (this->rudderExist)
  // {
  //   this->ConveyRudderVisualCommand(_msg);
  // }

  // Print gzmsg
  gzmsg << "Control msg detected! at "
        << this->time << " s" << std::endl;
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::ConveyKinematicsCommands(
             const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
{
  // ignition::math::Quaterniond orientation(this->modelState.pose.orientation.x,
  //                                         this->modelState.pose.orientation.y,
  //                                         this->modelState.pose.orientation.z,
  //                                         this->modelState.pose.orientation.w);
  // ignition::math::Vector3d orientation_euler = orientation.Euler();
  // gzmsg << "=====================" << std::endl;
  // gzmsg << orientation_euler << std::endl;
  // ignition::math::Quaterniond orientation_quat;
  // orientation_quat.Euler(orientation_euler.X(),
  //                        orientation_euler.Y(),
  //                        orientation_euler.Z());

  // gazebo_msgs::SetModelState cmd_msg;
  // cmd_msg.request.model_state = this->modelState;
  // cmd_msg.request.model_state.pose.orientation.x = orientation_quat.X();
  // cmd_msg.request.model_state.pose.orientation.y = orientation_quat.Y();
  // cmd_msg.request.model_state.pose.orientation.z = orientation_quat.Z();
  // cmd_msg.request.model_state.pose.orientation.w = orientation_quat.W();

  // ignition::math::Quaterniond orientation(this->modelState.pose.orientation.x,
  //                                         this->modelState.pose.orientation.y,
  //                                         this->modelState.pose.orientation.z,
  //                                         this->modelState.pose.orientation.w);
  // ignition::math::Vector3d orientation_euler = orientation.Euler();
  // gzmsg << "=====================" << std::endl;
  // gzmsg << orientation_euler << std::endl;
  // gzmsg << "---------------------" << std::endl;
  // gzmsg << orientation.X() << std::endl;
  // gzmsg << orientation.Y() << std::endl;
  // gzmsg << orientation.Z() << std::endl;
  // gzmsg << orientation.W() << std::endl<< std::endl;
  // gazebo_msgs::SetModelState cmd_msg;
  // cmd_msg.request.model_state = this->modelState;
  // cmd_msg.request.model_state.pose.orientation.x = orientation.X();
  // cmd_msg.request.model_state.pose.orientation.y = orientation.Y();
  // cmd_msg.request.model_state.pose.orientation.z = orientation.Z();
  // cmd_msg.request.model_state.pose.orientation.w = orientation.W();

  // this->commandPublisher["Model"].call(cmd_msg);

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
  int _pitch_cmd_type = _msg->pitch_cmd_type;
  // 1 : battery position, 2: target once , 3: target servo
  if (_pitch_cmd_type == 1)  // battery position
  {
    double battpos = _msg->target_pitch_value/39.3701*1000;  // [m to inch]
    double target_pitch_rad
              = this->f_pitch_battpos_cal_m * battpos + this->f_pitch_battpos_cal_b;
    target_pitch = target_pitch_rad;
  }
  else if (_pitch_cmd_type == 2)  // target once
  {
    target_pitch = _msg->target_pitch_value;
  }
  else if (_pitch_cmd_type == 3)  // target servo
  {
    // nothing now
  }
  else if (_pitch_cmd_type == 0)  // nothing
  {
    target_pitch = this->prev_pitch;
  }
  else
  {
    gzmsg << "WRONG PITCH COMMAND TYPE "
        << "(1 : battery position, 2: target once, 3: target servo)"
        << std::endl;
  }
  this->prev_pitch = target_pitch;

  // --------------------------------------- //
  // ----------- RUDDER CONTROL ------------ //
  // --------------------------------------- //
  double target_yaw;
  int _rudder_control_mode = _msg->rudder_control_mode;
  // 1 : control heading, 2: control angle
  if (_rudder_control_mode == 1)  // control heading
  {
    target_yaw = _msg->target_heading;
  }
  else if (_rudder_control_mode == 2)  // control angle
  {
    int _rudder_angle = _msg->rudder_angle;
    // Rudder angle control
    // 1: center, 2: port, 3: staboard, 4: direct
    double rudderAngleZero = 0.0;
    double rudderAnglePort = M_PI/6.0;
    double rudderAngleStbd = -M_PI/6.0;
    double rudderAngleTarget = _msg->target_rudder_angle;
    if (_rudder_angle == 1)  // center
    {
      target_yaw = this->prev_yaw + rudderAngleZero;
    }
    else if (_rudder_angle == 2)  // port
    {
      target_yaw = this->prev_yaw + rudderAnglePort;
    }
    else if (_rudder_angle == 3)  // staboard
    {
      target_yaw = this->prev_yaw + rudderAngleStbd;
    }
    else if (_rudder_angle == 4)  // direct
    {
      target_yaw = this->prev_yaw + rudderAngleTarget;
    }
    else if (_rudder_angle == 0)  // nothing
    {
      target_yaw = this->prev_yaw;
    }
    else
    {
      gzmsg << "WRONG RUDDER ANGLE COMMAND "
        << "(1: center, 2: port, 3: staboard, 4: direct)"
        << std::endl;
    }
  }
  else if (_rudder_control_mode == 0)  // nothing
  {
    target_yaw = this->prev_yaw;
  }
  else
  {
    gzmsg << "WRONG RUDDER COMMAND TYPE "
        << "(1 : control heading, 2: control angle)"
        << std::endl;
  }
  this->prev_yaw = target_yaw;

  // --------------------------------------- //
  // ------- MOTOR/THRUSTER COMMAND -------- //
  // --------------------------------------- //
  int _motor_cmd_type = _msg->motor_cmd_type;
  double _motor_cmd_value = _msg->target_motor_cmd;
  // 1 : voltage control, 2: power control
  if (_motor_cmd_type == 1 || 2)
  {
    this->calcThrusterForce(_motor_cmd_type, _motor_cmd_value);
    this->prev_motorPower = this->motorPower;
  }
  else if (_motor_cmd_type == 0)  // nothing
  {
    // nothing
  }
  else
  {
    gzmsg << "WRONG THRUSTER COMMAND TYPE "
        << "(1 : voltage control, 2: power control)"
        << std::endl;
  }
  double v_thrust = sqrt((2*this->prev_motorPower)/(fluidDensity*Area*C_D));

  // --------------------------------------- //
  // ------- Caclulate flight model -------- //
  // --------------------------------------- //
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
    cmd_msg.request.model_state.twist.linear.z = vx*sin(target_pitch)+vz*cos(target_pitch) - v_thrust*sin(target_pitch);;
  }
  else
  {
    cmd_msg.request.model_state.twist.linear.x = v_thrust*cos(target_pitch)*cos(target_yaw);
    cmd_msg.request.model_state.twist.linear.y = v_thrust*cos(target_pitch)*sin(target_yaw);
    cmd_msg.request.model_state.twist.linear.z = -v_thrust*tan(target_pitch);
  }
  this->commandPublisher["Model"].call(cmd_msg);
  this->modelState = cmd_msg.request.model_state;

  // Check submergence
  // this->link->AddForceAtRelativePosition(this->buoyancyForce, this->cog);
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
  status_msg.roll = this->modelRPY.Z();
  status_msg.pitch = this->modelRPY.Y();
  status_msg.yaw = this->modelRPY.X();
  status_msg.heading = this->modelRPY.X();
  status_msg.depth = - this->modelXYZ.Z();
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
void DirectKinematicsROSPlugin::ConveyDynamicsCommands(
             const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
{
  // --------------------------------------------- //
  // ---------- MOTOR/THRUSTER COMMAND ----------- //
  // --------------------------------------------- //
  int _motor_cmd_type = _msg->motor_cmd_type;
  double _motor_cmd_value = _msg->target_motor_cmd;
  this->calcThrusterForce(_motor_cmd_type, _motor_cmd_value);

  // --------------------------------------------- //
  // ----------- Dynamics Calculation ------------ //
  // --------------------------------------------- //
  this->massPos = _msg->target_pitch_value;
  this->pumpPos = _msg->target_pumped_volume/1000000.0/(M_PI*(this->r_w)*(this->r_w));
  this->CalculateDynamics
        (this->massPos, this->pumpPos, this->motorPower);
}
/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::CalculateDynamics(
             double _massPos, double _pumpPos, double _thrustPower)
{
  // ---- Mass calculation ----- //
  // Ballast volume and mass
  double V_B = -_pumpPos*M_PI*(this->r_w)*(this->r_w);
  double m_w = V_B*this->fluidDensity;

  // Total vehicle mass
  double m = this->m_h + this->m_s + m_w;
  this->m = m;

  // Moving mass position
  double x_s = this->x_s_o + _massPos;

  // Hull mass center (uniform hull mass distribution assumed)
  double x_h = -this->m_s/this->m_h*this->x_s_o;

  // Ballast tank mass center (pushing from front)
  double x_w = this->x_w_o+(V_B/(M_PI*(this->r_w)*(this->r_w)));

  // Center of gravity
  (this->cog).X() = (x_h*this->m_h + x_s*this->m_s + x_w*m_w)/(this->m_h+this->m_s+m_w);

  // ---- Inertial matrix calculation ----- //
  double a = this->l_h/2.0; // half the length
  double b = this->r_h;   // hull radius
  // inertial matrix (Fossen p.42 (2.156))
  double I_yy = 4.0/15.0*m*(b*b+a*a); double I_zz = I_yy;  double I_xx = 4.0/15.0*m*(b*b+b*b);
  this->link->GetInertial()->SetCoG((this->cog).X(),(this->cog).Y(),(this->cog).Z());
  this->link->GetInertial()->SetIXX(I_xx);
  this->link->GetInertial()->SetIYY(I_yy);
  this->link->GetInertial()->SetIZZ(I_zz);
  this->link->GetInertial()->SetMass(m);

  // ---- Added mass matrix calculation ----- //
  // computed according to the procedure in Fossen p. 41
  double e = 1.0-std::pow(b/a,2);
  double alpha_o = (2.0*(1.0-e*e)/std::pow(e,3))*(0.5*std::log((1.0+e)/(1.0-e))-e);
  double beta_o = 1.0/std::pow(e,2)-((1-std::pow(e,2))/(2.0*std::pow(e,3)))*std::log((1.0+e)/(1.0-e));
  double X_udot = -(alpha_o/(2.0-alpha_o))*this->m;
  double Y_vdot = -(beta_o/(2.0-beta_o))*this->m;
  double Z_wdot = Y_vdot;
  double K_pdot = 0.0;
  double M_qdot = -0.2*this->m*(std::pow(std::pow(b,2)-std::pow(a,2),2)*(alpha_o-beta_o))/
      (2.0*(std::pow(b,2)-std::pow(a,2))+(std::pow(b,2)+std::pow(a,2))*(beta_o-alpha_o));
  double N_rdot = M_qdot;

  // M_A_cg
  Eigen::Matrix6d Ma_cg = Eigen::Matrix6d::Identity();
  Ma_cg(0,0) = -X_udot;  Ma_cg(1,1) = -Y_vdot;  Ma_cg(2,2) = -Z_wdot;
  Ma_cg(3,3) = -K_pdot;  Ma_cg(4,4) = -M_qdot;  Ma_cg(5,5) = -N_rdot;

  // M_A = LeftSide*M_A_CG*RightSide
  Eigen::Vector3d cog = Eigen::Vector3d((this->cog).X(),(this->cog).Y(),(this->cog).Z());
  Eigen::Matrix3d S_r_cg = CrossProductOperator(cog);
  Eigen::Matrix6d LeftSide = Eigen::Matrix6d::Identity();
  LeftSide(0,3) = S_r_cg(0,0); LeftSide(0,4) = S_r_cg(0,1); LeftSide(0,5) = S_r_cg(0,2);
  LeftSide(1,3) = S_r_cg(1,0); LeftSide(1,4) = S_r_cg(1,1); LeftSide(1,5) = S_r_cg(1,2);
  LeftSide(2,3) = S_r_cg(2,0); LeftSide(2,4) = S_r_cg(2,1); LeftSide(2,5) = S_r_cg(2,2);
  Eigen::Matrix6d RightSide = Eigen::Matrix6d::Identity();
  Eigen::Matrix3d S_r_cg_T = S_r_cg.transpose();
  RightSide(0,3) = S_r_cg_T(0,0); RightSide(0,4) = S_r_cg_T(0,1); RightSide(0,5) = S_r_cg_T(0,2);
  RightSide(1,3) = S_r_cg_T(1,0); RightSide(1,4) = S_r_cg_T(1,1); RightSide(1,5) = S_r_cg_T(1,2);
  RightSide(2,3) = S_r_cg_T(2,0); RightSide(2,4) = S_r_cg_T(2,1); RightSide(2,5) = S_r_cg_T(2,2);
  Eigen::Matrix6d M_A = LeftSide*Ma_cg*RightSide;

  this->Ma = M_A;

  // Link's pose
  ignition::math::Pose3d pose;
  ignition::math::Vector3d linVel, angVel;
  pose = this->link->WorldPose();
  linVel = this->link->RelativeLinearVel();
  angVel = this->link->RelativeAngularVel();

  // Transform the flow velocity to the BODY frame
  ignition::math::Vector3d flowVel = pose.Rot().RotateVectorReverse(
    this->flowVelocity);

  Eigen::Vector6d velRel, acc;
  // Compute the relative velocity
  velRel = EigenStack(
    this->ToNED(linVel - flowVel),
    this->ToNED(angVel));

  // Update added Coriolis matrix
  this->ComputeAddedCoriolisMatrix(velRel, this->Ma, this->Ca);

  // Update damping matrix
  this->ComputeDampingMatrix(velRel, this->D);

  // Filter acceleration (see issue explanation above)
  this->ComputeAcc(velRel, this->time.Double(), 0.3);

  // We can now compute the additional forces/torques due to thisdynamic
  // effects based on Eq. 8.136 on p.222 of Fossen: Handbook of Marine Craft ...

  // Damping forces and torques
  Eigen::Vector6d damping = -this->D * velRel;

  // Added-mass forces and torques
  Eigen::Vector6d added = -this->Ma * this->filteredAcc;

  // Added Coriolis term
  Eigen::Vector6d cor = -this->Ca * velRel;

  // --- Calculate Rigid Body Diagonal terms --- //
  // Rigid Body mass diagonal term
  Eigen::Matrix3d m_S_r_cg = this->m * CrossProductOperator(cog);
  Eigen::Matrix6d M_RB = Eigen::Matrix6d::Identity() * this->m;
  M_RB(3,3) = I_xx; M_RB(4,4) = I_yy; M_RB(5,5) = I_zz;
  Eigen::Matrix6d M_RB_NoDiag = M_RB;
  M_RB(0,3) = -m_S_r_cg(0,0); M_RB(0,4) = -m_S_r_cg(0,1); M_RB(0,5) = -m_S_r_cg(0,2);
  M_RB(1,3) = -m_S_r_cg(1,0); M_RB(1,4) = -m_S_r_cg(1,1); M_RB(1,5) = -m_S_r_cg(1,2);
  M_RB(2,3) = -m_S_r_cg(2,0); M_RB(2,4) = -m_S_r_cg(2,1); M_RB(2,5) = -m_S_r_cg(2,2);
  M_RB(3,0) = m_S_r_cg(0,0); M_RB(3,1) = m_S_r_cg(0,1); M_RB(3,2) = m_S_r_cg(0,2);
  M_RB(4,0) = m_S_r_cg(1,0); M_RB(4,1) = m_S_r_cg(1,1); M_RB(4,2) = m_S_r_cg(1,2);
  M_RB(5,0) = m_S_r_cg(2,0); M_RB(5,1) = m_S_r_cg(2,1); M_RB(5,2) = m_S_r_cg(2,2);
  // Rigid Body Coriolis diagonal term
  Eigen::Matrix6d C_RB = Eigen::Matrix6d::Zero();
  Eigen::Vector6d ab = M_RB * velRel;
  Eigen::Matrix3d Sa = -1.0 * CrossProductOperator(ab.head<3>());
  C_RB << Eigen::Matrix3d::Zero(), Sa,
         Sa, -1.0 * CrossProductOperator(ab.tail<3>());
  Eigen::Matrix6d C_RB_NoDiag = Eigen::Matrix6d::Zero();
  ab = M_RB_NoDiag * velRel;
  Sa = -1.0 * CrossProductOperator(ab.head<3>());
  C_RB_NoDiag << Eigen::Matrix3d::Zero(), Sa,
         Sa, -1.0 * CrossProductOperator(ab.tail<3>());
  // Calculate diagonal effect as RHS force
  Eigen::Vector6d rigid_diag = -(M_RB-M_RB_NoDiag)*this->filteredAcc - (C_RB-C_RB_NoDiag)*velRel;

    // --- Calculate hull hydrodynamic force (Graver) --- //
  //angle of attack of hull
  double alpha_h;
  if (velRel(0) > 0.0)
      alpha_h = atan(velRel(2)/velRel(0));
  else
      alpha_h = 0.0;
  // Graver hydrodynamic forces for hull
  double A_h = M_PI*this->r_h*this->r_h;
  double C_D_h = 0.214 + 32.3*alpha_h*alpha_h;
  double C_L_h = 11.76*alpha_h + 4.6 * alpha_h*alpha_h;
  double C_M_h = 0.63*alpha_h;
  double F_h_kernel = std::pow(std::pow(velRel(0)*velRel(0)+velRel(2)*velRel(2),0.5),2);
  double F_D_h = 0.5*this->fluidDensity*A_h*F_h_kernel*C_D_h;
  double F_L_h = 0.5*this->fluidDensity*A_h*F_h_kernel*C_L_h;
  double F_M_h = 0.5*this->fluidDensity*A_h*F_h_kernel*C_M_h;
  Eigen::Vector6d hull_hydro; hull_hydro << F_L_h*sin(alpha_h)-F_D_h*cos(alpha_h), 0.0, -F_L_h*cos(alpha_h)-F_D_h*sin(alpha_h), 0.0, F_M_h, 0.0;

  // --- Calculate Buoyancy force (Hydrostatic restoring force, Fossen (2.168)) --- //
  // Get world pos eta(p,q,r)
  ignition::math::Vector3<double> vR(0.0, 0.0, 0.0);
  ignition::math::Vector4<double> q(0.0, 0.0, 0.0, 0.0);
  q.X() = pose.Rot().X();
  q.Y() = pose.Rot().Y();
  q.Z() = pose.Rot().Z();
  q.W() = pose.Rot().W();
  // roll (x-axis rotation)
  double sinr_cosp = 2.0 * (q.X() * q.Y() + q.Z() * q.W());
  double cosr_cosp = 1.0 - 2.0 * (q.Y() * q.Y() + q.Z() * q.Z());
  vR.X() = std::atan2(sinr_cosp, cosr_cosp);
  // pitch (y-axis rotation)
  double sinp = 2.0 * (q.X() * q.Z() - q.W() * q.Y());
  if (std::abs(sinp) >= 1.0)
    vR.Y() = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    vR.Y() = std::asin(sinp);
  // yaw (z-axis rotation)
  double siny_cosp = 2.0 * (q.X() * q.W() + q.Y() * q.Z());
  double cosy_cosp = 1.0 - 2.0 * (q.Z() * q.Z() + q.W() * q.W());
  vR.Z() = std::atan2(siny_cosp, cosy_cosp);
  // Hydrostatic restoring forces from Fossen (2.168)
  double W = this->m*this->gravity; // Gravitational force acting on center of gravity
  double B = (this->m_h+this->m_s)*this->gravity; // Buoyancy force acting on center of buoyancy
  ignition::math::Vector3d buoyancyForce, buoyancyTorque;
  this->buoyancyForce = ignition::math::Vector3d(
      (W-B)*sin(vR.Y()), -(W-B)*cos(vR.Y())*sin(vR.X()) , -(W-B)*cos(vR.Y())*cos(vR.X())
  );
  buoyancyTorque = ignition::math::Vector3d(
      -((this->cog).Y()*W-(this->cob).Y()*B)*cos(vR.Y())*cos(vR.X())+((this->cog).Z()*W-(this->cob).Z()*B)*cos(vR.Y())*sin(vR.X()),
      ((this->cog).Z()*W-(this->cob).Z()*B)*sin(vR.Y())+((this->cog).X()*W-(this->cob).X()*B)*cos(vR.Y())*cos(vR.X()),
      -((this->cog).X()*W-(this->cob).X()*B)*cos(vR.Y())*sin(vR.X())-((this->cog).Y()*W-(this->cob).Y()*B)*sin(vR.Y())
  );

  Eigen::Vector6d hydrostatic;
  hydrostatic << this->buoyancyForce.X(), this->buoyancyForce.Y(), this->buoyancyForce.Z(), buoyancyTorque.X(), buoyancyTorque.Y(), buoyancyTorque.Z();

  // All additional (compared to standard rigid body) Fossen terms combined.
  Eigen::Vector6d tau;
  Eigen::Vector6d thruster; thruster << _thrustPower, 0.0, 0.0, 0.0, 0.0, 0.0;

  tau = damping + added + cor + thruster + rigid_diag + hull_hydro - hydrostatic;

  // Added for stability for now
  // tau(1) = 0.0;
  // tau(3) = 0.0;
  // tau(5) = 0.0;

  // // Check submergence
  // this->CheckSubmergence();
  // if (!this->isSubmerged && tau(2) >= 0)
  // {
  //   tau(2) = 0.0;
  //   this->link->SetLinearVel(
  //     ignition::math::Vector3d(
  //       this->link->WorldLinearVel().X(),
  //       this->link->WorldLinearVel().Y(),
  //       0.0));
  // }

  GZ_ASSERT(!std::isnan(tau.norm()), "Hydrodynamic forces vector is nan");

  if (!std::isnan(tau.norm()))
  {
    // Convert the forces and moments back to Gazebo's reference frame
    ignition::math::Vector3d hydForce =
      this->FromNED(Vec3dToGazebo(tau.head<3>()));
    ignition::math::Vector3d hydTorque =
      this->FromNED(Vec3dToGazebo(tau.tail<3>()));

    // Forces and torques are also wrt link frame
    this->link->AddRelativeForce(hydForce);
    this->link->AddRelativeTorque(hydTorque);
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
void DirectKinematicsROSPlugin::ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel,
                                          Eigen::Matrix6d &_Ma,
                                          Eigen::Matrix6d &_Ca) const
{
  // This corresponds to eq. 6.43 on p. 120 in
  // Fossen, Thor, "Handbook of Marine Craft and Hydrodynamics and Motion
  // Control", 2011
  Eigen::Vector6d ab = _Ma * _vel;
  Eigen::Matrix3d Sa = -1.0 * CrossProductOperator(ab.head<3>());
  _Ca << Eigen::Matrix3d::Zero(), Sa,
         Sa, -1.0 * CrossProductOperator(ab.tail<3>());
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::ComputeDampingMatrix(const Eigen::Vector6d& _vel,
                                    Eigen::Matrix6d &_D) const
{
  // From Antonelli 2014: the viscosity of the fluid causes
  // the presence of dissipative drag and lift forces on the
  // body. A common simplification is to consider only linear
  // and quadratic damping terms and group these terms in a
  // matrix Drb

  _D.setZero();

  // Simplified only to consider DLin here.
  _D = -1.0 * this->DLin;
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::ComputeAcc(Eigen::Vector6d _velRel, double _time,
                                  double _alpha)
{
  // Compute Fossen's nu-dot numerically. We have to do this for now since
  // Gazebo reports angular accelerations that are off by orders of magnitude.
  double dt = _time - lastTime;

  if (dt <= 0.0)  // Extra caution to prevent division by zero
    return;

  Eigen::Vector6d acc = (_velRel - this->lastVelRel) / dt;

  // TODO  We only have access to the acceleration of the previous simulation
  //       step. The added mass will induce a strong force/torque counteracting
  //       it in the current simulation step. This can lead to an oscillating
  //       system.
  //       The most accurate solution would probably be to first compute the
  //       latest acceleration without added mass and then use this to compute
  //       added mass effects. This is not how gazebo works, though.
  this->filteredAcc = (1.0 - _alpha) * this->filteredAcc + _alpha * acc;

  lastTime = _time;
  this->lastVelRel = _velRel;
}

/////////////////////////////////////////////////
void DirectKinematicsROSPlugin::calcThrusterForce(int cmd_type, double cmd_value)
{
  // 1 : voltage command, 2: power command
  if (cmd_type == 1)  // voltage command
  {
    double v1 = 1.8953;
    double v2 = -1.995;
    double v3 = 1.8701;
    this->motorPower = v1*(cmd_value*cmd_value)+ v2*cmd_value + v3;
    if (cmd_value == 0.0)
      this->motorPower = 0.0;
    this->motorPower = this->motorPower * 2.0;  // Dual thrusters
    // rotate propellers for visual effets
    this->PropRotate(this->motorPower);
  }
  else if (cmd_type == 2)  // power command
  {
    double w1 = -0.020919;
    double w2 = 1.4699;
    double w3 = 0.97895;
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
    if (prev_pitch < 0 || this->buoyancyForce.Z() > 0.0)
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
