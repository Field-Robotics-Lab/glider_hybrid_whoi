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


#include <kinematics_ros_plugins/KinematicsROSPlugin.hh>

#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/rendering/rendering.hh>

namespace kinematics_ros
{
/////////////////////////////////////////////////
KinematicsROSPlugin::KinematicsROSPlugin()
{
}

/////////////////////////////////////////////////
KinematicsROSPlugin::~KinematicsROSPlugin()
{
  this->rosNode->shutdown();
  this->updateConnection.reset();

  // CSV log write stream close
  writeLog.close();
}

/////////////////////////////////////////////////
void KinematicsROSPlugin::Load(gazebo::physics::ModelPtr _model,
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
          (flowTopic, 10 ,boost::bind(&KinematicsROSPlugin::UpdateFlowVelocity, this, _1));
  }
  if (_sdf->HasElement("use_global_ocean_current"))
    this->useGlobalCurrent = _sdf->Get<bool>("use_global_ocean_current");
  else
    this->useGlobalCurrent = false;

  // Read projection coordinate for GDAL transform
  if (_sdf->HasElement("espg_projection"))
    _sdf->GetElement("espg_projection")->GetValue()->Get(this->espg_projection_);
  else
    this->espg_projection_ = 26987;

  // If surface vehicle
  if (_sdf->HasElement("surface_vehicle"))
    this->surfaceVehicle = _sdf->Get<bool>("surface_vehicle");
  else
    this->surfaceVehicle = false;

  // Subscribe command from outside
  ros::SubscribeOptions so =
  ros::SubscribeOptions::create<frl_vehicle_msgs::UwGliderCommand>(
      this->model->GetName() + "/kinematics/UwGliderCommand",
      1,
      boost::bind(&KinematicsROSPlugin::ConveyCommands, this, _1),
      ros::VoidPtr(), &this->commandSubQueue);
  this->commandSubscriber = this->rosNode->subscribe(so);
  // Spin up the queue helper thread.
  this->commandSubQueueThread = std::thread(std::bind(
      &KinematicsROSPlugin::commandSubThread, this));

  // Publisher for glider status
  this->statePublisher =
      this->rosNode->advertise<frl_vehicle_msgs::UwGliderStatus>(
        this->model->GetName() + "/kinematics/UwGliderStatus", 10);

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

  // ---------------- READ Dynamics coefficients ---------------- //
  if (_sdf->HasElement("use_dynamics"))
    this->dynamics = _sdf->Get<bool>("use_dynamics");
  else
    this->dynamics = false; // default off

  if (this->dynamics)
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

    // Initialize position/orientation and velocity;
    this->eta.setZero();
    this->eta_last.setZero();
    this->eta_dot.setZero();
    this->nu.setZero();
    this->nu_last.setZero();
    this->nu_dot.setZero();

    if (_sdf->HasElement("hard_code_input"))
      this->hardCodeInput = _sdf->Get<bool>("hard_code_input");
    else
      this->hardCodeInput = false;
  }

  // Initiate variables
  this->prev_pitch = 0.0;
  this->prev_yaw = 0.0;
  this->prev_motorPower = 0.0;
  this->flowVelocity = ignition::math::Vector3d(0.0, 0.0, 0.0);

  // Free surface detection
  this->buoyancyFlag = true; // Initialize buoyancy engine
  this->link = this->model->GetLink(this->model->GetName() + "/" + this->base_link_name);
  this->isSubmerged = true;

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
  this->prev_motorPower = 0.0;
  // this->modelVel = ignition::math::Vector3d(0.0, 0.0, 0.0);
  ignition::math::Pose3d model_pose = this->model->WorldPose();
  ignition::math::Vector3<double> model_RPY =
      calculateRPY(model_pose.Rot().X(), model_pose.Rot().Y(),
                  model_pose.Rot().Z(), model_pose.Rot().W());
  this->prev_yaw = model_RPY.Z();
  this->eta_last << model_pose.Pos().X(), model_pose.Pos().Y(), model_pose.Pos().Z(),
               model_RPY.X(), model_RPY.Y(), model_RPY.Z();

  // this->targetLinearVel_prev = Eigen::Vector3d(0.0, 0.0, 0.0);
  // this->targetAngularVel_prev = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Free surface detection
  this->buoyancyFlag = true; // Initialize buoyancy engine
  this->link = this->model->GetLink(this->model->GetName() + "/" + this->base_link_name);
  // this->boundingBox = link->BoundingBox();
  this->isSubmerged = true;

  // Connect the update event callback
  this->Connect();

  // Initiated
  gzmsg << std::endl;
  gzmsg << "##################################################" << std::endl;
  gzmsg << "#######  KINEMATICS CONTROL PLUGIN  #######" << std::endl;
  gzmsg << "##################################################" << std::endl;
  gzmsg << "Vehicle Model name      : " << this->model->GetName() << std::endl;
  gzmsg << "Vehicle Base_link name  : " << this->base_link_name << std::endl;
  gzmsg << "--------------------------------------------------" << std::endl;
  gzmsg << "Command Topic (frl_msg type)" << std::endl;
  gzmsg << ":\t" + this->model->GetName()
          + "/kinematics/UwGliderCommand" << std::endl;
  gzmsg << "State Topic (frl_msg type)" << std::endl;
  gzmsg << ":\t" + this->model->GetName()
          + "/kinematics/UwGliderStatus" << std::endl;
  gzmsg << "##################################################" << std::endl;
  gzmsg << std::endl;

  // get writeLog Flag
  this->controlMsgDetected = false;
  if (_sdf->HasElement("writeLog"))
  {
    this->writeLogFlag = _sdf->Get<bool>("writeLog");
    gzmsg << "World position of the base_link is saved at"
        << "/tmp/KinematicsLog.csv" << std::endl;
    remove("/tmp/KinematicsLog.csv");
  }
}

/////////////////////////////////////////////////
void KinematicsROSPlugin::Connect()
{
  // Connect the update event
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&KinematicsROSPlugin::Update,
                  this, _1));
}

/////////////////////////////////////////////////
void KinematicsROSPlugin::Init()
{
  // Nothing here
  this->writeCounter = 0;
}

/////////////////////////////////////////////////
void KinematicsROSPlugin::Reset()
{
  // Nothing here
}

/////////////////////////////////////////////////
void KinematicsROSPlugin::Update(const gazebo::common::UpdateInfo &)
{
  // Update time
  this->time = this->model->GetWorld()->SimTime();

  // Update model state
  this->updateModelState();

  // Dynamics Calculation
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
void KinematicsROSPlugin::updateModelState()
{
  gazebo_msgs::GetModelState model_state_msg;
  model_state_msg.request.model_name = this->model->GetName();
  model_state_msg.request.relative_entity_name = "world";
  // this->stateSubscriber["Model"].call(model_state_msg);
  this->modelState.model_name = this->model->GetName();
  // this->modelState.pose = model_state_msg.response.pose;
  this->modelState.pose.position.x = this->model->WorldPose().Pos().X();
  this->modelState.pose.position.y = this->model->WorldPose().Pos().Y();
  this->modelState.pose.position.z = this->model->WorldPose().Pos().Z();
  this->modelState.pose.orientation.x = this->model->WorldPose().Rot().X();
  this->modelState.pose.orientation.y = this->model->WorldPose().Rot().Y();
  this->modelState.pose.orientation.z = this->model->WorldPose().Rot().Z();
  this->modelState.pose.orientation.w = this->model->WorldPose().Rot().W();
}

/////////////////////////////////////////////////
void KinematicsROSPlugin::commandSubThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->commandSubQueue.callAvailable(ros::WallDuration(timeout));
  }
}

/////////////////////////////////////////////////
void KinematicsROSPlugin::ConveyCommands(
  const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
{
  // Convey commands to functions
  if(this->dynamics)
  {
    // Calculate Dynamics
    this->ConveyDynamicsCommands(_msg);
  }
  else
  {
    // Calculate Kinematics
    this->ConveyKinematicsCommands(_msg);

    // Apply ocean current
    this->applyOceanCurrent();
  }

  this->controlMsgDetected = true;
}

/////////////////////////////////////////////////
void KinematicsROSPlugin::ConveyDynamicsCommands(
             const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg)
{
  // Motor/thruster
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

  // Pitch by battery(mass) position
  switch(_msg->pitch_cmd_type)
  {
    case frl_vehicle_msgs::UwGliderCommand::PITCH_CMD_BATT_POS:
    {
      this->massPos = _msg->target_pitch_value/39.3701*1000;  // [m to inch]
      break;
    }

    default:
      gzmsg << "WRONG PITCH COMMAND TYPE "
          << " For dynamics, only (1 : battery position) can be used"
          << std::endl;
      break;
  }

  // Ballast pump
  if (_msg->target_pumped_volume != 0.0)
  {
    gzmsg << "Pump Position control Received" << std::endl;
    this->pumpPos = _msg->target_pumped_volume/1000000.0/(M_PI*(this->r_w)*(this->r_w));
  }

  // Heading control (rudder)
  double target_yaw = 1000.0;
  switch(_msg->rudder_control_mode)
  {
    // TODO : No control algorithm included yet for target heading
    // case frl_vehicle_msgs::UwGliderCommand::RUDDER_CONTROL_HEADING:
    //   target_yaw = M_PI/2 - _msg->target_heading;
    //   break;

    case frl_vehicle_msgs::UwGliderCommand::RUDDER_CONTROL_ANGLE:
    {
      double rudderAngleZero = 0.0;
      double rudderAnglePort = M_PI/6.0; // 30 degree
      double rudderAngleStbd = -M_PI/6.0; // 30 degree
      switch(_msg->rudder_angle)
      {
        case frl_vehicle_msgs::UwGliderCommand::RUDDER_ANGLE_CENTER:
          target_yaw = rudderAngleZero;
          break;

        case frl_vehicle_msgs::UwGliderCommand::RUDDER_ANGLE_PORT:
          target_yaw = rudderAnglePort;
          break;

        case frl_vehicle_msgs::UwGliderCommand::RUDDER_ANGLE_STBD:
          target_yaw = rudderAngleStbd;
          break;

        case frl_vehicle_msgs::UwGliderCommand::RUDDER_ANGLE_DIRECT:
          target_yaw = -_msg->target_rudder_angle; // positive turn stbd
          break;

        default:
          gzmsg << "WRONG RUDDER ANGLE COMMAND "
            << "(1: center, 2: port, 3: staboard, 4: direct)"
            << std::endl; break;
      }
      break;
    }

    case frl_vehicle_msgs::UwGliderCommand::RUDDER_CONTROL_NONE:
      break;

    default:
      gzmsg << "WRONG RUDDER COMMAND TYPE "
          << "For dynamics, only 2: control angle is available"
          << std::endl;
      break;
  }

  // Assign target yaw orientation if msg received
  if (target_yaw != 1000.0)
    this->prev_yaw = target_yaw;
}

/////////////////////////////////////////////////
void KinematicsROSPlugin::CalculateDynamics(
             double _massPos, double _pumpPos, double _thrustPower)
{
  // Hard input data without ros topic for plugin testing
  if (this->hardCodeInput){
    double hardInputPumpVol, hardInputMassPos;
    double hardInputYaw, hardInputThrustPower;
    double _time = this->time.Double();
    double Period = 400.0;

    hardInputPumpVol = 1.0e-04 * cos(_time/Period);
    hardInputMassPos = 1.0e-03 * cos(_time/Period);
    hardInputYaw = 30.0/180.0*M_PI;
    hardInputThrustPower = 0.5;

    // Max value limiter
    if (hardInputMassPos > x_s_o_max)
      hardInputMassPos = x_s_o_max;
    if (hardInputMassPos < -x_s_o_max)
      hardInputMassPos = -x_s_o_max;
    if (hardInputPumpVol > vol_w_max)
      hardInputPumpVol = vol_w_max;
    if (hardInputPumpVol < -vol_w_max)
      hardInputPumpVol = -vol_w_max;

    // Export
    _pumpPos = -hardInputPumpVol/(M_PI*(this->r_w)*(this->r_w));
    _massPos = hardInputMassPos;
    this->prev_yaw = hardInputYaw;
    _thrustPower = hardInputThrustPower;
  }

  // ---- Mass calculation ----- //
  // Ballast volume and mass
  double V_B = -_pumpPos*M_PI*(this->r_w)*(this->r_w);
  double m_w = V_B*this->fluidDensity;

  // Total vehicle mass
  this->m = this->m_h + this->m_s + m_w;

  // Moving mass position
  double x_s = this->x_s_o + _massPos;

  // Hull mass center (uniform hull mass distribution assumed)
  double x_h = -this->m_s/this->m_h*this->x_s_o;

  // Ballast tank mass center (pushing from front)
  double x_w = this->x_w_o+(V_B/(M_PI*(this->r_w)*(this->r_w)));

  // Center of gravity
  (this->cog).X() = (x_h*this->m_h + x_s*this->m_s + x_w*m_w)/(this->m_h+this->m_s+m_w);

  // ---- Rigid Body Matricies ----- //
  double a = this->l_h/2.0; // half the length
  double b = this->r_h;   // hull radius
  // inertial matrix (Fossen p.42 (2.156))
  double I_yy = 4.0/15.0*m*(b*b+a*a); double I_zz = I_yy;double I_xx = 4.0/15.0*m*(b*b+b*b);
  // M_RB = Fossen (2.91)
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
  Eigen::Vector6d ab = M_RB * nu_last;
  Eigen::Matrix3d Sa = -1.0 * CrossProductOperator(ab.head<3>());
  C_RB << Eigen::Matrix3d::Zero(), Sa,
         Sa, -1.0 * CrossProductOperator(ab.tail<3>());

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

  // Added mass Coriolis diagonal term
  Eigen::Matrix6d C_A = Eigen::Matrix6d::Zero();
  ab = M_A * nu_last;
  Sa = -1.0 * CrossProductOperator(ab.head<3>());
  C_A << Eigen::Matrix3d::Zero(), Sa,
         Sa, -1.0 * CrossProductOperator(ab.tail<3>());

  // ---- Hydrostatic restoring forces ----- //
  double W = this->m*this->gravity; // Gravitational force acting on center of gravity
  double B = (this->m_h+this->m_s)*this->gravity; // Buoyancy force acting on center of buoyancy
  Eigen::Vector6d tau_R;
  tau_R << (W-B)*sin(eta(4)), -(W-B)*cos(eta(4))*sin(eta(3)), -(W-B)*cos(eta(4))*cos(eta(3)),
           -((this->cog).Y()*W-(this->cob).Y()*B)*cos(eta(4))*cos(eta(3))+((this->cog).Z()*W-(this->cob).Z()*B)*cos(eta(4))*sin(eta(3)),
           ((this->cog).Z()*W-(this->cob).Z()*B)*sin(eta(4))+((this->cog).X()*W-(this->cob).X()*B)*cos(eta(4))*cos(eta(3)),
          -((this->cog).X()*W-(this->cob).X()*B)*cos(eta(4))*sin(eta(3))-((this->cog).Y()*W-(this->cob).Y()*B)*sin(eta(4));

  // --- Calculate hull hydrodynamic force (Graver) --- //
  //angle of attack of hull
  double alpha_h;
  if (abs(nu_last(0)) > 0.0)
      alpha_h = atan(nu_last(2)/nu_last(0));
  else
      alpha_h = 0.0;
  // Graver hydrodynamic forces for hull
  double A_h = M_PI*this->r_h*this->r_h;
  double C_D_h = 0.214 + 32.3*alpha_h*alpha_h;
  double C_L_h = 11.76*alpha_h + 4.6 * alpha_h*alpha_h;
  double C_M_h = 0.63*alpha_h;
  double F_h_kernel = std::pow(std::pow(nu_last(0)*nu_last(0)+nu_last(1)*nu_last(1)+nu_last(2)*nu_last(2),0.5),2);
  double F_D_h = 0.5*this->fluidDensity*A_h*F_h_kernel*C_D_h;
  double F_L_h = 0.5*this->fluidDensity*A_h*F_h_kernel*C_L_h;
  double F_M_h = 0.5*this->fluidDensity*A_h*F_h_kernel*C_M_h;
  Eigen::Vector6d tau_H_h; tau_H_h << F_L_h*sin(alpha_h)-F_D_h*cos(alpha_h), 0.0, -F_L_h*cos(alpha_h)-F_D_h*sin(alpha_h), 0.0, F_M_h, 0.0;

  // --- Calculate rudder hydrodynamic force (Graver) --- //
  //angle of attack of rudder
  double alpha_r;
  if (abs(nu_last(0)) > 0.0)
      alpha_r = this->prev_yaw;
  else
      alpha_r = 0.0;
  // Graver hydrodynamic forces for rudder
  // TODO : Demo hydordynamic forces coefficient used here using the template of the hull
  double A_r = M_PI*this->r_h*this->r_h / 10.0;
  double C_D_r = (0.214 + 32.3*alpha_r*alpha_r) / 10.0;
  double C_L_r = (11.76*alpha_r + 4.6 * alpha_r*alpha_r) / 10.0;
  double C_M_r = 0.63*alpha_r * 10.0;
  double F_r_kernel = std::pow(std::pow(nu_last(0)*nu_last(0)+nu_last(1)*nu_last(1)+nu_last(2)*nu_last(2),0.5),2);
  double F_D_r = 0.5*this->fluidDensity*A_r*F_r_kernel*C_D_r;
  double F_L_r = 0.5*this->fluidDensity*A_r*F_r_kernel*C_L_r;
  double F_M_r = 0.5*this->fluidDensity*A_r*F_r_kernel*C_M_r;
  Eigen::Vector6d tau_H_r; tau_H_r << F_L_r*sin(alpha_r)-F_D_r*cos(alpha_r),
            -F_L_r*cos(alpha_r)-F_D_r*sin(alpha_r), 0.0, 0.0, 0.0, F_M_r;

  // Hydrodynamic forces by the propeller
  Eigen::Vector6d tau_H_p; tau_H_p << _thrustPower, 0.0, 0.0, 0.0, 0.0, 0.0;

  // --- Solve --- //
  // totla matricies and forces
  Eigen::Matrix6d M_tot = M_RB + M_A;
  Eigen::Matrix6d C_tot = C_RB + C_A;
  Eigen::Matrix6d D_tot = -1.0 * this->DLin;
  Eigen::Vector6d tau_H = tau_H_h + tau_H_r + tau_H_p;

  // Solve for nu_dot
  this->nu_dot = (M_tot).colPivHouseholderQr().solve(tau_H-(C_tot)*nu_last-(D_tot)*nu_last-tau_R);

  // --- Update vehicle position --- //
  // Caculate position
  double dt = this->time.Double() - this->lastTime;
  this->nu = this->nu_last + dt*this->nu_dot;
  this->eta_dot = Jacobian(this->eta_last(3),this->eta_last(4),this->eta_last(5))*this->nu;
  this->eta = this->eta_last + dt*this->eta_dot;

  // Apply ocean current
  double dx = this->flowVelocity.Y() * dt;
  double dy = this->flowVelocity.X() * dt;
  double dz = -this->flowVelocity.Z() * dt;
  this->eta(0) += dx; this->eta(1) += dy; this->eta(2) += dz;

  // Update pose
  ignition::math::Pose3d model_pose;
  model_pose.Pos().X() = this->eta(0);
  model_pose.Pos().Y() = this->eta(1);
  model_pose.Pos().Z() = this->eta(2);
  ignition::math::Quaterniond target_orientation;
  target_orientation.Euler(this->eta(3), this->eta(4), this->eta(5));
  model_pose.Rot() = target_orientation;
  this->model->SetWorldPose(model_pose);

  // update time and variables
  this->lastTime = this->time.Double();
  this->nu_last = this->nu;
  this->eta_last = this->eta;

  // Eliminate Gazebo physics
  this->link->ResetPhysicsStates();
}


/////////////////////////////////////////////////
void KinematicsROSPlugin::ConveyKinematicsCommands(
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
      // Commented out : Use dynamics flag for battery position command
      gzmsg << "USE DYNAMICS for battery position command " << std::endl;
      // this->battpos = _msg->target_pitch_value/39.3701*1000;  // [m to inch]
      // double target_pitch_rad
      //           = this->f_pitch_battpos_cal_m * this->battpos + this->f_pitch_battpos_cal_b;
      // target_pitch = target_pitch_rad;
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
  double buoyancy_vel_x, buoyancy_vel_z;
  if (this->prev_pumpVol != 0.0)
  {
    buoyancy_vel_x = vx*cos(target_pitch)+vz*sin(target_pitch);
    buoyancy_vel_z = vx*sin(target_pitch)+vz*cos(target_pitch);
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
  // this->commandPublisher["Model"].call(cmd_msg);
  ignition::math::Pose3d targetPose;
  targetPose.Pos() = this->model->WorldPose().Pos();
  targetPose.Rot().X() = cmd_msg.request.model_state.pose.orientation.x;
  targetPose.Rot().Y() = cmd_msg.request.model_state.pose.orientation.y;
  targetPose.Rot().Z() = cmd_msg.request.model_state.pose.orientation.z;
  targetPose.Rot().W() = cmd_msg.request.model_state.pose.orientation.w;
  this->model->SetWorldPose(targetPose);

  ignition::math::Vector3d targetLinearTwist;
  targetLinearTwist.X() = cmd_msg.request.model_state.twist.linear.x;
  targetLinearTwist.Y() = cmd_msg.request.model_state.twist.linear.y;
  targetLinearTwist.Z() = cmd_msg.request.model_state.twist.linear.z;
  ignition::math::Vector3d targetAngularTwist(0.0, 0.0, 0.0);
  this->model->SetWorldTwist(targetLinearTwist, targetAngularTwist);

  this->modelState = cmd_msg.request.model_state;

  // Save model velocity right after commend published
  this->modelVel = ignition::math::Vector3d(
                    this->link->WorldLinearVel().X(),
                    this->link->WorldLinearVel().Y(),
                    this->link->WorldLinearVel().Z());

  // Save data for log
  if (this->writeLogFlag)
  {
    this->thrusterVel = v_thrust;
    this->buoyancyVel = ignition::math::Vector2d(buoyancy_vel_x, buoyancy_vel_z);
    this->vehicleVel = this->modelVel;
  }

}

/////////////////////////////////////////////////
void KinematicsROSPlugin::applyOceanCurrent()
{
  this->link->SetLinearVel(ignition::math::Vector3d(
            this->modelVel.X() + this->flowVelocity.Y(),
            this->modelVel.Y() + this->flowVelocity.X(),
            this->modelVel.Z() - this->flowVelocity.Z()));
}

/////////////////////////////////////////////////
void KinematicsROSPlugin::ConveyModelState()
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

  // Calculate Lat/Lon using GDAl tranform
  OGRSpatialReference srs;
  srs.importFromEPSG(this->espg_projection_);
  OGRSpatialReference tsrs;
  tsrs.importFromEPSG(4326);
  OGRCoordinateTransformation *poCT;
  poCT = OGRCreateCoordinateTransformation(&srs, &tsrs);
  double sNorthing = this->modelXYZ.Y();
  double sEasting = this->modelXYZ.X();
  double tLat = sNorthing;
  double tLon = sEasting;
  poCT->Transform(1, &tLon, &tLat);

  // Construct UwGliderStatus msg
  frl_vehicle_msgs::UwGliderStatus status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.longitude = tLon;
  status_msg.latitude = tLat;
  status_msg.roll = this->modelRPY.Z();
  status_msg.pitch = this->modelRPY.Y();
  status_msg.heading = M_PI/2 - this->modelRPY.X();
  // if (abs(status_msg.heading) < M_PI)
  // {
  //   if status_msg.heading < 0
  //   {
  //     status_msg.heading += 2 * M_PI;
  //   }
  //   else
  //   {
  //     status_msg.heading -= 2 * M_PI;
  //   }
  // }
  status_msg.depth = - this->modelXYZ.Z();
  status_msg.altitude = this->modelXYZ.Z();
  status_msg.motor_power = this->motorPower;
  status_msg.rudder_angle = this->rudderAngle;
  status_msg.pumped_volume = this->prev_pumpVol;
  status_msg.battery_position = this->battpos;

  this->statePublisher.publish(status_msg);

  // Save data for log
  if (this->writeLogFlag)
  {
    this->lat = status_msg.latitude;
    this->lon = status_msg.longitude;
  }
}

/////////////////////////////////////////////////
void KinematicsROSPlugin::writeCSVLog()
{
// CSV log write stream
if (this->writeLogFlag)
{
  double time = this->time.Double();
  if (this->writeCounter == 0)
  {
    writeLog.open("/tmp/KinematicsLog.csv");
    writeLog << "# Hybrid Glider Plugin Log\n";
    writeLog << "# t,x,y,altitude,roll,pitch,heading,lat,lon,thrustPower,pumpVol"
             << ",batPos,thrustVel,xBuoyancyVel,zBuoyancyVel"
             << ",xVehicleVel,yVehicleVel,zVehicleVel,xOceanCurrent,yOceanCurrent,zOceanCurrent" << "\n";
    writeLog.close();
    this->writeCounter = this->writeCounter + 1;
  }
  if (floor(time * 10) == time * 10)
  {
    int prec = std::numeric_limits<double>::digits10+2; // generally 17
    writeLog.open("/tmp/KinematicsLog.csv", std::ios_base::app);
    writeLog << std::setprecision(prec) << time << ","
            << this->modelXYZ.X() << "," << this->modelXYZ.Y() << ","
            << this->modelXYZ.Z() << "," << this->modelRPY.Z() << ","
            << this->modelRPY.Y() << "," << M_PI/2 - this->modelRPY.X() << ","
            << this->lat << "," << this->lon << "," << this->motorPower
            << "," << this->prev_pumpVol << "," << this->battpos << ","
            << this->thrusterVel << "," << this->buoyancyVel.X() << ","
            << this->buoyancyVel.Y()  << "," << this->vehicleVel.X()
            << "," << this->vehicleVel.Y() << "," << this->vehicleVel.Z()
            << "," << this->flowVelocity.X() << "," << this->flowVelocity.Y()
            << "," << this->flowVelocity.Z() << "\n";
    writeLog.close();
  }
}
}

/////////////////////////////////////////////////
ignition::math::Vector3<double> KinematicsROSPlugin::calculateRPY
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
void KinematicsROSPlugin::PropRotate(double thrustPower)
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
void KinematicsROSPlugin::UpdateFlowVelocity
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
ignition::math::Vector3d KinematicsROSPlugin::ToNED(ignition::math::Vector3d _vec)
{
  ignition::math::Vector3d output = _vec;
  output.Y() = -1.0 * output.Y();
  output.Z() = -1.0 * output.Z();
  return output;
}

/////////////////////////////////////////////////
ignition::math::Vector3d KinematicsROSPlugin::FromNED(ignition::math::Vector3d _vec)
{
  return this->ToNED(_vec);
}

/////////////////////////////////////////////////
void KinematicsROSPlugin::calcThrusterForce(int cmd_type, double cmd_value)
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
void KinematicsROSPlugin::CheckSubmergence()
{
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  gazebo::rendering::VisualPtr visual = scene->GetVisual(this->link->GetModel()->GetName());

  double height = visual->BoundingBox().ZLength();
  double z = this->link->WorldPose().Pos().Z();
  bool previousState = this->isSubmerged;

  if (previousState)
    this->lastPose = this->link->WorldPose();

  if (!this->surfaceVehicle) // For submerged vessel
  {
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
  else  // For surface vehicle
  {
    // Keep the vehicle on surface
    double surfaceZ = + height/10;
    this->lastPose.Pos().Z() = surfaceZ;

    // set pitch and roll to zero
    ignition::math::Quaterniond quat;
    quat.Euler(0.0, 0.0, prev_yaw);
    this->lastPose.Rot() = quat;
    this->link->SetWorldPose(this->lastPose);
  }
}

GZ_REGISTER_MODEL_PLUGIN(KinematicsROSPlugin)
}
