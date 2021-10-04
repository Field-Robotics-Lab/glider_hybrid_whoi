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

/// \file Kinematics ROS plugin for a ROS node

#ifndef __KINEMATICS_ROS_PLUGIN_HH__
#define __KINEMATICS_ROS_PLUGIN_HH__

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/SetLinkState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

// Functionality from GDAL for projections
#include <gdal/ogr_spatialref.h>

#include <std_msgs/Bool.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <frl_vehicle_msgs/UwGliderCommand.h>
#include <frl_vehicle_msgs/UwGliderStatus.h>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <map>
#include <string>
#include <vector>
#include <thread>

namespace Eigen
{
  /// \brief Definition of a 6x6 Eigen matrix
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  /// \brief Definition of a 6 element Eigen vector
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

namespace kinematics_ros
{
  class KinematicsROSPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: KinematicsROSPlugin();

    /// \brief Destructor
    public: virtual ~KinematicsROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::ModelPtr _model,
                      sdf::ElementPtr _sdf);

    /// \brief Initialize Module.
    public: virtual void Init();

    /// \brief Reset Module.
    public: virtual void Reset();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: virtual void Update(const gazebo::common::UpdateInfo &);

    /// \brief Connects the update event callback
    protected: virtual void Connect();

    /// \brief Update event
    protected: gazebo::event::ConnectionPtr updateConnection;

    /// \brief Convey model state from gazebo topic to outside
    protected: virtual void ConveyModelState();

    /// \brief Convey model state from gazebo topic to outside
    protected: virtual void ConveyCommands(const frl_vehicle_msgs::
                                UwGliderCommand::ConstPtr &_msg);

    /// \brief Convey model state from gazebo topic to outside (model)
    protected: virtual void ConveyKinematicsCommands(const frl_vehicle_msgs::
                                    UwGliderCommand::ConstPtr &_msg);

    /// \brief Pointer to the model structure
    protected: gazebo::physics::ModelPtr model;

    /// \brief Pointer to this ROS node's handle.
    private: boost::scoped_ptr<ros::NodeHandle> rosNode;

    /// \brief ROS Subscribers from outside
    private: ros::Subscriber commandSubscriber;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue commandSubQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread commandSubQueueThread;

    /// \brief ROS helper function that processes messages
    private: void commandSubThread();

    /// \brief Flag for surface vehicle
    protected: bool surfaceVehicle;

    /// \brief update ocean current
    protected: virtual void applyOceanCurrent();
    protected: bool stratifiedOceanCurrent;
    protected: ignition::math::Vector3<double> modelVel;

    /// \brief update model state
    protected: virtual void updateModelState();

    /// \brief update model state
    protected: virtual ignition::math::Vector3<double> calculateRPY(
                              double x, double y, double z, double w);

    /// \brief ROS Publishers to outside
    private: ros::Publisher statePublisher;

    /// \brief Time at gazebo simulation
    protected: gazebo::common::Time time;

    /// \brief Model State
    protected: gazebo_msgs::ModelState modelState;

    /// \brief Model Pose
    protected: ignition::math::Vector3<double> modelXYZ;
    protected: ignition::math::Vector3<double> modelRPY;

    /// \brief Model Lat/Lon
    protected: double lat;
    protected: double lon;

    /// \brief Motor power
    protected: double motorPower;

    /// \brief Velocity by thruster
    protected: double thrusterVel;

    /// \brief Velocity by buoyancy
    protected: ignition::math::Vector2<double> buoyancyVel;

    /// \brief Velocity by thruster
    protected: ignition::math::Vector3<double> vehicleVel;

    /// \brief Rudder angle
    protected: double rudderAngle;

    /// \brief Fluid density
    protected: double fluidDensity;

    /// \brief Gravity
    protected: double gravity;

    /// \brief buoyancy Command Flag
    protected: bool buoyancyFlag;

    /// \brief Flag to use the global current velocity or the individually
    /// assigned current velocity
    protected: bool useGlobalCurrent;

    /// \brief Flow velocity vector read from topic
    protected: ignition::math::Vector3d flowVelocity;

    /// \brief Reads flow velocity topic
    protected: void UpdateFlowVelocity
      (const geometry_msgs::TwistStamped::ConstPtr &_msg);

    /// \brief Subcriber to flow message
    private: ros::Subscriber flowSubscriber;

    /// \brief Battery pitch control kinematics coefficents
    protected: double f_pitch_battpos_cal_m;
    protected: double f_pitch_battpos_cal_b;
    protected: double battpos;

    /// \brief Thruster power coefficents
    protected: double f_thruster_voltage_v1;
    protected: double f_thruster_voltage_v2;
    protected: double f_thruster_voltage_v3;
    protected: double f_thruster_power_w1;
    protected: double f_thruster_power_w2;
    protected: double f_thruster_power_w3;

    /// \brief Previous values
    protected: double prev_pitch;
    protected: double prev_yaw;
    protected: double prev_motorPower;
    protected: double prev_pumpVol;

    /// \brief Coefficients
    protected: double Area;
    protected: double C_D;
    protected: double C_L;

    /// ============================================= ///
    /// ===========   DYNAMICS (for pitch)   ======== ///
    /// ============================================= ///
    /// \brief A flag for dynamics calculations
    private: bool dynamics;

    /// \brief A flag for hardcoded input
    private: bool hardCodeInput;

    /// \brief Buoyancy pump position
    protected: double pumpPos;

    /// \brief Sliding mass position
    protected: double massPos;

    /// \brief total_mass
    protected: double m;

    /// \brief Center of gravity
    protected: ignition::math::Vector3d cog;

    /// \brief Center of buoyancy in the body frame
    protected: ignition::math::Vector3d cob;

    /// \brief ballast_radius
    protected: double r_w;

    /// \brief hull_length
    protected: double l_h;

    /// \brief hull_radius
    protected: double r_h;

    /// \brief hull_mass
    protected: double m_h;

    /// \brief shifter_mass
    protected: double m_s;

    /// \brief initial_mass_position
    protected: double x_s_o;

    /// \brief initial_ballast_position
    protected: double x_w_o;

    /// \brief max_mass_position
    protected: double x_s_o_max;

    /// \brief max_ballast_volume
    protected: double vol_w_max;

    /// \brief Added-mass matrix
    protected: Eigen::Matrix6d Ma;

    /// \brief Added-mass associated Coriolis matrix
    protected: Eigen::Matrix6d Ca;

    /// \brief Damping matrix
    protected: Eigen::Matrix6d D;

    /// \brief Linear damping matrix
    protected: Eigen::Matrix6d DLin;

    /// \brief Last timestamp (in seconds)
    protected: double lastTime;

    /// \brief Dynamics post/vel variables
    protected: Eigen::Vector6d eta, eta_last, eta_dot, eta_dot_last;
    protected: Eigen::Vector6d nu, nu_last, nu_dot, nu_dot_last;
    protected: Eigen::Vector6d lastVelRel;
    // protected: Eigen::Vector3d targetLinearVel_prev;
    // protected: Eigen::Vector3d targetAngularVel_prev;

    /// \brief Get parameters (read matrix form defenitions)
    protected: void GetParam(std::string _tag, std::vector<double>& _output);

    /// \brief Filtered linear & angular acceleration vector in link frame.
    /// This is used to prevent the model to become unstable given that Gazebo
    /// only calls the update function at the beginning or at the end of a
    /// iteration of the physics engine
    protected: Eigen::Vector6d filteredAcc;

    /// \brief Computes the added-mass Coriolis matrix Ca.
    protected: void ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel,
                                             Eigen::Matrix6d &_Ma,
                                             Eigen::Matrix6d &_Ca) const;

    /// \brief Updates the damping matrix for the current velocity
    protected: void ComputeDampingMatrix(const Eigen::Vector6d& _vel,
                                       Eigen::Matrix6d &_D) const;

    /// \brief Filter acceleration (fix due to the update structure of Gazebo)
    protected: void ComputeAcc(Eigen::Vector6d _velRel,
                            double _time,
                            double _alpha = 0.3);

    /// \brief Convey commands to calculate dynamics
    protected: virtual void ConveyDynamicsCommands
        (const frl_vehicle_msgs::UwGliderCommand::ConstPtr &_msg);

    /// \brief Calculate dynamics when commands conveyed and every updates
    protected: virtual void CalculateDynamics
        (double _massPos, double _pumpVol, double _thrustPower);

    /// ============================================= ///
    /// ===========-=== Visual Effects ======-======= ///
    /// ============================================= ///

    /// \brief Base link name
    protected: std::string base_link_name;

    /// \brief Publishers for propeller roatation visual effects
    protected: std::map<std::string, ros::Publisher> propVisualPubs;

    /// \brief A flag for dual propellers
    protected: bool dualProps;

    /// \brief Function to rotate the propellers
    protected: virtual void PropRotate(double thrustPower);

    /// ============================================= ///
    /// ============== Other functions ============== ///
    /// ============================================= ///

    /// \brief espg projection coordinate for GDAL transform
    protected: int espg_projection_;

    /// \brief Convert vector to comply with the NED reference frame
    protected: ignition::math::Vector3d ToNED(ignition::math::Vector3d _vec);

    /// \brief Convert vector to comply with the NED reference frame
    protected: ignition::math::Vector3d FromNED(ignition::math::Vector3d _vec);

    /// \brief Function to calculate thruster characteristics
    protected: virtual void calcThrusterForce(int cmd_type, double cmd_value);

    /// \brief CSV log writing stream for verifications
    protected: std::ofstream writeLog;
    protected: u_int64_t writeCounter;
    protected: bool writeLogFlag;
    protected: virtual void writeCSVLog();
    protected: bool controlMsgDetected;

    private: std::map<std::string, geometry_msgs::TransformStamped> nedTransform;
    private: std::map<std::string, tf2_ros::TransformBroadcaster> tfBroadcaster;

    /// --------- Free surface detection ----------///
    /// \brief Pointer to the base_link
    protected: gazebo::physics::LinkPtr link;

    /// \brief Is submerged flag
    protected: bool isSubmerged;

    /// \brief Is submerged flag
    protected: ignition::math::Pose3d lastPose;

    /// \brief Is submerged flag
    protected: ignition::math::Vector3d buoyancyForce;

    /// \brief Calculate buoyancy force with respect to free surface
    protected: virtual void CheckSubmergence();

  };

/// \brief Conversion of a string to a double vector
inline ignition::math::Vector3<double> Str2Vector(std::string _input)
{
  ignition::math::Vector3<double> results;
  int num = 0;
  std::string buf;
  std::stringstream ss(_input);
  while (ss >> buf)
  {
    results[num] = std::stod(buf);
    num = num+1;
  }
  return results;
}

/// \brief Conversion of a string to a double vector
inline Eigen::Matrix6d Str2Matrix6(std::string _input)
{
  Eigen::Matrix6d results;
  int num = 0;
  std::string buf;
  std::stringstream ss(_input);
  while (ss >> buf)
  {
    results(num) = std::stod(buf);
    num = num+1;
    // results << std::stod(buf);m(0,0)
  }
  return results;
}

/// \brief Returns the cross product operator matrix
/// for Eigen vectors
inline Eigen::Matrix3d CrossProductOperator(Eigen::Vector3d _x)
{
  Eigen::Matrix3d output;
  output << 0.0, -_x[2], _x[1], _x[2], 0.0, -_x[0], -_x[1], _x[0], 0.0;
  return output;
}

/// \brief Returns the cross product operator matrix
/// for Gazebo vectors
inline Eigen::Matrix3d CrossProductOperator(ignition::math::Vector3d _x)
{
  Eigen::Matrix3d output;
  output << 0.0, -_x[2], _x[1], _x[2], 0.0, -_x[0], -_x[1], _x[0], 0.0;
  return output;
}

inline Eigen::Vector3d ToEigen(const ignition::math::Vector3d &_x)
{
  return Eigen::Vector3d(_x[0], _x[1], _x[2]);
}

inline Eigen::Matrix3d ToEigen(const ignition::math::Matrix3d &_x)
{
  Eigen::Matrix3d m;
  m << _x(0, 0), _x(0, 1), _x(0, 2),
       _x(1, 0), _x(1, 1), _x(1, 2),
       _x(2, 0), _x(2, 1), _x(2, 2);
  return m;
}

inline Eigen::Vector6d EigenStack(const ignition::math::Vector3d &_x,
                                  const ignition::math::Vector3d &_y)
{
    Eigen::Vector3d xe = ToEigen(_x);
    Eigen::Vector3d ye = ToEigen(_y);
    Eigen::Vector6d out;
    out << xe, ye;
    return out;
}

inline ignition::math::Vector3d Vec3dToGazebo(const Eigen::Vector3d &_x)
{
  return ignition::math::Vector3d(_x[0], _x[1], _x[2]);
}

inline ignition::math::Matrix3d Mat3dToGazebo(const Eigen::Matrix3d &_x)
{
  return ignition::math::Matrix3d(_x(0, 0), _x(0, 1), _x(0, 2),
     _x(1, 0), _x(1, 1), _x(1, 2),
     _x(2, 0), _x(2, 1), _x(2, 2));
}

inline Eigen::Matrix6d Jacobian(const double &phi,
                                const double &theta,
                                const double &psi)
{
    Eigen::Matrix6d out;
    out << cos(psi)*cos(theta),
    -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi),
    sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta), 0, 0, 0,
    sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi),
    -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi), 0, 0, 0,
    -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi), 0, 0, 0,
    0, 0, 0, 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
    0, 0, 0, 0, cos(phi), -sin(phi),
    0, 0, 0, 0, sin(phi)/cos(theta), cos(phi)/cos(theta);
    return out;
}
}

#endif  // __KINEMATICS_ROS_PLUGIN_HH__
