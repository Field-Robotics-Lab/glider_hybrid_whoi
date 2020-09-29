#ifndef _DIRECT_KINEMATICS_HH_
#define _DIRECT_KINEMATICS_HH_
#define M_PI 3.14159265358979323846

// defaults for the plugin
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// for msg receiving for the plugin
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
// for ros connection
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "gazebo_msgs/SetModelState.h"

#include <cmath>
#include <iostream>

namespace gazebo
{
  /// \brief A plugin to control the vehicle kinematics directly
  class DirectKinematics : public ModelPlugin
  {
    /// \brief Constructor
  public:
    DirectKinematics() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
  public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Plugin load notifier
      std::cerr << "\nThe Direct Kinematics plugin is attach to model[" << _model->GetName() << "]\n";

      // Store the model pointer for convenience.
      this->model = _model;
      
      // Default to zero velocity
      ignition::math::Vector3d velocity = ignition::math::Vector3d(0, 0, 0);

      // Check that the velocity element exists, then read the value
      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

      this->SetVelocity(velocity);
      
      //----------- FOR MSG CONNECTION ---------- //
      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->model->GetWorld()->Name());

      // If topic is available, subscribe to it
      if (_sdf->HasElement("direct_kinematics_topic"))
      {
        std::string topicName = _sdf->Get<std::string>("direct_kinematics_topic");
        GZ_ASSERT(!topicName.empty(),
                  "specity direct kinematics topic tag");

        gzmsg << "Subscribing to direct kinematics topic: " << topicName
            << std::endl;
        this->sub = this->node->Subscribe(topicName,
          &DirectKinematics::OnMsg, this);
      }
    
    }
    /// \brief Set the velocity of the vehicle
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const ignition::math::Vector3d &_vel)
    {
      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(_vel);
    }

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    //----------- FOR MSG CONNECTION ---------- //
    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      // Apply velocity to the model.

      // gzmsg << std::endl << _msg->x() << std::endl;
      // gzmsg << _msg->y() << std::endl;
      // gzmsg << _msg->z() << std::endl << std::endl;
      ignition::math::Vector3d _velocity = ignition::math::Vector3d(_msg->x(),_msg->y(),_msg->z());
      // _msg->x() * cos(_msg->y()) * cos(_msg->z()),
      // _msg->x() * sin(_msg->y()) * cos(_msg->z()),
      // _msg->x() * sin(_msg->z()));
      this->SetVelocity(_velocity);

      // Apply orientation to the model.
      ignition::math::Pose3d pose = this->model->WorldPose();
      ignition::math::Vector3<double> vP = pose.Pos();
      double phi = std::atan(_msg->y()/_msg->x());
      double theta = std::atan(std::sqrt(_msg->x()*_msg->x()+_msg->y()*_msg->y())/_msg->z()) + M_PI/2.0 ;
      gzmsg << std::endl << theta << std::endl;
      gzmsg << phi << std::endl << std::endl;
      if( std::isnan(theta) )
      { 
        theta = 0;
      }
      if( std::isnan(phi) )
      { 
        phi = 0;
      }
      pose.Set(vP.X(),vP.Y(),vP.Z(),0,theta,phi);
      this->model->SetWorldPose(pose);
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(DirectKinematics)
} // namespace gazebo
#endif