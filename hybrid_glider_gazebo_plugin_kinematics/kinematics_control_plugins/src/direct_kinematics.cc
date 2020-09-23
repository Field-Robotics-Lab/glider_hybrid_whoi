#ifndef _DIRECT_KINEMATICS_HH_
#define _DIRECT_KINEMATICS_HH_

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
    /// \brief Set the velocity of the vejoc;e
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
      this->SetVelocity(ignition::math::Vector3d(_msg->x(),_msg->y(),_msg->z()));
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(DirectKinematics)
} // namespace gazebo
#endif