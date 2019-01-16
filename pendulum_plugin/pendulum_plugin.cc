#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  class PendulumPlugin : public ModelPlugin
  {
  public: 

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PendulumPlugin::OnUpdate, this));

      // Initialize ros, if it has not already been initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/force_command",
            1,
            boost::bind(&PendulumPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&PendulumPlugin::QueueThread, this));

    }

    // Called by the world update start event
    void OnUpdate()
    { // hardcoded before ROS integration:
 
      // Apply a small linear velocity to the model.
      // this->model->SetLinearVel(ignition::math::Vector3d(0, .3, 0));

      // apply a force to the chasis simulating motor actuation
      // float yAxisForce = 0.5;
      // model->GetLink("link_0")->SetForce(ignition::math::Vector3d(0,yAxisForce,0));
    }

    // handle incoming ROS messages
    void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      float yAxisForce = _msg->data;
      // this->SetVelocity(_msg->data);
      model->GetLink("link_0")->SetForce(ignition::math::Vector3d(0,yAxisForce,0));
    }

  private:

    // ROS helper function to process messages 
    void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    // private members for ROS
    physics::ModelPtr model; // Pointer to the model
    event::ConnectionPtr updateConnection; // Pointer to the update event connection
    std::unique_ptr<ros::NodeHandle> rosNode; // a node used for ROS transport
    ros::Subscriber rosSub; // a ROS subscriber
    ros::CallbackQueue rosQueue; // A ROS callbackqueue that helps process messages
    std::thread rosQueueThread; // A thread the keeps running the rosQueue
    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PendulumPlugin)
}
