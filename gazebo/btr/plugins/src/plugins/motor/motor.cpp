/***************************************************************************
 *  motor.cpp - Plugin for controling a model through a simulated motor
 *
 *  Created: Wed Jan 29 16:10:17 2014
 *  Copyright  2014  Frederik Zwilling
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "motor.h"

#include <utils/misc/gazebo_api_wrappers.h>

#include <math.h>

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Motor)

Motor::Motor()
{
}

Motor::~Motor()
{
	printf("Destructing Motor Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */

void
Motor::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
	// Store the pointer to the model
	this->model_ = _parent;

	//get the model-name
	this->name_ = model_->GetName();
	printf("Loading Motor Plugin of model %s\n", name_.c_str());

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->update_connection_ =
	  event::Events::ConnectWorldUpdateBegin(boost::bind(&Motor::OnUpdate, this, _1));

	//Create the communication Node for communication with fawkes
	this->node_ = transport::NodePtr(new transport::Node());
	//the namespace is set to the model name!
	this->node_->Init(model_->GetWorld()->GZWRAP_NAME() + "/" + name_);

	//initialize movement commands:
	vx_     = 0.0;
	vy_     = 0.0;
	vomega_ = 0.0;

	//create subscriber
	this->motor_move_sub_ = this->node_->Subscribe(std::string("~/RobotinoSim/MotorMove/"),
	                                               &Motor::on_motor_move_msg,
	                                               this);

	// subscriber for ROS
	// Initialize ros, if it has not already bee initialized.
	std::string clientName= "gazsim_mtr_" + this->model_->GetName();
	if (!ros::isInitialized())
	{
	  int argc = 0;
	  char **argv = NULL;
	  ros::init(argc, argv, clientName.c_str(),
	      ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to
	// the Gazebo node
	this->rosNode.reset(new ros::NodeHandle(clientName.c_str()));

	// Create a named topic, and subscribe to it.
	ros::SubscribeOptions so =
	  ros::SubscribeOptions::create<geometry_msgs::Twist>(
	      "/" + this->model_->GetName() + "/cmd_vel",
	      1,
	      boost::bind(&Motor::OnRosMsg, this, _1),
	      ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so);

	// Spin up the queue helper thread.
	// this->rosQueueThread =
	//  std::thread(std::bind(&Motor::QueueThread, this));
	this->rosQueueThread = 
		boost::thread(boost::bind(&Motor::QueueThread, this));

	printf("Subscribe %s%s%s\n", "/", this->model_->GetName().c_str(), "/cmd_vel");
}

/** Called by the world update start event
 */
void
Motor::OnUpdate(const common::UpdateInfo & /*_info*/)
{
	//Apply movement command
	float x, y;
	float yaw = this->model_->GZWRAP_WORLD_POSE().GZWRAP_ROT_EULER_Z;
	//foward part
	x = cos(yaw) * vx_;
	y = sin(yaw) * vx_;
	//sideways part
	x += cos(yaw + 3.1415926f / 2) * vy_;
	y += sin(yaw + 3.1415926f / 2) * vy_;
	// Apply velocity to the model.
	this->model_->SetLinearVel(gzwrap::Vector3d(x, y, 0));
	this->model_->SetAngularVel(gzwrap::Vector3d(0, 0, vomega_));
}

/** on Gazebo reset
 */
void
Motor::Reset()
{
	//stop movement
	vx_     = 0;
	vy_     = 0;
	vomega_ = 0;
}

/** Functions for recieving Messages (registerd via suscribers)
 * @param msg message
 */
void
Motor::on_motor_move_msg(ConstVector3dPtr &msg)
{
	//printf("Got MotorMove Msg!!! %f %f %f\n", msg->x(), msg->y(), msg->z());
	//Transform relative motion into ablosulte motion
	vx_     = msg->x();
	vy_     = msg->y();
	vomega_ = msg->z();
}

// \brief Handle an incoming message from ROS
// \param[in] _msg A float value that is used to set the velocity
// of the Velodyne.
// void
// Motor::OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
void
Motor::OnRosMsg(const geometry_msgs::TwistConstPtr &_msg)
{
        // this->SetVelocity(_msg->data);
	// float yaw = this->model_->GZWRAP_WORLD_POSE().GZWRAP_ROT_EULER_Z;
        vx_     = _msg->linear.x;
        vy_     = _msg->linear.y;
        vomega_ = _msg->angular.z;
	// printf("Got MotorMove Msg!!! %f %f %f\n", vx_, vy_, vomega_);
}

/// \brief ROS helper function that processes messages
void
Motor::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
    // printf("%d\n", this->rosNode->ok());
  }
}

