/***************************************************************************
 *  odometry.cpp - provides odometry simulation of object model
 *
 *  Created: Fri Feb 20 18:05:07 2015
 *  Copyright  2015 Stefan Profanter
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

#include "odometry.h"

#include <utils/misc/gazebo_api_wrappers.h>

#include <math.h>

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Odometry)

///Constructor
Odometry::Odometry()
{
}
///Destructor
Odometry::~Odometry()
{
	printf("Destructing Odometry Plugin!\n");

	this->update_connection_.reset();
	this->rosQueue.clear();
	this->rosQueue.disable();
	this->rosNode->shutdown();
	// this->rosQueueThread.join();
	this->rosQueueThread.detach();

}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void
Odometry::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
	// Store the pointer to the model
	this->model_ = _parent;

	estimate_x     = 0;
	estimate_y     = 0;
	estimate_omega = 0;

	last_sent_time_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();

	//get the model-name
	this->name_ = model_->GetName();
	printf("Loading Odometry Plugin of model %s\n", name_.c_str());

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->update_connection_ =
	  event::Events::ConnectWorldUpdateBegin(boost::bind(&Odometry::OnUpdate, this, _1));

	//Create the communication Node for communication with fawkes
	this->node_ = transport::NodePtr(new transport::Node());

	//the namespace is set to the model name!
	this->node_->Init(model_->GetWorld()->GZWRAP_NAME() + "/" + name_);

	//init last sent time
	last_sent_time_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();

	//create publisher
	this->odometry_pub_ = this->node_->Advertise<msgs::Vector3d>("~/RobotinoSim/Odometry/");

	//create subscriber
	this->set_odometry_sub_ = this->node_->Subscribe(std::string("~/RobotinoSim/SetOdometry/"),
	                                                &Odometry::on_set_odometry_msg,
	                                                this);

	// subscriber for ROS
	//  Initialize ros, if it has not already bee initialized.
	std::string clientName= "gazsim_odom" + this->model_->GetName();
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

        this->odom_pub = this->rosNode->advertise<nav_msgs::Odometry>("/"+this->model_->GetName()+"/odom", 50);
        printf("Publish /%s%s\n", this->model_->GetName().c_str(), "/odom");

        this->current_time = ros::Time::now();
        this->last_time = ros::Time::now();

	// Create a named topic, and subscribe to it.
	ros::AdvertiseServiceOptions aso =
	  ros::AdvertiseServiceOptions::create<robotino_msgs::ResetOdometry>(
	      "/" + this->model_->GetName() + "/reset_odometry",
	      boost::bind(&Odometry::OnRosMsg, this, _1, _2),
	      ros::VoidPtr(), &this->rosQueue);
	this->rosAdv = this->rosNode->advertiseService(aso);
	printf("AdvertiseService %s%s%s\n", "/", this->model_->GetName().c_str(), "/reset_odometry");


	// Spin up the queue helper thread.
	// sleep(1);
	// this->rosQueueThread =
	//   std::thread(std::bind(&Odometry::QueueThread, this));
	//
	this->rosQueueThread =
	  boost::thread(boost::bind(&Odometry::QueueThread, this));

}

/** Called by the world update start event
 */
void
Odometry::OnUpdate(const common::UpdateInfo & /*_info*/)
{
	//Send position information to Fawkes
	double time = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
	if (time - last_sent_time_ > (1.0 / 10.0)) {
		send_position();
		last_sent_time_ = time;
	}
}

/** on Gazebo reset
 */
void
Odometry::Reset()
{
}

/** Functions for recieving Messages (registerd via suscribers)
 * @param msg message
 */
void
Odometry::on_set_odometry_msg(ConstVector3dPtr &msg)
{
	//std::cout << "Got new odometry: " << msg->x() << "|" << msg->y() << "|" << msg->z() << std::endl;
	{
		boost::mutex::scoped_lock lock(readingsMutex);
		estimate_x      = msg->x();
		estimate_y      = msg->y();
		estimate_omega  = msg->z();
		last_sent_time_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
	}
}

/** Sending position to Fawkes
 * 
 */
void
Odometry::send_position()
{
	#define GZWRAP_RELATIVE_POSE RelativePose
	gzwrap::Vector3d linearVel  = this->model_->GZWRAP_RELATIVE_LINEAR_VEL();
	gzwrap::Vector3d angularVel = this->model_->GZWRAP_RELATIVE_ANGULAR_VEL();
	gzwrap::Pose3d realPose   = this->model_->GZWRAP_RELATIVE_POSE();

	//get the elapsed time since last update
	double elapsedSeconds = model_->GetWorld()->GZWRAP_SIM_TIME().Double() - last_sent_time_;

	//now add some simulated error
	// rand multiplied by max noise which is in seconds
	elapsedSeconds += ((rand() % 2001 - 1000) / 1000.0) * 0.1;

	// we need to project the velocity values from robot base to world base
	float vecX = linearVel.GZWRAP_X;
	float vecY = linearVel.GZWRAP_Y;

	//rotate vector by current robot angle
	float cs = cos(-estimate_omega * 1.0 + 1.5708 * 1.0);
	float sn = sin(-estimate_omega * 1.0 + 1.5708 * 1.0);
	float tx = vecX * cs - vecY * sn;
	float ty = -(vecX * sn + vecY * cs);
	// printf("estimate : %f, %f ,%f\n", tx, ty, estimate_omega);

	// now update robot's position
	{
		// boost::mutex::scoped_lock lock(readingsMutex);
		estimate_x += tx * elapsedSeconds;
		estimate_y += ty * elapsedSeconds;
		estimate_omega += angularVel.GZWRAP_Z * elapsedSeconds;
		estimate_omega = this->model_->GZWRAP_WORLD_POSE().GZWRAP_ROT_EULER_Z;
		// printf("angularVel.z %f, %f, %f\n", angularVel.GZWRAP_Z, elapsedSeconds, angularVel.GZWRAP_Z * elapsedSeconds);
		// printf("linearVel.Z %f\n", linearVel.GZWRAP_Z);
		if (estimate_omega < -M_PI)
			estimate_omega = 2 * M_PI - estimate_omega;
		else if (estimate_omega > M_PI)
			estimate_omega = -2 * M_PI + estimate_omega;
		// printf("estimate_omega: %f\n", estimate_omega);
	}

	if (odometry_pub_->HasConnections()) {
		//build message
		msgs::Vector3d posMsg;
		posMsg.set_x(estimate_x);
		posMsg.set_y(estimate_y);
		posMsg.set_z(estimate_omega);

		//send
		odometry_pub_->Publish(posMsg);
	}

	ros::spinOnce();
	this->current_time = ros::Time::now();
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(estimate_omega);
	    
	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
    	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	
	//set the position
   	odom.pose.pose.position.x = estimate_x;
	odom.pose.pose.position.y = estimate_y;
	odom.pose.pose.position.z = 0.0 + estimate_omega;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = tx;
	odom.twist.twist.linear.y = ty;
	odom.twist.twist.angular.z = angularVel.GZWRAP_Z;

   	//publish the message
	odom_pub.publish(odom);
	//printf("publish\n");

}

// \brief Handle an incoming message from ROS
// \param[in] _msg A float value that is used to set the velocity
// of the Velodyne.
// void
// Motor::OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
//void
//Odometry::OnRosMsg(const nav_msgs::OdometryConstPtr &_msg)
bool
Odometry::OnRosMsg(robotino_msgs::ResetOdometry::Request &req,
                   robotino_msgs::ResetOdometry::Response &res)
{
        // this->SetVelocity(_msg->data);

	boost::mutex::scoped_lock lock(readingsMutex);
 	estimate_x      = req.x;
	estimate_y      = req.y;
	// estimate_omega  = pose_.pose.position.z;
	estimate_omega	= req.phi;
	// printf("%f\n", req.phi);
	last_sent_time_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
	return true;
}

/// \brief ROS helper function that processes messages
void
Odometry::QueueThread()
{
  static const double timeout = 0.01;
  // printf("start queueThread\n");
  while (this->rosNode->ok())
  {
    printf("Odometry::QueueThread\n");
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
    // printf("timeout: %f\n", timeout);
    // this->rosQueue.callAvailable();
    // printf("QueueThread: %d\n", this->rosNode->ok());
  }
  printf("QueueThread: finished\n");
}
