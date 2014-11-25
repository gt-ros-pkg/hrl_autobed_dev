/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Daehyung Park (Dr. Charles C. Kemp's HRL, GIT).
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * <02/01/2013>
 * This code includes a part of 'simulator_gazebo' package (BSD License)
 * of Willow Garage, Inc.
 * 
 * Daehyung Park is with Dr. Charles C. Kemp's the Healthcare Robotics Lab, 
 * Center for Robotics and Intelligent Machines, Georgia Institute of 
 * Technology. (Contact: deric.park@gatech.edu)
 *
 * We gratefully acknowledge support from DARPA Maximum Mobility
 * and Manipulation (M3) Contract W911NF-11-1-603.
 *********************************************************************/

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/Events.hh>
#include <common/common.hh>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "rosgraph_msgs/Clock.h"
#include "sttr_msgs/RagdollObjectArray.h"
#include "sttr_msgs/RagdollWrenchArray.h"
#include "sttr_srvs/GazeboObjectState.h"
#include "sttr_msgs/WrenchArray.h"

namespace gazebo
{   
  class ROSWorldPlugin : public WorldPlugin
  {

    public: ROSWorldPlugin()
    {
      this->world_created_ = false;
    }
    public: ~ROSWorldPlugin()
    {
      // disconnect slots
      event::Events::DisconnectWorldUpdateBegin(this->time_update_event_);
      event::Events::DisconnectWorldUpdateBegin(this->object_update_event_);
      // shutdown ros
      this->rosnode_->shutdown();
      delete this->rosnode_;

    }

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // setup ros related
      if (!ros::isInitialized()){
        std::string name = "ros_world_plugin_node";
        int argc = 0;
        ros::init(argc,NULL,name,ros::init_options::NoSigintHandler);
      }
      else
        ROS_ERROR("Something other than this ros_world plugin started ros::init(...), clock may not be published properly.");

      this->rosnode_ = new ros::NodeHandle("~");

      this->lock_.lock();
      if (this->world_created_)
      {
        this->lock_.unlock();
        return;
      }

      // set flag to true and load this plugin
      this->world_created_ = true;
      this->lock_.unlock();

      this->world = physics::get_world(_parent->GetName());
      if (!this->world)
      {
        ROS_FATAL("cannot load gazebo ros world server plugin, physics::get_world() fails to return world");
        return;
      }

      /// \brief advertise all services
      this->AdvertiseServices();

      // hooks for applying forces, publishing simtime on /clock
      this->time_update_event_   = event::Events::ConnectWorldUpdateBegin(boost::bind(&ROSWorldPlugin::publishSimTime,this));
      this->object_update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ROSWorldPlugin::publishObjectInfo,this));
    }


    /// \brief advertise services
    void AdvertiseServices()
    {
      // publish clock for simulated ros time
      this->pub_clock_         = this->rosnode_->advertise<rosgraph_msgs::Clock>("/clock",1/0.0005);
      //Will publish ragdoll coordinates
      this->pub_ragdollcog_     = this->rosnode_->advertise<sttr_msgs::RagdollObjectArray>("/gazebo/ragdollcog", 100);
      // set param for use_sim_time if not set by user alread
      this->rosnode_->setParam("/use_sim_time", true);
    }

    void publishSimTime()
    {
      common::Time currentTime = this->world->GetSimTime();
      rosgraph_msgs::Clock ros_time_;
      ros_time_.clock.fromSec(currentTime.Double());
      //  publish time to ros
      this->pub_clock_.publish(ros_time_);
    }
    // Publish Object Properties and Positions
    void publishObjectInfo()
    {
      this->lock_.lock();
      physics::Model_V models_;
      unsigned int nModel = 0;
      physics::Link_V links_;
      physics::LinkPtr link_;
      physics::LinkPtr child_link_;
      physics::Joint_V joints_;
      std::string linkName;
      std::string jointName;
      math::Pose obs_pose;
      math::Pose obj_pose;
      math::Vector3 object_forces;
      math::Vector3 object_torques;
      math::Vector3 crona_forces_at_forearm;
      math::Vector3 crona_torques_at_forearm;
      physics::JointWrench crona_jt;
      math::Vector3 crona_forces;
      math::Vector3 crona_torques;
      math::Vector3 crona_jointtorque;
      sdf::ElementPtr bodyElem_;
      physics::InertialPtr inertial_;
      math::Pose link_cog;
      math::Pose link_pose;
      double ragdoll_cog_num_x = 0;
      double ragdoll_cog_num_y = 0;
      double ragdoll_cog_num_z = 0;
      double ragdoll_cog_den = 0;
      double ragdoll_m1_num_x = 0;
      double ragdoll_m1_num_y = 0;
      double ragdoll_m1_num_z = 0;
      double ragdoll_m1_den = 0;
      double ragdoll_m2_num_x = 0;
      double ragdoll_m2_num_y = 0;
      double ragdoll_m2_num_z = 0;
      double ragdoll_m2_den = 0;
      double ragdoll_m3_num_x = 0;
      double ragdoll_m3_num_y = 0;
      double ragdoll_m3_num_z = 0;
      double ragdoll_m3_den = 0;
      double lower_body_mass = 10.14;
      double l_thigh_mass = 9.912;
      double l_shin_mass = 3.031;
      double l_foot_mass = 0.959;
      double r_thigh_mass = 9.912;
      double r_shin_mass = 3.031;
      double r_foot_mass = 0.959;
      double middle_body_mass = 10.14;
      double upper_body_mass = 10.14;
      double l_arm_mass = 1.897;
      double l_wrist_mass = 1.135;
      double l_hand_mass = 0.427;
      double neck_mass = 1.1;
      double head_mass = 2.429;
      double r_arm_mass = 1.897;
      double r_wrist_mass = 1.135;
      double r_hand_mass = 0.427;
      math::Pose ragdoll_m1;
      math::Pose ragdoll_m2;
      math::Pose ragdoll_m3;
      math::Pose ragdoll_cog;
      math::Pose ragdoll_lower_body_cog;
      math::Pose ragdoll_l_thigh_cog;
      //math::Pose ragdoll_l_shin_cog;
      //math::Pose ragdoll_l_foot_cog;
      math::Pose ragdoll_r_thigh_cog;
      //math::Pose ragdoll_r_shin_cog;
      //math::Pose ragdoll_r_foot_cog;
      //math::Pose ragdoll_middle_body_cog;
      math::Pose ragdoll_upper_body_cog;
      //math::Pose ragdoll_l_arm_cog;
      //math::Pose ragdoll_l_wrist_cog;
      //math::Pose ragdoll_l_hand_cog;
      //math::Pose ragdoll_neck_cog;
      //math::Pose ragdoll_head_cog;
      //math::Pose ragdoll_r_arm_cog;
      //math::Pose ragdoll_r_wrist_cog;
      //math::Pose ragdoll_r_hand_cog;
      math::Pose ragdoll_lower_body_pose;
      math::Pose ragdoll_l_thigh_pose;
      math::Pose ragdoll_l_shin_pose;
      math::Pose ragdoll_l_foot_pose;
      math::Pose ragdoll_r_thigh_pose;
      math::Pose ragdoll_r_shin_pose;
      math::Pose ragdoll_r_foot_pose;
      math::Pose ragdoll_middle_body_pose;
      math::Pose ragdoll_upper_body_pose;
      math::Pose ragdoll_l_arm_pose;
      math::Pose ragdoll_l_wrist_pose;
      math::Pose ragdoll_l_hand_pose;
      math::Pose ragdoll_neck_pose;
      math::Pose ragdoll_head_pose;
      math::Pose ragdoll_r_arm_pose;
      math::Pose ragdoll_r_wrist_pose;
      math::Pose ragdoll_r_hand_pose;

      models_ = this->world->GetModels();
      nModel = this->world->GetModelCount();

     // printf("******************************* NO OF MODELS ******************************* %d \n", nModel);
     // if(nModel == 3){
     // std::string modelName_o = models_[2]->GetName();
     // printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %s",modelName_o.c_str());
      // Set header
      //}
      for (unsigned int i = 0; i < nModel; i++){
        std::string modelName = models_[i]->GetName();
        // Get only obstacles
        if (modelName.find("new_ragdoll") != std::string::npos){
          obs_pose = models_[i]->GetWorldPose();
  	  obj_pose = links_[0]->GetWorldCoGPose();
          //this->pub_objectcog_array_.frame_names.push_back("small_lift_box"); 
	
 	  links_ = models_[i]->GetLinks();
	  for (unsigned int j = 0; j < links_.size(); j++){
            std::string linkName = links_[j]->GetName();
	    //inertial_ = links_[j]->GetInertial();
	    //double mass = inertial_->GetMass();
	    link_cog = links_[j]->GetWorldCoGPose();
	    link_pose = links_[j]->GetCollision(0)->GetWorldPose();
	    if (linkName.find("lower_body") != std::string::npos){
		ragdoll_lower_body_cog.pos.x = link_cog.pos.x;
		ragdoll_lower_body_cog.pos.y = link_cog.pos.y;
		ragdoll_lower_body_cog.pos.z = link_cog.pos.z;
	        ragdoll_lower_body_cog.rot.x = link_cog.rot.x;
                ragdoll_lower_body_cog.rot.y = link_cog.rot.y;
                ragdoll_lower_body_cog.rot.z = link_cog.rot.z;
                ragdoll_lower_body_cog.rot.w = link_cog.rot.w;
		ragdoll_lower_body_pose.pos.x = link_pose.pos.x;
                ragdoll_lower_body_pose.pos.y = link_pose.pos.y;
                ragdoll_lower_body_pose.pos.z = link_pose.pos.z;
                ragdoll_lower_body_pose.rot.x = link_pose.rot.x;
                ragdoll_lower_body_pose.rot.y = link_pose.rot.y;
                ragdoll_lower_body_pose.rot.z = link_pose.rot.z;
                ragdoll_lower_body_pose.rot.w = link_pose.rot.w;
	  	ragdoll_m2_num_x = ragdoll_m2_num_x + lower_body_mass*link_cog.pos.x;
                ragdoll_m2_num_y = ragdoll_m2_num_y + lower_body_mass*link_cog.pos.y;
                ragdoll_m2_num_z = ragdoll_m2_num_z + lower_body_mass*link_cog.pos.z;
                ragdoll_m2_den = ragdoll_m2_den + lower_body_mass;
		ragdoll_cog_num_x = ragdoll_cog_num_x + lower_body_mass*link_cog.pos.x;
            	ragdoll_cog_num_y = ragdoll_cog_num_y + lower_body_mass*link_cog.pos.y;
            	ragdoll_cog_num_z = ragdoll_cog_num_z + lower_body_mass*link_cog.pos.z;
            	ragdoll_cog_den = ragdoll_cog_den + lower_body_mass;
		
	    }
	    if (linkName.find("l_thigh") != std::string::npos){
                ragdoll_l_thigh_cog.pos.x = link_cog.pos.x;
                ragdoll_l_thigh_cog.pos.y = link_cog.pos.y;
                ragdoll_l_thigh_cog.pos.z = link_cog.pos.z;
		ragdoll_l_thigh_cog.rot.x = link_cog.rot.x;
                ragdoll_l_thigh_cog.rot.y = link_cog.rot.y;
                ragdoll_l_thigh_cog.rot.z = link_cog.rot.z;
                ragdoll_l_thigh_cog.rot.w = link_cog.rot.w;
		ragdoll_l_thigh_pose.pos.x = link_pose.pos.x;
                ragdoll_l_thigh_pose.pos.y = link_pose.pos.y;
                ragdoll_l_thigh_pose.pos.z = link_pose.pos.z;
                ragdoll_l_thigh_pose.rot.x = link_pose.rot.x;
                ragdoll_l_thigh_pose.rot.y = link_pose.rot.y;
                ragdoll_l_thigh_pose.rot.z = link_pose.rot.z;
                ragdoll_l_thigh_pose.rot.w = link_pose.rot.w;
		ragdoll_m2_num_x = ragdoll_m2_num_x + l_thigh_mass*link_cog.pos.x;
                ragdoll_m2_num_y = ragdoll_m2_num_y + l_thigh_mass*link_cog.pos.y;
                ragdoll_m2_num_z = ragdoll_m2_num_z + l_thigh_mass*link_cog.pos.z;
                ragdoll_m2_den = ragdoll_m2_den + l_thigh_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + l_thigh_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + l_thigh_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + l_thigh_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + l_thigh_mass;
            }
	    if (linkName.find("l_shin") != std::string::npos){
		//ragdoll_l_shin_cog.pos.x = link_cog.pos.x;
                //ragdoll_l_shin_cog.pos.y = link_cog.pos.y;
                //ragdoll_l_shin_cog.pos.z = link_cog.pos.z;
                //ragdoll_l_shin_cog.rot.x = link_cog.rot.x;
                //ragdoll_l_shin_cog.rot.y = link_cog.rot.y;
                //ragdoll_l_shin_cog.rot.z = link_cog.rot.z;
                //ragdoll_l_shin_cog.rot.w = link_cog.rot.w;
		ragdoll_l_shin_pose.pos.x = link_pose.pos.x;
                ragdoll_l_shin_pose.pos.y = link_pose.pos.y;
                ragdoll_l_shin_pose.pos.z = link_pose.pos.z;
                ragdoll_l_shin_pose.rot.x = link_pose.rot.x;
                ragdoll_l_shin_pose.rot.y = link_pose.rot.y;
                ragdoll_l_shin_pose.rot.z = link_pose.rot.z;
                ragdoll_l_shin_pose.rot.w = link_pose.rot.w;
		ragdoll_m1_num_x = ragdoll_m1_num_x + l_shin_mass*link_cog.pos.x;
                ragdoll_m1_num_y = ragdoll_m1_num_y + l_shin_mass*link_cog.pos.y;
                ragdoll_m1_num_z = ragdoll_m1_num_z + l_shin_mass*link_cog.pos.z;
                ragdoll_m1_den = ragdoll_m1_den + l_shin_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + l_shin_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + l_shin_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + l_shin_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + l_shin_mass;
            }
	    if (linkName.find("l_foot") != std::string::npos){
		//ragdoll_l_foot_cog.pos.x = link_cog.pos.x;
                //ragdoll_l_foot_cog.pos.y = link_cog.pos.y;
                //ragdoll_l_foot_cog.pos.z = link_cog.pos.z;
                //ragdoll_l_foot_cog.rot.x = link_cog.rot.x;
                //ragdoll_l_foot_cog.rot.y = link_cog.rot.y;
                //ragdoll_l_foot_cog.rot.z = link_cog.rot.z;
                //ragdoll_l_foot_cog.rot.w = link_cog.rot.w;
		ragdoll_l_foot_pose.pos.x = link_pose.pos.x;
                ragdoll_l_foot_pose.pos.y = link_pose.pos.y;
                ragdoll_l_foot_pose.pos.z = link_pose.pos.z;
                ragdoll_l_foot_pose.rot.x = link_pose.rot.x;
                ragdoll_l_foot_pose.rot.y = link_pose.rot.y;
                ragdoll_l_foot_pose.rot.z = link_pose.rot.z;
                ragdoll_l_foot_pose.rot.w = link_pose.rot.w;
		ragdoll_m1_num_x = ragdoll_m1_num_x + l_foot_mass*link_cog.pos.x;
                ragdoll_m1_num_y = ragdoll_m1_num_y + l_foot_mass*link_cog.pos.y;
                ragdoll_m1_num_z = ragdoll_m1_num_z + l_foot_mass*link_cog.pos.z;
                ragdoll_m1_den = ragdoll_m1_den + l_foot_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + l_foot_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + l_foot_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + l_foot_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + l_foot_mass;
            }
	    if (linkName.find("r_thigh") != std::string::npos){
                ragdoll_r_thigh_cog.pos.x = link_cog.pos.x;
                ragdoll_r_thigh_cog.pos.y = link_cog.pos.y;
                ragdoll_r_thigh_cog.pos.z = link_cog.pos.z;
		ragdoll_r_thigh_cog.rot.x = link_cog.rot.x;
                ragdoll_r_thigh_cog.rot.y = link_cog.rot.y;
                ragdoll_r_thigh_cog.rot.z = link_cog.rot.z;
                ragdoll_r_thigh_cog.rot.w = link_cog.rot.w;
		ragdoll_r_thigh_pose.pos.x = link_pose.pos.x;
                ragdoll_r_thigh_pose.pos.y = link_pose.pos.y;
                ragdoll_r_thigh_pose.pos.z = link_pose.pos.z;
                ragdoll_r_thigh_pose.rot.x = link_pose.rot.x;
                ragdoll_r_thigh_pose.rot.y = link_pose.rot.y;
                ragdoll_r_thigh_pose.rot.z = link_pose.rot.z;
                ragdoll_r_thigh_pose.rot.w = link_pose.rot.w;
		ragdoll_m2_num_x = ragdoll_m2_num_x + r_thigh_mass*link_cog.pos.x;
                ragdoll_m2_num_y = ragdoll_m2_num_y + r_thigh_mass*link_cog.pos.y;
                ragdoll_m2_num_z = ragdoll_m2_num_z + r_thigh_mass*link_cog.pos.z;
                ragdoll_m2_den = ragdoll_m2_den + r_thigh_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + r_thigh_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + r_thigh_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + r_thigh_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + r_thigh_mass;
		ragdoll_m2.rot.x = link_cog.rot.x;
		ragdoll_m2.rot.y = link_cog.rot.y;
		ragdoll_m2.rot.z = link_cog.rot.z;
		ragdoll_m2.rot.w = link_cog.rot.w;
                }
	    if (linkName.find("r_shin") != std::string::npos){
		//ragdoll_r_shin_cog.pos.x = link_cog.pos.x;
                //ragdoll_r_shin_cog.pos.y = link_cog.pos.y;
                //ragdoll_r_shin_cog.pos.z = link_cog.pos.z;
                //ragdoll_r_shin_cog.rot.x = link_cog.rot.x;
                //ragdoll_r_shin_cog.rot.y = link_cog.rot.y;
                //ragdoll_r_shin_cog.rot.z = link_cog.rot.z;
                //ragdoll_r_shin_cog.rot.w = link_cog.rot.w;
		ragdoll_r_shin_pose.pos.x = link_pose.pos.x;
                ragdoll_r_shin_pose.pos.y = link_pose.pos.y;
                ragdoll_r_shin_pose.pos.z = link_pose.pos.z;
                ragdoll_r_shin_pose.rot.x = link_pose.rot.x;
                ragdoll_r_shin_pose.rot.y = link_pose.rot.y;
                ragdoll_r_shin_pose.rot.z = link_pose.rot.z;
                ragdoll_r_shin_pose.rot.w = link_pose.rot.w;
		ragdoll_m1_num_x = ragdoll_m1_num_x + r_shin_mass*link_cog.pos.x;
                ragdoll_m1_num_y = ragdoll_m1_num_y + r_shin_mass*link_cog.pos.y;
                ragdoll_m1_num_z = ragdoll_m1_num_z + r_shin_mass*link_cog.pos.z;
                ragdoll_m1_den = ragdoll_m1_den + r_shin_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + r_shin_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + r_shin_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + r_shin_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + r_shin_mass;
		ragdoll_m1.rot.x = link_cog.rot.x;
                ragdoll_m1.rot.y = link_cog.rot.y;
                ragdoll_m1.rot.z = link_cog.rot.z;
                ragdoll_m1.rot.w = link_cog.rot.w;
	    }
	    if (linkName.find("r_foot") != std::string::npos){
		//ragdoll_r_foot_cog.pos.x = link_cog.pos.x;
                //ragdoll_r_foot_cog.pos.y = link_cog.pos.y;
                //ragdoll_r_foot_cog.pos.z = link_cog.pos.z;
                //ragdoll_r_foot_cog.rot.x = link_cog.rot.x;
                //ragdoll_r_foot_cog.rot.y = link_cog.rot.y;
                //ragdoll_r_foot_cog.rot.z = link_cog.rot.z;
                //ragdoll_r_foot_cog.rot.w = link_cog.rot.w;
		ragdoll_r_foot_pose.pos.x = link_pose.pos.x;
                ragdoll_r_foot_pose.pos.y = link_pose.pos.y;
                ragdoll_r_foot_pose.pos.z = link_pose.pos.z;
                ragdoll_r_foot_pose.rot.x = link_pose.rot.x;
                ragdoll_r_foot_pose.rot.y = link_pose.rot.y;
                ragdoll_r_foot_pose.rot.z = link_pose.rot.z;
                ragdoll_r_foot_pose.rot.w = link_pose.rot.w;
		ragdoll_m1_num_x = ragdoll_m1_num_x + r_foot_mass*link_cog.pos.x;
                ragdoll_m1_num_y = ragdoll_m1_num_y + r_foot_mass*link_cog.pos.y;
                ragdoll_m1_num_z = ragdoll_m1_num_z + r_foot_mass*link_cog.pos.z;
                ragdoll_m1_den = ragdoll_m1_den + r_foot_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + r_foot_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + r_foot_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + r_foot_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + r_foot_mass;
            }
	    if (linkName.find("middle_body") != std::string::npos){
		//ragdoll_middle_body_cog.pos.x = link_cog.pos.x;
                //ragdoll_middle_body_cog.pos.y = link_cog.pos.y;
                //ragdoll_middle_body_cog.pos.z = link_cog.pos.z;
                //ragdoll_middle_body_cog.rot.x = link_cog.rot.x;
                //ragdoll_middle_body_cog.rot.y = link_cog.rot.y;
                //ragdoll_middle_body_cog.rot.z = link_cog.rot.z;
                //ragdoll_middle_body_cog.rot.w = link_cog.rot.w;
		ragdoll_middle_body_pose.pos.x = link_pose.pos.x;
                ragdoll_middle_body_pose.pos.y = link_pose.pos.y;
                ragdoll_middle_body_pose.pos.z = link_pose.pos.z;
                ragdoll_middle_body_pose.rot.x = link_pose.rot.x;
                ragdoll_middle_body_pose.rot.y = link_pose.rot.y;
                ragdoll_middle_body_pose.rot.z = link_pose.rot.z;
                ragdoll_middle_body_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + middle_body_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + middle_body_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + middle_body_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + middle_body_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + middle_body_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + middle_body_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + middle_body_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + middle_body_mass;
            }
	    if (linkName.find("upper_body") != std::string::npos){
                ragdoll_upper_body_cog.pos.x = link_cog.pos.x;
                ragdoll_upper_body_cog.pos.y = link_cog.pos.y;
                ragdoll_upper_body_cog.pos.z = link_cog.pos.z;
		ragdoll_upper_body_cog.rot.x = link_cog.rot.x;
                ragdoll_upper_body_cog.rot.y = link_cog.rot.y;
                ragdoll_upper_body_cog.rot.z = link_cog.rot.z;
                ragdoll_upper_body_cog.rot.w = link_cog.rot.w;
		ragdoll_upper_body_pose.pos.x = link_pose.pos.x;
                ragdoll_upper_body_pose.pos.y = link_pose.pos.y;
                ragdoll_upper_body_pose.pos.z = link_pose.pos.z;
                ragdoll_upper_body_pose.rot.x = link_pose.rot.x;
                ragdoll_upper_body_pose.rot.y = link_pose.rot.y;
                ragdoll_upper_body_pose.rot.z = link_pose.rot.z;
                ragdoll_upper_body_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + upper_body_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + upper_body_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + upper_body_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + upper_body_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + upper_body_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + upper_body_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + upper_body_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + upper_body_mass;
		ragdoll_m3.rot.x = link_cog.rot.x;
                ragdoll_m3.rot.y = link_cog.rot.y;
                ragdoll_m3.rot.z = link_cog.rot.z;
                ragdoll_m3.rot.w = link_cog.rot.w;
            }
	    if (linkName.find("l_arm") != std::string::npos){
		//ragdoll_l_arm_cog.pos.x = link_cog.pos.x;
                //ragdoll_l_arm_cog.pos.y = link_cog.pos.y;
                //ragdoll_l_arm_cog.pos.z = link_cog.pos.z;
                //ragdoll_l_arm_cog.rot.x = link_cog.rot.x;
                //ragdoll_l_arm_cog.rot.y = link_cog.rot.y;
                //ragdoll_l_arm_cog.rot.z = link_cog.rot.z;
                //ragdoll_l_arm_cog.rot.w = link_cog.rot.w;
		ragdoll_l_arm_pose.pos.x = link_pose.pos.x;
                ragdoll_l_arm_pose.pos.y = link_pose.pos.y;
                ragdoll_l_arm_pose.pos.z = link_pose.pos.z;
                ragdoll_l_arm_pose.rot.x = link_pose.rot.x;
                ragdoll_l_arm_pose.rot.y = link_pose.rot.y;
                ragdoll_l_arm_pose.rot.z = link_pose.rot.z;
                ragdoll_l_arm_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + l_arm_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + l_arm_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + l_arm_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + l_arm_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + l_arm_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + l_arm_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + l_arm_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + l_arm_mass;
            }
	    if (linkName.find("l_wrist") != std::string::npos){
		//ragdoll_l_wrist_cog.pos.x = link_cog.pos.x;
                //ragdoll_l_wrist_cog.pos.y = link_cog.pos.y;
                //ragdoll_l_wrist_cog.pos.z = link_cog.pos.z;
                //ragdoll_l_wrist_cog.rot.x = link_cog.rot.x;
                //ragdoll_l_wrist_cog.rot.y = link_cog.rot.y;
                //ragdoll_l_wrist_cog.rot.z = link_cog.rot.z;
                //ragdoll_l_wrist_cog.rot.w = link_cog.rot.w;
		ragdoll_l_wrist_pose.pos.x = link_pose.pos.x;
                ragdoll_l_wrist_pose.pos.y = link_pose.pos.y;
                ragdoll_l_wrist_pose.pos.z = link_pose.pos.z;
                ragdoll_l_wrist_pose.rot.x = link_pose.rot.x;
                ragdoll_l_wrist_pose.rot.y = link_pose.rot.y;
                ragdoll_l_wrist_pose.rot.z = link_pose.rot.z;
                ragdoll_l_wrist_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + l_wrist_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + l_wrist_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + l_wrist_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + l_wrist_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + l_wrist_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + l_wrist_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + l_wrist_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + l_wrist_mass;
            }
	    if (linkName.find("l_hand") != std::string::npos){
		//ragdoll_l_hand_cog.pos.x = link_cog.pos.x;
                //ragdoll_l_hand_cog.pos.y = link_cog.pos.y;
                //ragdoll_l_hand_cog.pos.z = link_cog.pos.z;
                //ragdoll_l_hand_cog.rot.x = link_cog.rot.x;
                //ragdoll_l_hand_cog.rot.y = link_cog.rot.y;
                //ragdoll_l_hand_cog.rot.z = link_cog.rot.z;
                //ragdoll_l_hand_cog.rot.w = link_cog.rot.w;
		ragdoll_l_hand_pose.pos.x = link_pose.pos.x;
                ragdoll_l_hand_pose.pos.y = link_pose.pos.y;
                ragdoll_l_hand_pose.pos.z = link_pose.pos.z;
                ragdoll_l_hand_pose.rot.x = link_pose.rot.x;
                ragdoll_l_hand_pose.rot.y = link_pose.rot.y;
                ragdoll_l_hand_pose.rot.z = link_pose.rot.z;
                ragdoll_l_hand_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + l_hand_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + l_hand_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + l_hand_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + l_hand_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + l_hand_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + l_hand_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + l_hand_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + l_hand_mass;
            }
	    if (linkName.find("neck") != std::string::npos){
		//ragdoll_neck_cog.pos.x = link_cog.pos.x;
                //ragdoll_neck_cog.pos.y = link_cog.pos.y;
                //ragdoll_neck_cog.pos.z = link_cog.pos.z;
                //ragdoll_neck_cog.rot.x = link_cog.rot.x;
                //ragdoll_neck_cog.rot.y = link_cog.rot.y;
                //ragdoll_neck_cog.rot.z = link_cog.rot.z;
                //ragdoll_neck_cog.rot.w = link_cog.rot.w;
		ragdoll_neck_pose.pos.x = link_pose.pos.x;
                ragdoll_neck_pose.pos.y = link_pose.pos.y;
                ragdoll_neck_pose.pos.z = link_pose.pos.z;
                ragdoll_neck_pose.rot.x = link_pose.rot.x;
                ragdoll_neck_pose.rot.y = link_pose.rot.y;
                ragdoll_neck_pose.rot.z = link_pose.rot.z;
                ragdoll_neck_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + neck_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + neck_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + neck_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + neck_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + neck_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + neck_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + neck_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + neck_mass;
            }
            if (linkName.find("head") != std::string::npos){
		//ragdoll_head_cog.pos.x = link_cog.pos.x;
                //ragdoll_head_cog.pos.y = link_cog.pos.y;
                //ragdoll_head_cog.pos.z = link_cog.pos.z;
                //ragdoll_head_cog.rot.x = link_cog.rot.x;
                //ragdoll_head_cog.rot.y = link_cog.rot.y;
                //ragdoll_head_cog.rot.z = link_cog.rot.z;
                //ragdoll_head_cog.rot.w = link_cog.rot.w;
		ragdoll_head_pose.pos.x = link_pose.pos.x;
                ragdoll_head_pose.pos.y = link_pose.pos.y;
                ragdoll_head_pose.pos.z = link_pose.pos.z;
                ragdoll_head_pose.rot.x = link_pose.rot.x;
                ragdoll_head_pose.rot.y = link_pose.rot.y;
                ragdoll_head_pose.rot.z = link_pose.rot.z;
                ragdoll_head_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + head_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + head_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + head_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + head_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + head_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + head_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + head_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + head_mass;
            }
            if (linkName.find("r_arm") != std::string::npos){
		//ragdoll_r_arm_cog.pos.x = link_cog.pos.x;
                //ragdoll_r_arm_cog.pos.y = link_cog.pos.y;
                //ragdoll_r_arm_cog.pos.z = link_cog.pos.z;
                //ragdoll_r_arm_cog.rot.x = link_cog.rot.x;
                //ragdoll_r_arm_cog.rot.y = link_cog.rot.y;
                //ragdoll_r_arm_cog.rot.z = link_cog.rot.z;
                //ragdoll_r_arm_cog.rot.w = link_cog.rot.w;
		ragdoll_r_arm_pose.pos.x = link_pose.pos.x;
                ragdoll_r_arm_pose.pos.y = link_pose.pos.y;
                ragdoll_r_arm_pose.pos.z = link_pose.pos.z;
                ragdoll_r_arm_pose.rot.x = link_pose.rot.x;
                ragdoll_r_arm_pose.rot.y = link_pose.rot.y;
                ragdoll_r_arm_pose.rot.z = link_pose.rot.z;
                ragdoll_r_arm_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + r_arm_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + r_arm_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + r_arm_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + r_arm_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + r_arm_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + r_arm_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + r_arm_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + r_arm_mass;
            }
            if (linkName.find("r_wrist") != std::string::npos){
		//ragdoll_r_wrist_cog.pos.x = link_cog.pos.x;
                //ragdoll_r_wrist_cog.pos.y = link_cog.pos.y;
                //ragdoll_r_wrist_cog.pos.z = link_cog.pos.z;
                //ragdoll_r_wrist_cog.rot.x = link_cog.rot.x;
                //ragdoll_r_wrist_cog.rot.y = link_cog.rot.y;
                //ragdoll_r_wrist_cog.rot.z = link_cog.rot.z;
                //ragdoll_r_wrist_cog.rot.w = link_cog.rot.w;
		ragdoll_r_wrist_pose.pos.x = link_pose.pos.x;
                ragdoll_r_wrist_pose.pos.y = link_pose.pos.y;
                ragdoll_r_wrist_pose.pos.z = link_pose.pos.z;
                ragdoll_r_wrist_pose.rot.x = link_pose.rot.x;
                ragdoll_r_wrist_pose.rot.y = link_pose.rot.y;
                ragdoll_r_wrist_pose.rot.z = link_pose.rot.z;
                ragdoll_r_wrist_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + r_wrist_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + r_wrist_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + r_wrist_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + r_wrist_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + r_wrist_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + r_wrist_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + r_wrist_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + r_wrist_mass;
            }
	    if (linkName.find("r_hand") != std::string::npos){
		//ragdoll_r_hand_cog.pos.x = link_cog.pos.x;
                //ragdoll_r_hand_cog.pos.y = link_cog.pos.y;
                //ragdoll_r_hand_cog.pos.z = link_cog.pos.z;
                //ragdoll_r_hand_cog.rot.x = link_cog.rot.x;
                //ragdoll_r_hand_cog.rot.y = link_cog.rot.y;
                //ragdoll_r_hand_cog.rot.z = link_cog.rot.z;
                //ragdoll_r_hand_cog.rot.w = link_cog.rot.w;
		ragdoll_r_hand_pose.pos.x = link_pose.pos.x;
                ragdoll_r_hand_pose.pos.y = link_pose.pos.y;
                ragdoll_r_hand_pose.pos.z = link_pose.pos.z;
                ragdoll_r_hand_pose.rot.x = link_pose.rot.x;
                ragdoll_r_hand_pose.rot.y = link_pose.rot.y;
                ragdoll_r_hand_pose.rot.z = link_pose.rot.z;
                ragdoll_r_hand_pose.rot.w = link_pose.rot.w;
                ragdoll_m3_num_x = ragdoll_m3_num_x + r_hand_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + r_hand_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + r_hand_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + r_hand_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + r_hand_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + r_hand_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + r_hand_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + r_hand_mass;
            }
	
	  }

	  ragdoll_m1.pos.x = ragdoll_m1_num_x/ragdoll_m1_den;
          ragdoll_m1.pos.y = ragdoll_m1_num_y/ragdoll_m1_den;
          ragdoll_m1.pos.z = ragdoll_m1_num_z/ragdoll_m1_den;

          ragdoll_m2.pos.x = ragdoll_m2_num_x/ragdoll_m2_den;
          ragdoll_m2.pos.y = ragdoll_m2_num_y/ragdoll_m2_den;
          ragdoll_m2.pos.z = ragdoll_m2_num_z/ragdoll_m2_den;

          ragdoll_m3.pos.x = ragdoll_m3_num_x/ragdoll_m3_den;
          ragdoll_m3.pos.y = ragdoll_m3_num_y/ragdoll_m3_den;
          ragdoll_m3.pos.z = ragdoll_m3_num_z/ragdoll_m3_den;

          this->pub_objectcog_array_.frame_names.push_back("ragdoll_m1");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_m1.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_m1.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_m1.pos.z);
          this->pub_objectcog_array_.rotation_x.push_back(ragdoll_m1.rot.x);
          this->pub_objectcog_array_.rotation_y.push_back(ragdoll_m1.rot.y);
          this->pub_objectcog_array_.rotation_z.push_back(ragdoll_m1.rot.z);
          this->pub_objectcog_array_.rotation_w.push_back(ragdoll_m1.rot.w);
          this->pub_objectcog_array_.mass.push_back(ragdoll_m1_den);

          this->pub_objectcog_array_.frame_names.push_back("ragdoll_m2");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_m2.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_m2.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_m2.pos.z);
	  this->pub_objectcog_array_.rotation_x.push_back(ragdoll_m2.rot.x);
          this->pub_objectcog_array_.rotation_y.push_back(ragdoll_m2.rot.y);
          this->pub_objectcog_array_.rotation_z.push_back(ragdoll_m2.rot.z);
          this->pub_objectcog_array_.rotation_w.push_back(ragdoll_m2.rot.w);
          this->pub_objectcog_array_.mass.push_back(ragdoll_m2_den);

          this->pub_objectcog_array_.frame_names.push_back("ragdoll_m3");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_m3.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_m3.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_m3.pos.z);
	  this->pub_objectcog_array_.rotation_x.push_back(ragdoll_m3.rot.x);
          this->pub_objectcog_array_.rotation_y.push_back(ragdoll_m3.rot.y);
          this->pub_objectcog_array_.rotation_z.push_back(ragdoll_m3.rot.z);
          this->pub_objectcog_array_.rotation_w.push_back(ragdoll_m3.rot.w);
          this->pub_objectcog_array_.mass.push_back(ragdoll_m3_den);

	  this->pub_objectcog_array_.frame_names.push_back("ragdoll_lower_body_cog");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_lower_body_cog.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_lower_body_cog.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_lower_body_cog.pos.z);

	  this->pub_objectcog_array_.frame_names.push_back("ragdoll_l_thigh_cog");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_l_thigh_cog.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_l_thigh_cog.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_l_thigh_cog.pos.z);
	
	  this->pub_objectcog_array_.frame_names.push_back("ragdoll_r_thigh_cog");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_r_thigh_cog.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_r_thigh_cog.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_r_thigh_cog.pos.z);

	  this->pub_objectcog_array_.frame_names.push_back("ragdoll_upper_body_cog");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_upper_body_cog.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_upper_body_cog.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_upper_body_cog.pos.z);

	  ragdoll_cog.pos.x = ragdoll_cog_num_x/ragdoll_cog_den;
	  ragdoll_cog.pos.y = ragdoll_cog_num_y/ragdoll_cog_den;
	  ragdoll_cog.pos.z = ragdoll_cog_num_z/ragdoll_cog_den;
	  //gzdbg << "Ragdoll Center of Gravity: " << ragdoll_cog << "\n";

	  this->pub_objectcog_array_.frame_names.push_back("ragdoll_cog");
	  this->pub_objectcog_array_.centers_x.push_back(ragdoll_cog.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_cog.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_cog.pos.z);
	  this->pub_objectcog_array_.mass.push_back(ragdoll_cog_den);
	
	  // publish the cog of each ragdoll link
	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_lower_body_pose");
	  this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_lower_body_pose.pos.x);
	  this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_lower_body_pose.pos.y);
	  this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_lower_body_pose.pos.z);
	  this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_lower_body_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_lower_body_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_lower_body_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_lower_body_pose.rot.w);
	   
	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_l_thigh_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_l_thigh_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_l_thigh_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_l_thigh_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_l_thigh_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_l_thigh_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_l_thigh_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_l_thigh_pose.rot.w);

 	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_l_shin_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_l_shin_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_l_shin_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_l_shin_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_l_shin_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_l_shin_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_l_shin_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_l_shin_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_l_foot_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_l_foot_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_l_foot_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_l_foot_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_l_foot_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_l_foot_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_l_foot_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_l_foot_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_r_thigh_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_r_thigh_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_r_thigh_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_r_thigh_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_r_thigh_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_r_thigh_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_r_thigh_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_r_thigh_pose.rot.w);

          this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_r_shin_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_r_shin_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_r_shin_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_r_shin_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_r_shin_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_r_shin_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_r_shin_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_r_shin_pose.rot.w);

          this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_r_foot_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_r_foot_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_r_foot_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_r_foot_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_r_foot_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_r_foot_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_r_foot_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_r_foot_pose.rot.w);	

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_middle_body_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_middle_body_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_middle_body_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_middle_body_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_middle_body_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_middle_body_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_middle_body_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_middle_body_pose.rot.w);
	
	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_upper_body_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_upper_body_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_upper_body_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_upper_body_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_upper_body_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_upper_body_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_upper_body_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_upper_body_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_l_arm_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_l_arm_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_l_arm_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_l_arm_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_l_arm_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_l_arm_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_l_arm_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_l_arm_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_l_wrist_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_l_wrist_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_l_wrist_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_l_wrist_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_l_wrist_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_l_wrist_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_l_wrist_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_l_wrist_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_l_hand_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_l_hand_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_l_hand_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_l_hand_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_l_hand_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_l_hand_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_l_hand_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_l_hand_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_neck_cog");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_neck_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_neck_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_neck_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_neck_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_neck_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_neck_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_neck_pose.rot.w);

          this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_head_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_head_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_head_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_head_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_head_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_head_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_head_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_head_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_r_arm_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_r_arm_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_r_arm_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_r_arm_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_r_arm_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_r_arm_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_r_arm_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_r_arm_pose.rot.w);

          this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_r_wrist_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_r_wrist_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_r_wrist_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_r_wrist_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_r_wrist_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_r_wrist_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_r_wrist_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_r_wrist_pose.rot.w);

          this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_r_hand_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_r_hand_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_r_hand_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_r_hand_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_r_hand_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_r_hand_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_r_hand_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_r_hand_pose.rot.w);

	    }
      }
        //  publish object to ros
      this->pub_ragdollcog_.publish(this->pub_ragdollcog_array_);
      this->pub_ragdollcog_array_.frame_names.clear();
      this->pub_ragdollcog_array_.centers_x.clear();
      this->pub_ragdollcog_array_.centers_y.clear();
      this->pub_ragdollcog_array_.centers_z.clear();
      this->pub_ragdollcog_array_.rotation_x.clear();
      this->pub_ragdollcog_array_.rotation_y.clear();
      this->pub_ragdollcog_array_.rotation_z.clear();
      this->pub_ragdollcog_array_.rotation_w.clear();
      
      this->lock_.unlock();

      }
    // 
    private: physics::WorldPtr world;
    private: event::ConnectionPtr time_update_event_;
    private: event::ConnectionPtr object_update_event_;
    /// \brief A mutex to lock access to fields that are used in ROS message callbacks
    private: boost::mutex lock_;

    /// \brief utilites for checking incoming string URDF/XML/Param
    bool world_created_;

    ros::NodeHandle* rosnode_;

    // ROS Publisher & Subscriber
    ros::Publisher pub_clock_;
    // ROS Publisher & Subscriber
    ros::Publisher pub_ragdollcog_;
    // Object related variables
    math::Vector3 prev_torque_vector;
    //bool object_reset;
    sttr_msgs::RagdollObjectArray pub_object_array_;
    sttr_msgs::RagdollObjectArray pub_objectcog_array_;
    sttr_msgs::RagdollObjectArray pub_ragdollcog_array_;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ROSWorldPlugin)
}

