#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/World.hh>
#include <physics/physics.hh>
#include <sensors/sensors.hh>
#include <sensors/ContactSensor.hh>
#include <math/Pose.hh>
#include <math/Quaternion.hh>
#include <math/Vector3.hh>
#include <common/common.hh>
#include <stdio.h>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/TaxelArray.h"

#include "KMlocal.h"

namespace gazebo
{   
  class ROSTactilePlugin : public SensorPlugin
  {

    public: ROSTactilePlugin()
    {
      // KM variable initialization
      // execution parameters (see KMterm.h and KMlocal.h)
      this->KM_term = new KMterm(50, 0, 0, 0, 0.1, 0.1, 3, 0.5, 10, 0.95);
      // number of centers
      this->KM_k   = 3;
      // dimension
      this->KM_dim = 3;
      // init data pts object
      this->KM_dataPts = new KMdata(this->KM_dim, 3); //hard code the default nPts
    }
    public: ~ROSTactilePlugin()
    {
      this->KM_term->~KMterm(); // does this work?
      this->KM_dataPts->~KMdata();
     
      delete this->KM_term;
      delete this->KM_dataPts;
      this->node->shutdown();
      delete this->node;
    }

    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {     
      // Start up ROS
      if (!ros::isInitialized()){
        std::string name = "ros_pressuremat_plugin_node";
        int argc = 0;
        ros::init(argc, NULL, name);
      }
      else{
        ROS_WARN("ROS Tactile Plugin>> Something other than this ros_world plugin started ros::init(...), clock may not be published properly.");
      }


      // Get the parent sensor.
      this->parentSensor =
        boost::shared_dynamic_cast<sensors::ContactSensor>(_sensor);

      // Make sure the parent sensor is valid.
      if (!this->parentSensor)
        { 
          gzerr << "ROSTactilePlugin requires a ContactSensor.\n";
          return;
        } 

      // Get the parent world.
      std::string worldName = this->parentSensor->GetWorldName();
      this->parentWorld = physics::get_world(worldName);

      // ROS Nodehandle
      this->node = new ros::NodeHandle("ROSTactileNode");

      // ROS Topic Base Name
      std::string topicName = "/";
      topicName.append(this->parentSensor->GetName());
      topicName.append("/taxels/forces");

      // ROS Publisher
      this->pub = this->node->advertise<hrl_haptic_manipulation_in_clutter_msgs::TaxelArray>(topicName, 1);

      // Frame Initialize
      // get the contact pose reference frame name
      this->frame_name_ = "torso_lift_link";

      // preset myFrame to NULL, will search for the body with matching name in UpdateChild()
      // since most bodies are constructed on the fly
      //this->refFrame = NULL;

      // ROS Taxel Array
      this->frame_id.append(this->parentSensor->GetParentName());
      this->frame_id = this->frame_id.substr(this->frame_id.rfind(":")+1,this->frame_id.length());
      this->taxel.header.frame_id = "torso_lift_link";//"this->frame_id; //this->parentSensor->GetParentName();
      this->taxel.sensor_type     = "force";

      // Connect to the sensor update event.
      this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&ROSTactilePlugin::OnUpdate, this));

      // Make sure the parent sensor is active.
      this->parentSensor->SetActive(true);      
    }


    // Called by the world update start event
    public: void OnUpdate()
    {
      // mutex
      boost::mutex::scoped_lock lock(this->mutex);
      //boost::mutex::scoped_lock sclock(*this->parentSensor->GetMRMutex());

      // Get Torso Lift Link Frame
      /// if frameName specified is "world", "/map" or "map" report back inertial values in the gazebo world
      if (this->frame_name_ != "world" && this->frame_name_ != "/map" && this->frame_name_ != "map" ){        
        // lock in case a model is being spawned (?)
        //boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
        
        // look through all models in the world, search for body name that matches frameName
        std::vector<physics::ModelPtr> all_models = this->parentWorld->GetModels();
        for (std::vector<physics::ModelPtr>::iterator iter = all_models.begin(); iter != all_models.end(); iter++){
          if (*iter) this->refFrame = (*iter)->GetLink(this->frame_name_);
          if (this->refFrame) break;
        }
        
        // not found
        if (this->refFrame == NULL){
          ROS_DEBUG("ros_pressuremat_plugin: frameName: %s does not exist yet, will not publish\n",this->frame_name_.c_str());
          return;
        }
      }

      // Get all the contacts.
      msgs::Contacts contacts;
      contacts = this->parentSensor->GetContacts();
            
      // Set simulation time to taxel msg header
      this->taxel.header.stamp = ros::Time(this->parentWorld->GetSimTime().Double());
      
      // Set data points for KM
      this->KM_nPts    = contacts.contact_size();
      // Number of Available Contacts Index
      int KM_Avlb_nPts = 0;

      // Plugin's relative speed with respect to simulator
      double plugin_speed = 0.01;
      
      // Only if there is at least one contact,
      if (this->KM_nPts > 0){
        // Init pre_clustered contacts vectors
        math::Vector3 pre_contacts_position[this->KM_nPts];
        math::Vector3 pre_contacts_normal[this->KM_nPts];
        math::Vector3 pre_contacts_force[this->KM_nPts];
        
        // Init clustered contacts vectors
        math::Vector3 contacts_position[this->KM_nPts];
        math::Vector3 contacts_normal[this->KM_nPts];
        math::Vector3 contacts_force[this->KM_nPts];
        
        // get reference frame (body(link)) pose and subtract from it to get 
        // relative force, torque, position and normal vectors
        math::Pose frame_pose;
        math::Quaternion frame_rot;
        math::Vector3 frame_pos;
        
        
        // Main loop
        for (unsigned int i = 0; i < this->KM_nPts; ++i){          
          // Get reference frame (torso)
          if (this->refFrame){
            frame_pose = this->refFrame->GetWorldPose();//-this->myBody->GetCoMPose();
            frame_pos = frame_pose.pos;
            frame_rot = frame_pose.rot;
          }
          else{
            gzdbg << "No reference frame!!!\n";
            // no specific frames specified, use identity pose, keeping 
            // relative frame at inertial origin
            frame_pos = math::Vector3(0,0,0);
            frame_rot = math::Quaternion(1,0,0,0); // gazebo u,x,y,z == identity
            frame_pose = math::Pose(frame_pos, frame_rot);
          }
          
          // Init average contact vectors
          math::Vector3 contact_position_avg = math::Vector3(0,0,0);
          math::Vector3 contact_normal_avg   = math::Vector3(0,0,0);     
          math::Vector3 contact_force_sum    = math::Vector3(0,0,0);
          
          int nPts = 0;
          
          // Sub loop to get contact information from one contact object
          for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j){

            // Force is not normal-force. Thus, it should be projected to normal vector, since 
            // real robot only can sense normal-forces.
            // RVIZ : only handle normal force
            // MPC  : automatically project force to normal vector

            // HACK... force direction is not perfect. So, I used absolute force with normal vector after this. 
            std::string body_name = contacts.contact(i).wrench(j).body_1_name();
            math::Vector3 contact_force;

            if (body_name.find(this->frame_id) != std::string::npos){                        
              contact_force = msgs::Convert(contacts.contact(i).wrench(j).body_2_force());
            }
            else{
              contact_force = msgs::Convert(contacts.contact(i).wrench(j).body_1_force());
            }

            // Count only available forces
            // HACK: exclude too huge force by Daehyung 20130729
            if (contact_force.GetSquaredLength() > 0.0001 && contact_force.GetSquaredLength() < 100000.){
                contact_force_sum += frame_rot.RotateVectorReverse(contact_force);
                nPts++;
            }
            else{
                continue;
            }

            // Contact position
            math::Vector3 contact_position = msgs::Convert(contacts.contact(i).position(j));
            contact_position = contact_position - frame_pos;
            contact_position_avg += frame_rot.RotateVectorReverse(contact_position);

            // rotate normal into user specified frame. 
            // frame_rot is identity if world is used.
            math::Vector3 contact_normal = msgs::Convert(contacts.contact(i).normal(j));
            std::string collisionFrame1 = contacts.contact(i).collision1();
            collisionFrame1 = collisionFrame1.substr(collisionFrame1.rfind(":"),collisionFrame1.length());
            if (collisionFrame1.find(this->frame_id) != std::string::npos){
              contact_normal_avg -= frame_rot.RotateVectorReverse(contact_normal);             
            }
            else{
              contact_normal_avg += frame_rot.RotateVectorReverse(contact_normal);             
            }
          }
          
          if (nPts > 0){
            contact_position_avg /= (double)nPts;
            contact_normal_avg.Normalize();
            //contact_force_avg /= (double)nPts;
            
            double contact_normal_force_mag = contact_normal_avg.Dot(contact_force_sum); // projection
            contact_force_sum = contact_normal_avg*fabs(contact_normal_force_mag); // normal_force
            
            // Store contacts data which can be clustered after this.
            pre_contacts_position[KM_Avlb_nPts].x = contact_position_avg.x;
            pre_contacts_position[KM_Avlb_nPts].y = contact_position_avg.y;
            pre_contacts_position[KM_Avlb_nPts].z = contact_position_avg.z;
            
            pre_contacts_normal[KM_Avlb_nPts].x = contact_normal_avg.x;
            pre_contacts_normal[KM_Avlb_nPts].y = contact_normal_avg.y;
            pre_contacts_normal[KM_Avlb_nPts].z = contact_normal_avg.z;
            
            pre_contacts_force[KM_Avlb_nPts].x = contact_force_sum.x;
            pre_contacts_force[KM_Avlb_nPts].y = contact_force_sum.y;
            pre_contacts_force[KM_Avlb_nPts].z = contact_force_sum.z;

            KM_Avlb_nPts++;
          }
        }
    
            
        // set taxel array message
        for (unsigned int i = 0; i < KM_Avlb_nPts; i++){

          if (fabs(pre_contacts_force[i].x) + fabs(pre_contacts_force[i].y) + fabs(pre_contacts_force[i].z) > 0){


            // set taxel array message
            this->taxel.centers_x.push_back(pre_contacts_position[i].x);
            this->taxel.centers_y.push_back(pre_contacts_position[i].y);
            this->taxel.centers_z.push_back(pre_contacts_position[i].z);
            
            this->taxel.normals_x.push_back(pre_contacts_normal[i].x);
            this->taxel.normals_y.push_back(pre_contacts_normal[i].y);
            this->taxel.normals_z.push_back(pre_contacts_normal[i].z);
            
            this->taxel.values_x.push_back(pre_contacts_force[i].x * plugin_speed);
            this->taxel.values_y.push_back(pre_contacts_force[i].y * plugin_speed);
            this->taxel.values_z.push_back(pre_contacts_force[i].z * plugin_speed);
            this->taxel.link_names.push_back(this->frame_id);              
          }
        }
      }
        
      this->pub.publish(taxel);
      
      // Clear taxel array
      this->taxel.centers_x.clear();
      this->taxel.centers_y.clear();
      this->taxel.centers_z.clear();
      this->taxel.normals_x.clear();
      this->taxel.normals_y.clear();
      this->taxel.normals_z.clear();
      this->taxel.values_x.clear();
      this->taxel.values_y.clear();
      this->taxel.values_z.clear();
      this->taxel.link_names.clear();
      
      ros::spinOnce();
    }

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    // ROS Publisher
    ros::Publisher pub;

    // ROS TaxelArray
    hrl_haptic_manipulation_in_clutter_msgs::TaxelArray taxel;

    // Frame conversion
    private: std::string frame_id;
    private: std::string frame_name_;
    
    // Sensor reference frame
    private: physics::LinkPtr refFrame;

    // World
    private: physics::WorldPtr parentWorld;

    /// Mutex to protect updates.
    private: boost::mutex mutex;

    // K-means clustring
    private: KMterm *KM_term;
    private: int KM_k; 
    private: int KM_dim;
    private: int KM_nPts;
    private: KMdata *KM_dataPts;
    //private: KMdataArray *KM_dataArray;
    // math::Vector3 *positionPtr;
    // math::Vector3 *normalPtr;
    // math::Vector3 *forcePtr;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(ROSTactilePlugin)
}


