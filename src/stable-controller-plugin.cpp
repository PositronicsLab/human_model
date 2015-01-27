/*****************************************************************************
 * Simple controller to stabalize position of the robot's end-effector that is attached to a mass
 ****************************************************************************/

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/common/PID.hh>

using namespace std;
using namespace gazebo::physics;

#define PRINT_DEBUG 0

namespace gazebo {
   class ControllerPlugin : public ModelPlugin
   {
     private: physics::ModelPtr model;
     private: event::ConnectionPtr connection;

     // One per joint
     private: std::vector<boost::shared_ptr<common::PID> > jointPIDs;
     private: std::vector<math::Angle> targetAngles;
     private: std::vector<double> totalEfforts;
     private: std::vector<physics::JointPtr> joints;
     
     private: common::Time prevUpdateTime;

     public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
     {
       #if(PRINT_DEBUG)
       cout << "Initializing plugin stable controller plugin" << endl;
       #endif
       
       // Iterate over the element children to find the joints to control
       if(!_sdf->HasElement("controlled-joint")){
         cerr << "Controlled joint not specified" << endl;
         return;
       }

       for(sdf::ElementPtr child = _sdf->GetElement("controlled-joint"); child != NULL; child = child->GetNextElement()){
         if(child->GetName() == "controlled-joint"){
           const std::string jointName = child->GetValueString();
           #if(PRINT_DEBUG)
           cout << "Adding controlled joint: " << jointName << endl;
           #endif
           const physics::JointPtr joint = _parent->GetJoint(jointName);

           if(joint == NULL){
             cerr << "Could not find joint: " << jointName << endl;
             continue;
           }

           joints.push_back(joint);
           for(unsigned int j = 0; j < joint->GetAngleCount(); ++j){
             targetAngles.push_back(joint->GetAngle(j));
             jointPIDs.push_back(boost::shared_ptr<common::PID>(new common::PID(2.0, 0.0, 1.0, 10)));
           }
         }    
       }
       totalEfforts.resize(jointPIDs.size());
       this->model = _parent;
       
       connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ControllerPlugin::updateController, this));
     }
    

     // Inspired by: http://answers.gazebosim.org/question/2341/set-and-get-position-of-gazebo-model-using-ros/
     private: void updateController(){

       // compute the steptime for the PID
       common::Time currTime = this->model->GetWorld()->GetSimTime();
       common::Time stepTime = currTime - this->prevUpdateTime;
       this->prevUpdateTime = currTime;

       int index = 0;
       for(unsigned int i = 0; i < joints.size(); ++i){
         const physics::JointPtr joint = joints[i];
         for(unsigned int j = 0; j < joint->GetAngleCount(); ++j){
           // get the current position of the joint and the target position, 
           double posTarget = *targetAngles[index];
           double posCurr = joint->GetAngle(j).Radian();

          // calculate the error between the current position and the target one
          double posErr = posCurr - posTarget;

          // compute the effort via the PID, which you will apply on the joint
          totalEfforts[index] += jointPIDs[index]->Update(posErr, stepTime);

          // apply the force on the joint
          joints[i]->SetForce(j, totalEfforts[index]);
          index++;
       }
     }     
   }
   ~ControllerPlugin(){
     #if(PRINT_DEBUG)
     cout << "Destroying stable joint controller plugin" << endl;
     #endif
     event::Events::DisconnectWorldUpdateBegin(this->connection);
   }
  };

   // Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin)
 }
