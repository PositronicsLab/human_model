/*****************************************************************************
 * Simple controller to stabalize position of the robot's end-effector that is attached to a mass
 ****************************************************************************/

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/common/PID.hh>
#include <math.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/math/constants/constants.hpp>

using namespace std;
using namespace gazebo::physics;

#define PRINT_DEBUG 0

namespace gazebo {
   // Calculated via ZN with Ku = 0.05 and Tu = 0.4
   // Classic PID rule
   static const double KP = 0.03;
   static const double KD = 0.02525;
   static const double KI = 0.02475247524752;
   static const double IMAX = 10.0;
   static const double IMIN = 0.0;

   class StableControllerPlugin : public ModelPlugin
   {
     private: physics::ModelPtr model;
     private: event::ConnectionPtr connection;

     // One per joint
     private: std::vector<boost::shared_ptr<common::PID> > jointPIDs;
     private: std::vector<math::Angle> targetAngles;
     private: std::vector<physics::JointPtr> joints;
     private: std::vector<double> totalForce;
     private: common::Time prevUpdateTime;

     public: StableControllerPlugin() : ModelPlugin() {
#if(PRINT_DEBUG)
        cout << "Constructing the controller plugin" << std::endl;
#endif
     }

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
#if(PRINT_DEBUG)
              cout << "Target angle set to " << joint->GetAngle(j) << " for joint: " << jointName << endl;
#endif
              if (joint->GetEffortLimit(j) != -1) {
                jointPIDs.push_back(boost::shared_ptr<common::PID>(new common::PID(KP, KI, KD, IMAX, IMIN, joint->GetEffortLimit(0), -joint->GetEffortLimit(0))));
              } else {
#if(PRINT_DEBUG)
                 cout << "Discovered unlimited force joint" << endl;
#endif
                jointPIDs.push_back(boost::shared_ptr<common::PID>(new common::PID(KP, KI, KD, IMAX, IMIN)));
              }
           }
         }    
       }
       this->model = _parent;
       totalForce.resize(jointPIDs.size());
       connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&StableControllerPlugin::updateController, this));
     }
    

     // Inspired by: http://answers.gazebosim.org/question/2341/set-and-get-position-of-gazebo-model-using-ros/
     private: void updateController(){
       // compute the steptime for the PID
       common::Time currTime = this->model->GetWorld()->GetSimTime();
       common::Time stepTime = currTime - this->prevUpdateTime;
       this->prevUpdateTime = currTime;
#if(PRINT_DEBUG)
        cout << "Step time: " << stepTime << endl;
#endif
       int index = 0;
       for(unsigned int i = 0; i < joints.size(); ++i){
         const physics::JointPtr joint = joints[i];
         for(unsigned int j = 0; j < joint->GetAngleCount(); ++j){
           // get the current position of the joint and the target position, 
           double posTarget = *targetAngles[index];
           double posCurr = joint->GetAngle(j).Radian();

          // calculate the error between the current position and the target one
          double posErr = posCurr - posTarget;
#if 0
          if (boost::algorithm::ends_with(joint->GetName(), "roll_joint")) {
#if(PRINT_DEBUG)
             cout << "Initial error for continuous joint is: " << posErr << endl;
#endif
            if (fabs(posErr) > boost::math::constants::pi<double>()) {
               if (posErr < 0) {
                  posErr = 2 * boost::math::constants::pi<double>() + posErr;
               }
               else {
                  posErr = posErr - 2 * boost::math::constants::pi<double>();
               }
            }
#if(PRINT_DEBUG)
             cout << "Correcting error for continuous joint to: " << posErr << endl;
#endif
          }
#endif
#if(PRINT_DEBUG)
            cout << "Error for joint " << joints[i]->GetName() << " with current value: " << posCurr << " is: " << posErr << endl;
#endif
          // compute the effort via the PID, which you will apply on the joint
          double incrementalEffort = jointPIDs[index]->Update(posErr, stepTime);
#if(PRINT_DEBUG)
            cout << "Adding incremental effort " << incrementalEffort << " to joint " << joints[i]->GetName() << endl;
#endif
            assert(!isnan(incrementalEffort));
            totalForce[index] += incrementalEffort;
#if(PRINT_DEBUG)
  cout << "Applying " << totalForce[index] << " to joint " << joints[i]->GetName() << endl;
#endif
          // apply the force on the joint
          joints[i]->SetForce(j, totalForce[index]);
          index++;
       }
     }     
   }
   ~StableControllerPlugin(){
     #if(PRINT_DEBUG)
     cout << "Destroying stable joint controller plugin" << endl;
     #endif
     event::Events::DisconnectWorldUpdateBegin(this->connection);
   }
  };

   // Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(StableControllerPlugin);
 }
