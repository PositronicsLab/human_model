/*****************************************************************************
 * Simple controller to stabalize position of the robot's end-effector that is attached to a mass
 ****************************************************************************/

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "pd_controller.h"
#include <gazebo/physics/JointController.hh>

using namespace std;
using namespace gazebo::physics;

void init_plugin(){
}

static const double KP = 1;
static const double KD = 0.1;
static const string TARGET_JOINT_NAME = "r_shoulder_lift_joint";
static const unsigned int JOINT_AXIS_INDEX = 0;

namespace gazebo {
   class ControllerPlugin : public ModelPlugin
   {
     private: physics::ModelPtr model;
     private: event::ConnectionPtr connection;
     private: boost::shared_ptr<PDController> pd;
     private: math::Angle targetAngle;
     private: double totalEffort;

     public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
     {
       cout << "Initializing plugin" << endl;

       this->model = _parent;
       totalEffort = 0.0;
       
       connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ControllerPlugin::updateController, this));

       targetAngle = this->model->GetJoint(TARGET_JOINT_NAME)->GetAngle(JOINT_AXIS_INDEX);
       cout << "Target angle: " << *targetAngle << endl;
       JointControllerPtr jc = model->GetJointController();
       jc->SetJointPosition(TARGET_JOINT_NAME, 0.0, JOINT_AXIS_INDEX);
       pd = boost::shared_ptr<PDController>(new PDController(TARGET_JOINT_NAME, model->GetWorld()->GetSimTime().Double(), KD, KP));
     }
    
     private: void updateController(){
       cout << "Starting plugin update" << endl;
       common::Time currTime = this->model->GetWorld()->GetSimTime();

       physics::JointPtr joint = this->model->GetJoint(TARGET_JOINT_NAME);
       math::Angle currAngle = joint->GetAngle(JOINT_AXIS_INDEX);
       cout << "Current angle: " << currAngle << endl;
       double effort = -1.0 * pd->update(*currAngle, *targetAngle, currTime.Double());
       totalEffort += effort;
       cout << "Current Effort: " << joint->GetForce(JOINT_AXIS_INDEX) << " Deleta Effort: " << effort << " Total Effort: " << totalEffort << endl;
       // Max force is 30
       // joint->SetForce(JOINT_AXIS_INDEX, totalEffort);
       cout << "Ending plugin update" << endl; 
   }
     };

   // Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin)
 }
