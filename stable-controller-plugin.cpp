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

void init_plugin(){
}

static const string TARGET_JOINT_NAME = "r_shoulder_lift_joint";
static const unsigned int JOINT_AXIS_INDEX = 0;

namespace gazebo {
   class ControllerPlugin : public ModelPlugin
   {
     private: physics::ModelPtr model;
     private: event::ConnectionPtr connection;
     private: boost::shared_ptr<common::PID> jointPID;
     private: math::Angle targetAngle;
     private: double totalEffort;
     private: physics::JointPtr joint;
     private: common::Time prevUpdateTime;

     public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
     {
       cout << "Initializing plugin" << endl;

       this->model = _parent;
       totalEffort = 0.0;
       
       connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ControllerPlugin::updateController, this));
       joint = this->model->GetJoint(TARGET_JOINT_NAME);
       targetAngle = joint->GetAngle(JOINT_AXIS_INDEX);
       cout << "Target angle: " << *targetAngle << endl;
       jointPID = boost::shared_ptr<common::PID>(new common::PID(1000000.0, 0.0, 1.0, 10)); /* TODO: Determine if we should turn limits on */
     }
    

     // Inspired by: http://answers.gazebosim.org/question/2341/set-and-get-position-of-gazebo-model-using-ros/
     private: void updateController(){
       cout << "Starting plugin update" << endl;

       // compute the steptime for the PID
       common::Time currTime = this->model->GetWorld()->GetSimTime();
       common::Time stepTime = currTime - this->prevUpdateTime;
       this->prevUpdateTime = currTime;

       // set the current position of the joint, and the target position, 
       // and the maximum effort limit
       double posTarget = *targetAngle;
       double posCurr = this->joint->GetAngle(JOINT_AXIS_INDEX).Radian();
       double maxCmd = this->joint->GetEffortLimit(JOINT_AXIS_INDEX);
       cout << "Max effort: " << maxCmd << endl;

       // calculate the error between the current position and the target one
       double posErr = posCurr - posTarget;
       cout << "Pos curr: " << posCurr << " Pos target: " << posTarget << " posErr: " << posErr << endl;

       // compute the effort via the PID, which you will apply on the joint
       double effortCmd = this->jointPID->Update(posErr, stepTime);

       cout << "Executing effort: " << effortCmd << endl;
       // apply the force on the joint
       this->joint->SetForce(JOINT_AXIS_INDEX, effortCmd);       
       cout << "Ending plugin update" << endl; 
   }
     };

   // Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin)
 }
