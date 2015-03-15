#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/physics/joint.hh>
#include <gazebo/physics/world.hh>
#include <gazebo/physics/link.hh>
#include <stdio.h>
#include <iostream>
#include <boost/math/constants/constants.hpp>
using namespace std;

#define PRINT_DEBUG 1

namespace gazebo {

   static const std::string joints[] = {
      "r_shoulder_lift_joint",
      "r_shoulder_pan_joint",
      "r_upper_arm_roll_joint",
      "r_elbow_flex_joint",
      "r_forearm_roll_joint",
      "r_wrist_flex_joint",
      "r_wrist_roll_joint",
      "l_shoulder_lift_joint",
      "l_shoulder_pan_joint",
      "l_upper_arm_roll_joint",
      "l_elbow_flex_joint",
      "l_forearm_roll_joint",
      "l_wrist_flex_joint",
      "l_wrist_roll_joint",
   };

  class ArmCalibrationPlugin : public ModelPlugin {


    private: physics::ModelPtr model;
    private: physics::WorldPtr world;

    public: ArmCalibrationPlugin() : ModelPlugin() {
      
      #if(PRINT_DEBUG)
      std::cout << "Constructing the arm calibration plugin" << std::endl;
      #endif
    }
    
    
    private: void setZeroJointAngles() {
        #if (PRINT_DEBUG)
        cout << "*** Setting zero angles ***" << endl;
        #endif
       for(unsigned int i = 0; i < boost::size(joints); ++i){
          physics::JointPtr joint = model->GetJoint(joints[i]);
          for(unsigned int j = 0; j < joint->GetAngleCount(); ++j){
            #if (PRINT_DEBUG)
            cout << "Setting " << joint->GetName() << ": " << 0 << endl;
            #endif
            bool success = joint->SetPosition(j, 0.0);
            if(!success){
              cout << "Setting joint position failed" << endl;
            }
          }
        }
    }
    
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      model = _model;
      #if(PRINT_DEBUG)
      std::cout << "Loading arm calibration plugin" << std::endl;
      #endif

      setZeroJointAngles();
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(ArmCalibrationPlugin)
}
