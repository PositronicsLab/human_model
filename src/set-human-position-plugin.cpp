#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <stdio.h>
#include <iostream>
#include <boost/math/constants/constants.hpp>
using namespace std;

#define PRINT_DEBUG 1

namespace gazebo {

  class SetHumanPositionPlugin : public ModelPlugin {
    private: physics::ModelPtr model;
    private: physics::WorldPtr world;

    public: SetHumanPositionPlugin() : ModelPlugin() {
      
      #if(PRINT_DEBUG)
      std::cout << "Constructing the set human position plugin" << std::endl;
      #endif
    }
    
    
    private: void setAngles() {
        #if (PRINT_DEBUG)
        cout << "*** Setting angles ***" << endl;
        #endif

        model->GetJoint("neck")->SetPosition(0, 0.2);
        model->GetJoint("left_shoulder")->SetPosition(0, boost::math::constants::pi<double>());
        model->GetJoint("left_shoulder")->SetPosition(1, -0.2);
        model->GetJoint("right_shoulder")->SetPosition(0, boost::math::constants::pi<double>());
        model->GetJoint("right_shoulder")->SetPosition(1, 0.2);
        model->GetJoint("left_elbow")->SetPosition(0, boost::math::constants::pi<double>() / 2.0);
        model->GetJoint("right_elbow")->SetPosition(0, boost::math::constants::pi<double>() / 2.0);
        model->GetJoint("left_wrist")->SetPosition(0, 0.2);
        model->GetJoint("left_wrist")->SetPosition(1, 0.2);
        model->GetJoint("right_wrist")->SetPosition(0, 0.2);
        model->GetJoint("right_wrist")->SetPosition(1, -0.2);

        model->GetJoint("left_hip")->SetPosition(0, -boost::math::constants::pi<double>() / 8.0);
        model->GetJoint("left_hip")->SetPosition(1, boost::math::constants::pi<double>() / 16.0);
        model->GetJoint("right_hip")->SetPosition(0, -boost::math::constants::pi<double>() / 8.0);
        model->GetJoint("right_hip")->SetPosition(1, -boost::math::constants::pi<double>() / 16.0);

        model->GetJoint("left_knee")->SetPosition(0, boost::math::constants::pi<double>() / 4.0);
        model->GetJoint("right_knee")->SetPosition(0, boost::math::constants::pi<double>() / 4.0);

       // Ankle positions are fine
    }
    
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      model = _model;
      model->SetStatic(false);
      #if(PRINT_DEBUG)
      std::cout << "Loading set human position plugin" << std::endl;
      #endif

      setAngles();
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(SetHumanPositionPlugin
)
}
