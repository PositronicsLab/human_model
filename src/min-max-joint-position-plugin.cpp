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

  class MinMaxJointPositionPlugin : public ModelPlugin {
    private: unsigned int mode;
    private: physics::ModelPtr model;
    private: unsigned int step;
    private: event::ConnectionPtr connection;
    private: physics::WorldPtr world;

    public: MinMaxJointPositionPlugin() : ModelPlugin() {
      mode = 0;
      #if(PRINT_DEBUG)
      std::cout << "Constructing the min max joint position plugin" << std::endl;
      #endif
    }
    
    
    private: void setZeroJointAngles() {
        #if (PRINT_DEBUG)
        cout << "Setting zero angles" << endl;
        #endif
        physics::Joint_V joints = model->GetJoints();
        for(unsigned int i = 0; i < joints.size(); ++i){
          for(unsigned int j = 0; j < joints[i]->GetAngleCount(); ++j){
            #if (PRINT_DEBUG)
            cout << "Setting " << joints[i]->GetName() << ": " << 0 << endl;
            #endif
            bool success = joints[i]->SetPosition(j, 0.0);
            if(!success){
              cout << "Setting joint position failed" << endl;
            }
          }
        }
    }
    
    private: void setMaxJointAngles() {
        #if (PRINT_DEBUG)
        cout << "Setting max angles" << endl;
        #endif
        physics::Joint_V joints = model->GetJoints();
        for(unsigned int i = 0; i < joints.size(); ++i){
          for(unsigned int j = 0; j < joints[i]->GetAngleCount(); ++j){
            #if (PRINT_DEBUG)
            cout << "Setting " << joints[i]->GetName() << ": " << joints[i]->GetUpperLimit(j).Radian() << endl;
            #endif
            bool success = joints[i]->SetPosition(j, joints[i]->GetUpperLimit(j).Radian());
            if(!success){
              cout << "Setting joint position failed" << endl;
            }
          }
        }
    }
    
    private: void setMinJointAngles() {
        #if (PRINT_DEBUG)
        cout << "Setting min angles" << endl;
        #endif
        physics::Joint_V joints = model->GetJoints();
        for(unsigned int i = 0; i < joints.size(); ++i){
          for(unsigned int j = 0; j < joints[i]->GetAngleCount(); ++j){
            #if (PRINT_DEBUG)
            cout << "Setting " << joints[i]->GetName() << ": " << joints[i]->GetLowerLimit(j).Radian() << endl;
            #endif
            bool success = joints[i]->SetPosition(j, joints[i]->GetLowerLimit(j).Radian());
            if(!success){
              cout << "Setting joint position failed" << endl;
            }
          }
        }
    }
    
    private: void worldUpdate(){
      if(model->GetWorld()->GetSimTime() > 0.002 && mode == 2){
        setMaxJointAngles();
        mode = 3;
      }
      if(model->GetWorld()->GetSimTime() > 0.001 && mode == 1){
        setMinJointAngles();
        mode = 2;
      }
    }
    
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      model = _model;
      model->SetStatic(false);
      #if(PRINT_DEBUG)
      std::cout << "Loading min max joint position plugin" << std::endl;
      #endif
     
      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MinMaxJointPositionPlugin::worldUpdate, this));

      setZeroJointAngles();
      mode = 1;
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(MinMaxJointPositionPlugin)
}
