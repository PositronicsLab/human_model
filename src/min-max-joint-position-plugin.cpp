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

  class MinMaxJointPositionPlugin : public ModelPlugin {
    private: unsigned int mode;
    private: unsigned int currentJoint;
    private: unsigned int currentSubjoint;
    
    private: physics::ModelPtr model;
    private: unsigned int step;
    private: event::ConnectionPtr connection;
    private: physics::WorldPtr world;

    public: MinMaxJointPositionPlugin() : ModelPlugin() {
      mode = 0;
      currentJoint = 0;
      currentSubjoint = 0;
      
      #if(PRINT_DEBUG)
      std::cout << "Constructing the min max joint position plugin" << std::endl;
      #endif
    }
    
    
    private: void setZeroJointAngles() {
        #if (PRINT_DEBUG)
        cout << "*** Setting zero angles ***" << endl;
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
    
    private: void setJointAngle() {
        physics::Joint_V joints = model->GetJoints();
        
        if(currentJoint >= joints.size()){
          return;
        }
        
        if(currentSubjoint >= joints[currentJoint]->GetAngleCount()){
          currentSubjoint = 0;
          currentJoint++;
        }
        
        if(currentJoint >= joints.size()){
          return;
        }
        
        physics::JointPtr joint = joints[currentJoint];
        double desiredAngle = 0;
        if(mode == 0){
          desiredAngle = joints[currentJoint]->GetLowerLimit(currentSubjoint).Radian();
          #if (PRINT_DEBUG)
          cout << "*** Setting joint " << joint->GetName() << " to min " << desiredAngle << " ***" << endl;
          #endif
          mode = 1;
        }
        else if(mode == 1){
          desiredAngle = joints[currentJoint]->GetUpperLimit(currentSubjoint).Radian();
          #if (PRINT_DEBUG)
          cout << "*** Setting joint " << joint->GetName() << " to max " << desiredAngle << " ***" << endl;
          #endif

          mode = 2;
        }
        else {
            #if (PRINT_DEBUG)
            cout << "*** Resetting joint " << joint->GetName() << endl;
            #endif
            desiredAngle = 0;
            mode = 3;
        }
          
        
        bool success = joints[currentJoint]->SetPosition(currentSubjoint, desiredAngle);
        if(!success){
          cerr << "Setting joint position failed" << endl;
        }
        
        if(mode == 3){
            currentSubjoint++;
            mode = 0;
        }
    }
    
    private: void worldUpdate(){
      setJointAngle();
    }
    
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      model = _model;
      model->SetStatic(false);
      #if(PRINT_DEBUG)
      std::cout << "Loading min max joint position plugin" << std::endl;
      #endif
     
      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MinMaxJointPositionPlugin::worldUpdate, this));

      setZeroJointAngles();
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(MinMaxJointPositionPlugin)
}
