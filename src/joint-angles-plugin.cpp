#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/world.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/sensors/sensors.hh>


#include <iostream>
#include <fstream>

#define PRINT_DEBUG 1
#define PRINT_ANGLES 1

using namespace std;

namespace gazebo {
    
    class JointAnglesPlugin : public ModelPlugin {
        
    private: event::ConnectionPtr connection;
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    
    public: JointAnglesPlugin() : ModelPlugin() {
        #if(PRINT_DEBUG)
        cout << "Constructing the joint angles plugin" << std::endl;
        #endif
    }
        
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      #if(PRINT_DEBUG)
        cout << "Loading the joint angles plugin" << std::endl;
      #endif
      world = _model->GetWorld();
      model = _model;
      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&JointAnglesPlugin::worldUpdate, this));
    }
        
    public: ~JointAnglesPlugin(){
      event::Events::DisconnectWorldUpdateBegin(this->connection);
    }
    
    private: void worldUpdate(){
    #if(PRINT_ANGLES)
      // Iterate over model joints and print them
      const physics::Joint_V joints = model->GetJoints();
      for(unsigned int i = 0; i < joints.size(); ++i){
        for(unsigned int j = 0; j < joints[i]->GetAngleCount(); ++j){
          cout << "   " << joints[i]->GetAngle(j) << endl;
        }
      }
      cout << endl;
      #endif
    }
    };
    GZ_REGISTER_MODEL_PLUGIN(JointAnglesPlugin);
}

