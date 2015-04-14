#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/world.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/sensors/sensors.hh>


#include <iostream>
#include <fstream>

#define PRINT_DEBUG 0

using namespace std;

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

    class JointForcesPlugin : public ModelPlugin {

    private: event::ConnectionPtr connection;
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: ofstream csvFile;

    public: JointForcesPlugin() : ModelPlugin() {
        #if(PRINT_DEBUG)
        cout << "Constructing the joint forces plugin" << std::endl;
        #endif
    }
        
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      #if(PRINT_DEBUG)
        cout << "Loading the joint forces plugin" << std::endl;
      #endif
      world = _model->GetWorld();
      model = _model;

      // Open a file to store the results
      const string resultsFileName = "joint_forces.csv";
      csvFile.open(resultsFileName, ios::out);
      assert(csvFile.is_open());
      writeHeader(csvFile);
      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&JointForcesPlugin::worldUpdate, this));
    }
        
    public: ~JointForcesPlugin(){
      event::Events::DisconnectWorldUpdateBegin(this->connection);
      csvFile.close();
    }

    void writeHeader(ofstream& file) {
       file << "Time, ";
       for (unsigned int i = 0; i < boost::size(joints); ++i) {
          physics::JointPtr currJoint = model->GetJoint(joints[i]);
          for (unsigned int j = 0; j < currJoint->GetAngleCount(); ++j) {
             file << joints[i] << "(" << j << "),";
          }
       }
       file << endl;
    }

    private: void worldUpdate(){
       if(this->model->GetWorld()->GetSimTime().nsec % 10000000 != 0){
          return;
       }
       
      csvFile << world->GetSimTime().Float() << ", ";
      // Iterate over model joints and print them
      for (unsigned int i = 0; i < boost::size(joints); ++i) {
        physics::JointPtr currJoint = model->GetJoint(joints[i]);
        for(unsigned int j = 0; j < currJoint->GetAngleCount(); ++j){
          csvFile << "   " << currJoint->GetForce(j) << ", ";
        }
      }
      csvFile << endl;
    }
    };
    GZ_REGISTER_MODEL_PLUGIN(JointForcesPlugin);
}

