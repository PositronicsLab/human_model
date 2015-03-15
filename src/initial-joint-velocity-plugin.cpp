#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/physics/joint.hh>
#include <gazebo/physics/link.hh>
#include <gazebo/physics/world.hh>
#include <stdio.h>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <iostream>
#include <iostream>
#include <fstream>

using namespace std;

#define USE_FIXED_SEED 1
#define PRINT_VELOCITIES 1
#define PRINT_DEBUG 0

namespace gazebo {

  static const std::string joints[] = {
        "left_ankle",
        "right_ankle",
        "left_knee",
        "right_knee",
        "left_hip",
        "right_hip",
        "left_shoulder",
        "right_shoulder",
        "left_elbow",
        "right_elbow",
        "left_wrist",
        "right_wrist"
      };
    
  class InitialJointVelocityPlugin : public ModelPlugin {
    public: InitialJointVelocityPlugin() : ModelPlugin() {
      #if(PRINT_DEBUG)
      std::cout << "Constructing the initial velocity plugin" << std::endl;
      #endif
    }

    private: void writeHeader(ofstream& file){
      for(unsigned int i = 0; i < boost::size(joints); ++i){
        file << joints[i] << ", ";
      }
      file << "Trunk Linear X, Trunk Linear Y, Trunk Linear Z, Trunk Angular X, Trunk Angular Y, Trunk Angular Z";
      file << endl;
    }
    
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      #if(PRINT_DEBUG)
      std::cout << "Loading the initial velocity plugin" << std::endl;
      #endif
      // Get the name of the folder to store the result in
      const char* resultsFolder = std::getenv("RESULTS_FOLDER");
      if(resultsFolder == nullptr){
        cout << "Results folder not set. Using current directory." << endl;
        resultsFolder = "./";
      }
      
      // Open a file to store the random values
      ofstream csvFile;
      
      const string resultsFileName = string(resultsFolder) + "/" + "joint_velocities.csv";
      bool exists = boost::filesystem::exists(resultsFileName);
      csvFile.open(resultsFileName, ios::out | ios::app);
      assert(csvFile.is_open());
      
      if(!exists){
        writeHeader(csvFile);
      }

      // Create a random number generator. Note that this has a minute bias that it will
      // not generate 1.0
      boost::mt19937 rng;
      #if(!USE_FIXED_SEED)
        rng.seed(static_cast<unsigned int>(std::time(nullptr)));
      #else
        const char* scenarioNumber = std::getenv("i");
        if(scenarioNumber != nullptr){
            rng.seed(boost::lexical_cast<unsigned int>(scenarioNumber));
        }
        else {
          cout << "No scenario number set. Using 0" << endl;
          rng.seed(0);
        }
      #endif
      boost::uniform_real<float> u(-1.0f, 1.0f);
      boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > gen(rng, u);

      // Now loop over joints and set an initial velocity for each.
      // We specifically look for the human joints to avoid setting a velocity
      // on the robots joints
      for(unsigned int i = 0; i < boost::size(joints); ++i){
        physics::JointPtr joint = _model->GetJoint(joints[i]);
        // Set a random velocity for each axis
        for(unsigned int j = 0; j < joint->GetAngleCount(); ++j){
          float random = gen();
          #if(PRINT_VELOCITIES)
          std::cout << "Setting velocity for joint " << joints[i] << " axis number " << j << " to " << random << std::endl;
          #endif
          joint->SetMaxForce(j, 5.0);
          joint->SetVelocity(j, random);
          csvFile << random << ",";
        }
      }

       // Set the angular and linear velocity for the trunk.
       physics::LinkPtr trunk = _model->GetLink("trunk");
       math::Vector3 linear = math::Vector3(gen(), gen(), gen());
       math::Vector3 angular = math::Vector3(gen(), gen(), gen());

#if(PRINT_VELOCITIES)
       cout << "Setting linear velocity for trunk to: " << linear.x << ", " << linear.y << ", " << linear.z << endl;
       cout << "Setting angular velocity for trunk to: " << angular.x << ", " << angular.y << ", " << angular.z << endl;
#endif
       csvFile << linear.x << ", " << linear.y << ", " << linear.z << ", " << angular.x << ", " << angular.y << ", " << angular.z;
       trunk->SetLinearVel(linear);
       trunk->SetAngularVel(angular);
      csvFile << endl;
      csvFile.close();
    }

  };
  GZ_REGISTER_MODEL_PLUGIN(InitialJointVelocityPlugin)
}
