#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/physics/joint.hh>
#include <gazebo/physics/world.hh>
#include <stdio.h>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <iostream>
#include <iostream>
#include <fstream>

using namespace std;

#define USE_FIXED_SEED 1

namespace gazebo {
  class InitialJointVelocityPlugin : public WorldPlugin {
    public: InitialJointVelocityPlugin() : WorldPlugin() {
      std::cout << "Constructing the initial velocity plugin" << std::endl;
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      // Open a file to store the random values
      ofstream csvFile;
      csvFile.open("joint_velocities.csv", ios::out | ios::app);
      assert(csvFile.is_open());

      std::cout << "Loading the initial velocity plugin" << std::endl;

      // Create a random number generator. Note that this has a minute bias that it will
      // not generate 1.0
      boost::mt19937 rng;
      #if(!USE_FIXED_SEED)
      rng.seed(static_cast<unsigned int>(std::time(0)));
      #else
      rng.seed(0);
      #endif
      boost::uniform_real<float> u(-1.0f, 1.0f);
      boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > gen(rng, u);
      
      // Get the human
      physics::ModelPtr human = _world->GetModel("human");
      if(human == NULL){
        std::cout << "Human model not defined" << std::endl;
        return;
      }

      // Now loop over joints and set an initial velocity for each.
      // We specifically look for the human joints to avoid setting a velocity
      // on the robots joints
      const std::string joints[] = {
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

      for(unsigned int i = 0; i < boost::size(joints); ++i){
        physics::JointPtr joint = human->GetJoint(joints[i]);
        // Set a random velocity for each axis
        for(unsigned int j = 0; j < joint->GetAngleCount(); ++j){
          float random = gen();
          std::cout << "Setting velocity for joint " << joints[i] << " axis number " << j << " to " << random << std::endl;
          joint->SetMaxForce(j, 5.0);
          joint->SetVelocity(j, random);
          csvFile << random << ",";
        }
      }
      csvFile << endl;
      csvFile.close();
    };

  };
  GZ_REGISTER_WORLD_PLUGIN(InitialJointVelocityPlugin)
}
