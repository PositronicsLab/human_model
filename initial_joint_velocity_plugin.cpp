#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/physics/joint.hh>
#include <stdio.h>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <iostream>

namespace gazebo {
  class InitialJointVelocityPlugin : public WorldPlugin {
    public: InitialJointVelocityPlugin() : WorldPlugin() {
      std::cout << "Constructing the initial velocity plugin" << std::endl;
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      std::cout << "Loading the initial velocity plugin" << std::endl;

      // Create a random number generator. Note that this has a minute bias that it will
      // not generate 1.0
      boost::mt19937 rng;
      boost::uniform_real<float> u(-1.0f, 1.0f);
      boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > gen(rng, u);
      
      // Get the human
      physics::ModelPtr human = _world->GetModel("human");
      assert(human != NULL);

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
          joint->SetVelocity(j, random);
        }
      }
    };

  };
  GZ_REGISTER_WORLD_PLUGIN(InitialJointVelocityPlugin)
}
