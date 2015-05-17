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

#define PRINT_DEBUG 0

namespace gazebo {

  static const std::string links[] = {
        "trunk",
        "right_foot",
        "left_foot",
        "right_leg",
        "left_leg",
        "right_thigh",
        "left_thigh",
        "transpelvic_link",
        "clavicular_link",
        "head_neck",
        "right_upper_arm",
        "left_upper_arm",
        "right_forearm",
        "left_forearm",
        "right_hand",
        "left_hand"
      };
  static const std::string dims[] = { "x", "y", "z"};

  class InitialLinkPositionPlugin : public ModelPlugin {
    public: InitialLinkPositionPlugin() : ModelPlugin() {
      #if(PRINT_DEBUG)
      std::cout << "Constructing the initial link position plugin" << std::endl;
      #endif
    }

    private: void writeHeader(ofstream& file){
       for (unsigned int i = 0; i < boost::size(links); ++i) {
          for (unsigned int j = 0; j < boost::size(dims); ++j) {
             file << links[i] << "(" << dims[j] << "),";
          }
       }
      file << endl;
    }
    
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      #if(PRINT_DEBUG)
      std::cout << "Loading the initial link position plugin" << std::endl;
      #endif
      // Get the name of the folder to store the result in
      const char* resultsFolder = std::getenv("RESULTS_FOLDER");
      if(resultsFolder == nullptr){
        cout << "Results folder not set. Using current directory." << endl;
        resultsFolder = "./";
      }
      
      // Open a file to store the random values
      ofstream csvFile;
      
      const string resultsFileName = string(resultsFolder) + "/" + "link_positions.csv";
      bool exists = boost::filesystem::exists(resultsFileName);
      csvFile.open(resultsFileName, ios::out | ios::app);
      assert(csvFile.is_open());
      
      if(!exists){
        writeHeader(csvFile);
      }

      for (unsigned int i = 0; i < boost::size(links); ++i) {
         physics::LinkPtr link = _model->GetLink(links[i]);
         csvFile << link->GetWorldPose().pos.x << ", " << link->GetWorldPose().pos.y << ", " << link->GetWorldPose().pos.z << ",";
      }
      csvFile << endl;
      csvFile.close();
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(InitialLinkPositionPlugin)
}
