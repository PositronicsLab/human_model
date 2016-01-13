#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <stdio.h>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <iostream>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

using namespace std;

#define PRINT_DEBUG 0

namespace gazebo {

  static const string links[] = {
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
  static const string dims[] = { "x", "y", "z"};

  static const string velocityDims[] = { "x", "y", "z", "r", "p", "y"};


  class InitialLinkPositionPlugin : public ModelPlugin {

   private: physics::WorldPtr world;
   private: physics::ModelPtr model;
   private: event::ConnectionPtr connection;
   private:  ofstream csvFile;

    public: InitialLinkPositionPlugin() : ModelPlugin() {
      #if(PRINT_DEBUG)
      cout << "Constructing the initial link position plugin" << endl;
      #endif
    }

    private: void writeHeader(ofstream& file){
       for (unsigned int i = 0; i < boost::size(links); ++i) {
          for (unsigned int j = 0; j < boost::size(dims); ++j) {
             file << links[i] << "(" << dims[j] << "),";
          }
       }
       for (unsigned int i = 0; i < boost::size(links); ++i) {
          for (unsigned int j = 0; j < boost::size(velocityDims); ++j) {
             file << links[i] << "(" << velocityDims[j] << ") (m/s),";
          }
       }
      file << endl;
    }
    
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      world = _model->GetWorld();
      model = _model;

      #if(PRINT_DEBUG)
      cout << "Loading the initial link position plugin" << endl;
      #endif
      // Get the name of the folder to store the result in
      const char* resultsFolder = getenv("RESULTS_FOLDER");
      if(resultsFolder == nullptr){
        cout << "Results folder not set. Using current directory." << endl;
        resultsFolder = "./";
      }
      
      // Open a file to store the random values
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

      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&InitialLinkPositionPlugin::worldUpdate, this));
    }

    private: void worldUpdate(){
       #if(PRINT_DEBUG)
       cout << "Received world update for iteration: " << world->GetIterations() << endl;
       #endif
       // Link velocity data not available until iteration 2.
       if (world->GetIterations() != 2){
          return;
       }

      // Write out velocities
       for (unsigned int i = 0; i < boost::size(links); ++i) {
          physics::LinkPtr link = model->GetLink(links[i]);
          #if(PRINT_DEBUG)
          cout << "Writing out link: " << links[i] << " with linear velocity: " << link->GetWorldLinearVel() << " and angular velocity: " << link->GetWorldAngularVel() << endl;
          #endif
          csvFile << link->GetWorldLinearVel().x << ", " << link->GetWorldLinearVel().y << ", " << link->GetWorldLinearVel().z << "," << link->GetWorldAngularVel().x << ", " << link->GetWorldAngularVel().y << ", " << link->GetWorldAngularVel().z << ",";
       }
       csvFile << endl;
       csvFile.close();
       event::Events::DisconnectWorldUpdateBegin(this->connection);
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(InitialLinkPositionPlugin)
}
