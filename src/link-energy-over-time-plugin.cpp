#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/sensors.hh>


#include <iostream>
#include <fstream>
#include <vector>

#define PRINT_SENSORS 0
#define PRINT_DEBUG 0

using namespace std;

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

   static const double MAX_TIME = 2.0;
    
    class LinkEnergyOverTimePlugin : public ModelPlugin {

    private: gazebo::transport::NodePtr node;
    private: event::ConnectionPtr connection;
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: double averageLinkEnergy;
    private: ofstream outputCSV;

    public: LinkEnergyOverTimePlugin() : ModelPlugin() {
        #if(PRINT_DEBUG)
        cout << "Constructing the link energy over time plugin" << std::endl;
        #endif
    }
        
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      world = _model->GetWorld();
      model = _model;

      #if(PRINT_DEBUG)
      cout << "Loading the link energy over time plugin" << endl;
      #endif
      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LinkEnergyOverTimePlugin::worldUpdate, this));

      // Get the name of the folder to store the result in
      const char* resultsFolder = std::getenv("RESULTS_FOLDER");
      if(resultsFolder == nullptr){
         cout << "Results folder not set. Using current directory." << endl;
         resultsFolder = "./";
      }

      const string resultsFileName = string(resultsFolder) + "/" + "link_energy_over_time.csv";
      outputCSV.open(resultsFileName, ios::out);
      assert(outputCSV.is_open());
      writeHeader(outputCSV);

      double linkEnergy = 0;
      for (unsigned int i = 0; i < boost::size(links); ++i) {
         physics::LinkPtr link = model->GetLink(links[i]);
         linkEnergy += link->GetWorldEnergy();
      }
      outputCSV << world->GetSimTime().Float() << ", " << linkEnergy << endl;
    }
    
    private: void writeHeader(ofstream& outputCSV){
       outputCSV << "Time (s), Total Energy (J)" << endl;
    }

    private: void worldUpdate(){
#if(PRINT_DEBUG)
       cout << "Updating the world for time: " << world->GetSimTime().Float() << endl;
#endif
       for (unsigned int i = 0; i < boost::size(links); ++i) {
          const physics::LinkPtr link = model->GetLink(links[i]);
          averageLinkEnergy += link->GetWorldEnergy();
       }

       if (world->GetSimTime().nsec % (10 * 1000000) == 0) {
          outputCSV << world->GetSimTime().Float() << ", " << averageLinkEnergy / 10.0 << endl;
          averageLinkEnergy = 0;
       }

       if(world->GetSimTime().Float() >= MAX_TIME){
          #if(PRINT_DEBUG)
          cout << "Scenario completed. Updating results" << endl;
          #endif
          event::Events::DisconnectWorldUpdateBegin(this->connection);

          outputCSV << endl;
          outputCSV.close();
       }
    }
    };
    GZ_REGISTER_MODEL_PLUGIN(LinkEnergyOverTimePlugin);
}

