#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/world.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/sensors/sensors.hh>


#include <iostream>
#include <fstream>

#define PRINT_SENSORS 0
#define PRINT_DEBUG 0

using namespace std;

namespace gazebo {
    
    class ContactForcesPlugin : public ModelPlugin {
        
    private: std::vector<double> velocities;

    private: gazebo::transport::NodePtr node;
    private: event::ConnectionPtr connection;
    private: vector<event::ConnectionPtr> sensorConnections;
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    
    public: HeadInjuryCriterionPlugin() : ModelPlugin() {
        #if(PRINT_DEBUG)
        cout << "Constructing the HIC plugin" << std::endl;
        #endif
    }
        
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      world = _model->GetWorld();
      model = _model;
      #if(PRINT_DEBUG)
      cout << "Loading the HIC plugin" << endl;
      #endif

      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ContactForcesPlugin::worldUpdate, this));
    }
    
    private: void writeHeader(ofstream& outputCSV){
       outputCSV << "HIC, Time" << endl;
    }
    
    private: void printResults(){
            // Get the name of the folder to store the result in
            const char* resultsFolder = std::getenv("RESULTS_FOLDER");
            if(resultsFolder == nullptr){
              cout << "Results folder not set. Using current directory." << endl;
              resultsFolder = "./";
            }
        
            const string resultsFileName = string(resultsFolder) + "/" + "hic.csv";
            bool exists = boost::filesystem::exists(resultsFileName);
            ofstream outputCSV;
            outputCSV.open(resultsFileName, ios::out | ios::app);
            assert(outputCSV.is_open());
      
            if(!exists){
              writeHeader(outputCSV);
            }

            // Print the maximum HIC value and time
            outputCSV << maximumHic << ", " << maximumHicTime << endl;
            outputCSV.close();
        }
        
    private: void worldUpdate(){
        if(world->GetSimTime().Float() >= 2.0){
          #if(PRINT_DEBUG)
          cout << "Scenario completed. Updating results" << endl;
          #endif
          event::Events::DisconnectWorldUpdateBegin(this->connection);

          // Disconnect the sensors
          for(unsigned int i = 0; i < boost::size(contacts); ++i){
            sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor(world->GetName() + "::" + model->GetScopedName()
                                                       + "::" + contacts[i]);
            if(sensor == nullptr){
              cout << "Could not find sensor " << contacts[i] << endl;
              continue;
            }
            sensor->SetActive(false);
            sensor->DisconnectUpdated(sensorConnections[i]);
          }
          sensorConnections.clear();
          printResults();
          exit(0);
        }
    }
    };
    GZ_REGISTER_MODEL_PLUGIN(ContactForcesPlugin);
}

