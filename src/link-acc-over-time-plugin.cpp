#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/world.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/sensors/sensors.hh>


#include <iostream>
#include <fstream>
#include <vector>

#define PRINT_DEBUG 0

using namespace std;

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

   static const double MAX_TIME = 2.0;
    
    class LinkAccelerationOverTimePlugin
 : public ModelPlugin {

    private: gazebo::transport::NodePtr node;
    private: event::ConnectionPtr connection;
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: physics::LinkPtr trunk;
    private: ofstream outputCSV;

    public: LinkAccelerationOverTimePlugin
() : ModelPlugin() {
#if(PRINT_DEBUG)
       cout << "Constructing the link acceleration over time plugin" << std::endl;
#endif
    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      world = _model->GetWorld();
      model = _model;
      trunk = model->GetLink("trunk");

      #if(PRINT_DEBUG)
      cout << "Loading the link acceleration over time plugin" << endl;
      #endif
      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LinkAccelerationOverTimePlugin
::worldUpdate, this));

      // Get the name of the folder to store the result in
      const char* resultsFolder = std::getenv("RESULTS_FOLDER");
      if(resultsFolder == nullptr){
         cout << "Results folder not set. Using current directory." << endl;
         resultsFolder = "./";
      }

      const string resultsFileName = string(resultsFolder) + "/" + "link_accelerations.csv";
      outputCSV.open(resultsFileName, ios::out);
      assert(outputCSV.is_open());
      writeHeader(outputCSV);
    }
    
   private: void writeHeader(ofstream& outputCSV){
      outputCSV << "Time" << ", ";
      for(unsigned int i = 0; i < boost::size(links); ++i){
         outputCSV << links[i] << "(m/s^2), ";
      }
      outputCSV << endl;
   }

    private: void worldUpdate(){
#if(PRINT_DEBUG)
       cout << "Updating the world for time: " << world->GetSimTime().Float() << endl;
#endif
       outputCSV << world->GetSimTime().Float() << ", ";
       for(unsigned int i = 0; i < boost::size(links); ++i){
          outputCSV << model->GetLink(links[i])->GetWorldLinearAccel().GetLength() << ", ";
       }
       outputCSV << endl;

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
    GZ_REGISTER_MODEL_PLUGIN(LinkAccelerationOverTimePlugin);
}

