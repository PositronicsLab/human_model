#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/world.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/sensors/sensors.hh>


#include <iostream>
#include <fstream>
#include <vector>

#define PRINT_SENSORS 0
#define PRINT_DEBUG 0

using namespace std;

namespace gazebo {

   static const double MAX_TIME = 2.0;
    
    class VelocityOverTimePlugin : public ModelPlugin {

    private: gazebo::transport::NodePtr node;
    private: event::ConnectionPtr connection;
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: physics::LinkPtr trunk;
    private: math::Vector3 averageLinearVelocity;
    private: math::Vector3 averageRotationalVelocity;
    private: ofstream outputCSV;

    public: VelocityOverTimePlugin() : ModelPlugin() {
        #if(PRINT_DEBUG)
        cout << "Constructing the velocity over time plugin" << std::endl;
        #endif
    }
        
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      world = _model->GetWorld();
      model = _model;
      trunk = model->GetLink("trunk");

      #if(PRINT_DEBUG)
      cout << "Loading the velocity over time plugin" << endl;
      #endif
      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&VelocityOverTimePlugin::worldUpdate, this));

      // Get the name of the folder to store the result in
      const char* resultsFolder = std::getenv("RESULTS_FOLDER");
      if(resultsFolder == nullptr){
         cout << "Results folder not set. Using current directory." << endl;
         resultsFolder = "./";
      }

      const string resultsFileName = string(resultsFolder) + "/" + "velocities.csv";
      outputCSV.open(resultsFileName, ios::out);
      assert(outputCSV.is_open());

      writeHeader(outputCSV);

       // Write out t0
       outputCSV << world->GetSimTime().Double() << ", " << trunk->GetWorldCoGLinearVel().x << ", " << trunk->GetWorldCoGLinearVel().y << ", " << trunk->GetWorldCoGLinearVel().z << ", " << trunk->GetWorldAngularAccel().x << ", " << trunk->GetWorldAngularAccel().y << ", " << trunk->GetWorldAngularAccel().z << ", " << endl;
    }
    
    private: void writeHeader(ofstream& outputCSV){
       outputCSV << "Time(s), x(m/s), y(m/s), z(m/s), ax(m/s), ay(m/s), az(m/s)," << endl;
    }

    private: void worldUpdate(){
#if(PRINT_DEBUG)
       cout << "Updating the world for time: " << world->GetSimTime().Float() << endl;
#endif

       averageLinearVelocity += trunk->GetWorldCoGLinearVel();
       averageRotationalVelocity += trunk->GetWorldAngularVel();
       if (world->GetSimTime().nsec % (10 * 1000000) == 0) {
          outputCSV << world->GetSimTime().Double() << ", " << averageLinearVelocity.x / 10.0 << ", " << averageLinearVelocity.y / 10.0 << ", " << averageLinearVelocity.z / 10.0 << ", " << averageRotationalVelocity.x / 10.0 << ", " << averageRotationalVelocity.y / 10.0 << ", " << averageRotationalVelocity.z / 10.0 << ", " << endl;
          averageLinearVelocity = 0;
          averageRotationalVelocity = 0;
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
    GZ_REGISTER_MODEL_PLUGIN(VelocityOverTimePlugin);
}

