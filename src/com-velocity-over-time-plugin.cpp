#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/sensors.hh>


#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>

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
    private: ofstream outputCSVX;
    private: ofstream outputCSVY;
    private: ofstream outputCSVZ;
    private: int count;
    public: VelocityOverTimePlugin() : ModelPlugin() {
        #if(PRINT_DEBUG)
        cout << "Constructing the velocity over time plugin" << std::endl;
        #endif
        count = 0;
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

      {
        const string resultsXFileName = string(resultsFolder) + "/" + "velocities_x.csv";
        bool exists = boost::filesystem::exists(resultsXFileName);

        outputCSVX.open(resultsXFileName, ios::out | ios::app);
        assert(outputCSVX.is_open());
        if (!exists) {
          writeHeader(outputCSVX);
        }
      }

      {
        const string resultsYFileName = string(resultsFolder) + "/" + "velocities_y.csv";
        bool exists = boost::filesystem::exists(resultsYFileName);

        outputCSVY.open(resultsYFileName, ios::out | ios::app);
        assert(outputCSVY.is_open());

        if (!exists) {
          writeHeader(outputCSVY);
        }
      }


      {
        const string resultsZFileName = string(resultsFolder) + "/" + "velocities_z.csv";
        bool exists = boost::filesystem::exists(resultsZFileName);

        outputCSVZ.open(resultsZFileName, ios::out | ios::app);
        assert(outputCSVZ.is_open());

        if (!exists) {
           writeHeader(outputCSVZ);
        }
      }

       // Write out t0
       outputCSVX << "Velocity X (m/s), " << trunk->GetWorldCoGLinearVel().x << ", ";
       outputCSVY << "Velocity Y (m/s), " << trunk->GetWorldCoGLinearVel().y << ", ";
       outputCSVZ << "Velocity Z (m/s), " << trunk->GetWorldCoGLinearVel().z << ", ";
    }
    
    private: static void writeHeader(ofstream& outputCSV){
       outputCSV << "Time(s), ";
       for (double t = 0.0; t <= 2.0000000001 /* Float error */; t += 0.01) {
          outputCSV << t << ", ";
       }
       outputCSV << endl;
     }

    private: void worldUpdate(){
#if(PRINT_DEBUG)
       cout << "Updating the world for time: " << world->GetSimTime().Float() << endl;
#endif

       averageLinearVelocity += trunk->GetWorldCoGLinearVel() / 10.0;
       count++;
       if (world->GetSimTime().nsec % (10 * 1000000) == 0) {
          assert(count == 10);
          outputCSVX << averageLinearVelocity.x << ", ";
          outputCSVY << averageLinearVelocity.y << ", ";
          outputCSVZ << averageLinearVelocity.z << ", ";
          averageLinearVelocity = 0;
          count = 0;
       }

        if(world->GetSimTime().Float() >= MAX_TIME){
          #if(PRINT_DEBUG)
          cout << "Scenario completed. Updating results" << endl;
          #endif
          event::Events::DisconnectWorldUpdateBegin(this->connection);

          outputCSVX << endl;
          outputCSVX.close();

          outputCSVY << endl;
          outputCSVY.close();

          outputCSVZ << endl;
          outputCSVZ.close();
        }
    }
    };
    GZ_REGISTER_MODEL_PLUGIN(VelocityOverTimePlugin);
}

