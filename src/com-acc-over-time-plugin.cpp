#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/physics/Link.hh>

#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>

#define PRINT_DEBUG 0

using namespace std;

namespace gazebo {

   static const double MAX_TIME = 2.0;
    
    class AccelerationOverTimePlugin : public ModelPlugin {

    private: gazebo::transport::NodePtr node;
    private: event::ConnectionPtr connection;
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: physics::LinkPtr trunk;
    private: math::Vector3 averageLinearAcceleration;
    private: ofstream outputCSVX;
    private: ofstream outputCSVY;
    private: ofstream outputCSVZ;
    private: unsigned int count;
    public: AccelerationOverTimePlugin() : ModelPlugin() {
#if(PRINT_DEBUG)
       cout << "Constructing the acceleration over time plugin" << std::endl;
#endif
       count = 0;
    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      world = _model->GetWorld();
      model = _model;
      trunk = model->GetLink("trunk");

      #if(PRINT_DEBUG)
      cout << "Loading the acceleration over time plugin" << endl;
      #endif
      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AccelerationOverTimePlugin::worldUpdate, this));

      // Get the name of the folder to store the result in
      const char* resultsFolder = std::getenv("RESULTS_FOLDER");
      if(resultsFolder == nullptr){
         cout << "Results folder not set. Using current directory." << endl;
         resultsFolder = "./";
      }

      {
        const string resultsXFileName = string(resultsFolder) + "/" + "accelerations_x.csv";
        bool exists = boost::filesystem::exists(resultsXFileName);

        outputCSVX.open(resultsXFileName, ios::out | ios::app);
        assert(outputCSVX.is_open());
        if (!exists) {
          writeHeader(outputCSVX);
        }
      }

      {
        const string resultsYFileName = string(resultsFolder) + "/" + "accelerations_y.csv";
        bool exists = boost::filesystem::exists(resultsYFileName);

        outputCSVY.open(resultsYFileName, ios::out | ios::app);
        assert(outputCSVY.is_open());

        if (!exists) {
          writeHeader(outputCSVY);
        }
      }


      {
        const string resultsZFileName = string(resultsFolder) + "/" + "accelerations_z.csv";
        bool exists = boost::filesystem::exists(resultsZFileName);

        outputCSVZ.open(resultsZFileName, ios::out | ios::app);
        assert(outputCSVZ.is_open());

        if (!exists) {
           writeHeader(outputCSVZ);
        }
      }

       // Write out t0
       outputCSVX << "Acceleration X (m/s^2), " << trunk->GetWorldLinearAccel().x << ", ";
       outputCSVY << "Acceleration Y (m/s^2), " << trunk->GetWorldLinearAccel().y << ", ";
       outputCSVZ << "Acceleration Z (m/s^2), " << trunk->GetWorldLinearAccel().z << ", ";
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

       averageLinearAcceleration += trunk->GetWorldLinearAccel() / 10.0;
       count++;
       if (world->GetSimTime().nsec % (10 * 1000000) == 0) {
          assert(count == 10);
          outputCSVX << averageLinearAcceleration.x << ", ";
          outputCSVY << averageLinearAcceleration.y << ", ";
          outputCSVZ << averageLinearAcceleration.z << ", ";
          averageLinearAcceleration = 0;
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
    GZ_REGISTER_MODEL_PLUGIN(AccelerationOverTimePlugin);
}

