#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>

#define PRINT_SENSORS 0
#define PRINT_DEBUG 0

using namespace std;

namespace gazebo {

   static const double MAX_TIME = 2.0;

   static const std::string contacts[] = {
        "trunk::trunk_contact",
        "right_foot::right_foot_contact",
        "left_foot::left_foot_contact",
        "right_leg::right_leg_contact",
        "left_leg::left_leg_contact",
        "right_thigh::right_thigh_contact",
        "right_thigh::right_thigh_contact",
        "transpelvic_link::transpelvic_contact",
        "clavicular_link::clavicular_link_contact",
        "head_neck::head_neck_contact",
        "right_upper_arm::right_upper_arm_contact",
        "left_upper_arm::left_upper_arm_contact",
        "right_forearm::right_forearm_contact",
        "left_forearm::left_forearm_contact",
        "right_hand::right_hand_contact",
        "left_hand::left_hand_contact"
    };
    
    class ContactForcesOverTimePlugin : public ModelPlugin {

    private: gazebo::transport::NodePtr node;
    private: event::ConnectionPtr connection;
    private: vector<event::ConnectionPtr> sensorConnections;
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: double totalForce;
    private: ofstream outputCSV;

    public: ContactForcesOverTimePlugin() : ModelPlugin() {
        #if(PRINT_DEBUG)
        cout << "Constructing the contact forces over time plugin" << std::endl;
        #endif
    }
        
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      world = _model->GetWorld();
      model = _model;
      #if(PRINT_DEBUG)
      cout << "Loading the contact forces over time plugin" << endl;
      #endif
      
      #if(PRINT_SENSORS)
      cout << "Listing all sensors: " << endl;
      vector<sensors::SensorPtr> sensors = sensors::SensorManager::Instance()->GetSensors();
      for(unsigned int i = 0; i < sensors.size(); ++i){
        cout << sensors[i]->GetScopedName() << endl;
      }
      cout << endl;
      #endif
      
      for(unsigned int i = 0; i < boost::size(contacts); ++i){
        sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor(world->GetName() + "::" + _model->GetScopedName()
                                                       + "::" + contacts[i]);
        if(sensor == nullptr){
            cout << "Could not find sensor " << contacts[i] << endl;
            continue;
        }
        sensor->SetActive(true);
        sensorConnections.push_back(sensor->ConnectUpdated(
        boost::bind(&ContactForcesOverTimePlugin::contactForceForSensor, this, sensor, contacts[i])));
      }
      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ContactForcesOverTimePlugin::worldUpdate, this));

      // Get the name of the folder to store the result in
      const char* resultsFolder = std::getenv("RESULTS_FOLDER");
      if(resultsFolder == nullptr){
         cout << "Results folder not set. Using current directory." << endl;
         resultsFolder = "./";
      }

      const string resultsFileName = string(resultsFolder) + "/" + "contact_forces.csv";
      bool exists = boost::filesystem::exists(resultsFileName);
      outputCSV.open(resultsFileName, ios::out | ios::app);
      assert(outputCSV.is_open());

      if(!exists){
         writeHeader(outputCSV);
      }
      outputCSV << "Total Contact Force(N)" << ",";
    }

    private: void contactForceForSensor(sensors::SensorPtr sensor, const string& sensorName){
      #if(PRINT_DEBUG)
       cout << "Updating sensor " << sensorName << endl;
      #endif
      // Get all the contacts.
      msgs::Contacts contacts = boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor)->GetContacts();

      for (unsigned int i = 0; i < contacts.contact_size(); ++i){
        #if(PRINT_SENSORS)
        cout << "Collision between[" << contacts.contact(i).collision1()
             << "] and [" << contacts.contact(i).collision2() << "]\n";
        cout << " t[" << this->world->GetSimTime()
             << "] i[" << i
             << "] s[" << contacts.contact(i).time().sec()
             << "] n[" << contacts.contact(i).time().nsec()
             << "] size[" << contacts.contact(i).position_size()
             << "]\n";
        #endif

        math::Vector3 fTotal;
        for(unsigned int j = 0; j < contacts.contact(i).position_size(); ++j){
          #if(PRINT_SENSORS)
          cout << j << "  Position:"
               << contacts.contact(i).position(j).x() << " "
               << contacts.contact(i).position(j).y() << " "
               << contacts.contact(i).position(j).z() << "\n";
          cout << "   Normal:"
               << contacts.contact(i).normal(j).x() << " "
               << contacts.contact(i).normal(j).y() << " "
               << contacts.contact(i).normal(j).z() << "\n";
          cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
          #endif
          
          fTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_wrench().force().x(),
                            contacts.contact(i).wrench(j).body_1_wrench().force().y(),
                            contacts.contact(i).wrench(j).body_1_wrench().force().z());
        }
        #if(PRINT_SENSORS)
          cout << "Total Force: " << fTotal.GetLength() << endl;
        #endif
        totalForce += fTotal.GetLength();
      }
    }
    
    private: void writeHeader(ofstream& outputCSV){

       outputCSV << "Time" << ", ";
       for(double t = 0.0; t <= MAX_TIME; t += 0.001) {
          outputCSV << t << ", ";
       }
       outputCSV << endl;
    }

    private: void worldUpdate(){
#if(PRINT_DEBUG)
       cout << "Updating the world for time: " << world->GetSimTime().Float() << endl;
#endif
        outputCSV << totalForce / 10.0 << ", ";
        totalForce = 0;

        if(world->GetSimTime().Float() >= MAX_TIME){
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
          outputCSV << endl;
          outputCSV.close();
        }
    }
    };
    GZ_REGISTER_MODEL_PLUGIN(ContactForcesOverTimePlugin);
}

