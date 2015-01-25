#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/world.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/sensors/sensors.hh>


#include <iostream>
#include <fstream>

#define PRINT_SENSORS 0

using namespace std;

namespace gazebo {
    
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
    
    class ContactForcesPlugin : public ModelPlugin {
        
    private: std::map<std::string, double> maxForces;
    private: std::map<std::string, double> maxTorques;

    private: gazebo::transport::NodePtr node;
    private: event::ConnectionPtr connection;
    private: vector<event::ConnectionPtr> sensorConnections;
    private: physics::WorldPtr world;
    
    public: ContactForcesPlugin() : ModelPlugin() {
        cout << "Constructing the contact forces plugin" << std::endl;
    }
        
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      world = _model->GetWorld();
      cout << "Loading the contact forces plugin" << endl;
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
        boost::bind(&ContactForcesPlugin::updateContacts, this, sensor, contacts[i])));
      }
      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ContactForcesPlugin::worldUpdate, this));
    }
    
    private: void updateContacts(sensors::SensorPtr sensor, const string& sensorName){
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
        math::Vector3 tTotal;
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
          tTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_wrench().torque().x(),
                            contacts.contact(i).wrench(j).body_1_wrench().torque().y(),
                            contacts.contact(i).wrench(j).body_1_wrench().torque().z());
        }
        #if(PRINT_SENSORS)
          cout << "Total Force: " << fTotal.GetLength() << endl;
          cout << "Total Torque: " << tTotal.GetLength() << endl;
        #endif
        if(fTotal.GetLength() > maxForces[sensorName]){
            maxForces[sensorName] = fTotal.GetLength();
        }
        if(tTotal.GetLength() > maxTorques[sensorName]){
            maxTorques[sensorName] = tTotal.GetLength();
        }
    }
    }
    
    private: void printResults(){
            // Get the name of the folder to store the result in
            const char* resultsFolder = std::getenv("RESULTS_FOLDER");
            if(resultsFolder == nullptr){
              resultsFolder = "./";
            }
        
            ofstream outputCSV;
            outputCSV.open(string(resultsFolder) + "/" + "results.csv", ios::out | ios::app);
            assert(outputCSV.is_open());
      
            // Print max forces
            double overallMax = 0;
            std::string overallMaxLink;
            for(unsigned int i = 0; i < boost::size(contacts); ++i){
                std::cout << contacts[i] << ": " << maxForces[contacts[i]] << "(N)" << std::endl;
                outputCSV << maxForces[contacts[i]] << ", ";
                if(maxForces[contacts[i]] > overallMax){
                    overallMax = maxForces[contacts[i]];
                    overallMaxLink = contacts[i];
                }
            }
            
            std::cout << "Maximum force: " << std::endl;
            std::cout << overallMaxLink << ": " << overallMax << "(N)" << std::endl;
        
            outputCSV << overallMaxLink << ", " << overallMax;
        
            // Print max torques
            overallMax = 0;
            for(unsigned int i = 0; i < boost::size(contacts); ++i){
                std::cout << contacts[i] << ": " << maxTorques[contacts[i]] << "(N)" << std::endl;
                outputCSV << maxTorques[contacts[i]] << ", ";
                if(maxTorques[contacts[i]] > overallMax){
                    overallMax = maxTorques[contacts[i]];
                    overallMaxLink = contacts[i];
                }
            }
            
            std::cout << "Maximum torque: " << std::endl;
            std::cout << overallMaxLink << ": " << overallMax << "(N)" << std::endl;
        
            outputCSV << overallMaxLink << ", " << overallMax << endl;
        
            outputCSV.close();
        }
        
    private: void worldUpdate(){
        if(world->GetSimTime().Float() >= 2.0){
          cout << "Scenario completed. Updating results" << endl;
          event::Events::DisconnectWorldUpdateBegin(this->connection);
          sensorConnections.clear();
          printResults();
          exit(0);
        }
    }
    };
    GZ_REGISTER_MODEL_PLUGIN(ContactForcesPlugin);
}

