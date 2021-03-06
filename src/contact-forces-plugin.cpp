#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>

#define PRINT_SENSORS 0
#define PRINT_DEBUG 0

using namespace std;

namespace gazebo {
    //! Standard number of ms for HIC calculation
   static const unsigned int ACC_HISTORY_MAX = 15; // From http://www.mchenrysoftware.com/HIC%20and%20the%20ATB.pdf

   static const unsigned int SEC_TO_MSEC = 1e3;

   static const unsigned int MIN_HIC_ACC_TIME = 3; // Minimum number of ms for a collision

   static const double G = 9.80665;

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
    private: std::map<std::string, common::Time> maxForceTimes;
    private: std::map<std::string, math::Vector3> maxForceAccs;
    private: std::map<std::string, math::Vector3> maxForceVelocities;
    private: std::map<std::string, double> maxTorques;
    private: std::map<std::string, std::string> collidingLink;

    private: gazebo::transport::NodePtr node;
    private: event::ConnectionPtr connection;
    private: vector<event::ConnectionPtr> sensorConnections;
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: vector<math::Vector3> headLinearAcc;
    private: double maximumHic;
    private: common::Time maximumHicTime;

    public: ContactForcesPlugin() : ModelPlugin() {
        #if(PRINT_DEBUG)
        cout << "Constructing the contact forces plugin" << std::endl;
        #endif
        maximumHic = 0;
    }
        
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      world = _model->GetWorld();
      model = _model;
      #if(PRINT_DEBUG)
      cout << "Loading the contact forces plugin" << endl;
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
        boost::bind(&ContactForcesPlugin::updateContacts, this, sensor, contacts[i])));
      }
      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ContactForcesPlugin::worldUpdate, this));
    }
    
    private: void updateContacts(sensors::SensorPtr sensor, const string& sensorName){
      // Get all the contacts.
      msgs::Contacts contacts = boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor)->GetContacts();
      physics::LinkPtr trunk = model->GetLink("trunk");

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
           cout << "   Wrench 1:"
           << contacts.contact(i).wrench(j).body_1_wrench().force().x() << " "
           << contacts.contact(i).wrench(j).body_1_wrench().force().y() << " "
           << contacts.contact(i).wrench(j).body_1_wrench().force().z() << "\n";
           cout << "   Wrench 2:"
           << contacts.contact(i).wrench(j).body_2_wrench().force().x() << " "
           << contacts.contact(i).wrench(j).body_2_wrench().force().y() << " "
           << contacts.contact(i).wrench(j).body_2_wrench().force().z() << "\n";
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
            maxForceTimes[sensorName] = this->world->GetSimTime();
            // We want the link it collides with
           vector<string> strs;
           boost::split(strs, sensorName, boost::is_any_of("::"));
           collidingLink[sensorName] = !boost::contains(contacts.contact(i).collision1(), strs[0]) ? contacts.contact(i).collision1() : contacts.contact(i).collision2();
#if PRINT_DEBUG
           cout << "Colliding link is: " << collidingLink[sensorName] << ". Options were: " << contacts.contact(i).collision1() << " and " << contacts.contact(i).collision2() << endl;
#endif
            maxForceVelocities[sensorName] = trunk->GetWorldLinearVel();
            maxForceAccs[sensorName] = trunk->GetWorldLinearAccel();
        }
        if(tTotal.GetLength() > maxTorques[sensorName]){
            maxTorques[sensorName] = tTotal.GetLength();
        }
      }
    }
    
    private: void writeHeader(ofstream& outputCSV){
      for(unsigned int i = 0; i < boost::size(contacts); ++i){
        outputCSV << contacts[i] << "(N), ";
      }
      outputCSV << "Maximum Link, Maximum Force(N), Maximum Force Time(s), Colliding Link, Maximum Force Velocity X (m/s), Maximum Force Velocity Y (m/s), Maximum Force Velocity Z (m/s), Maximum Force Acceleration X (m/s^2), Maximum Force Acceleration Y (m/s^2), Maximum Force Acceleration Z (m/s^2), HIC(s), HIC Time(s), ";
      
      for(unsigned int i = 0; i < boost::size(contacts); ++i){
        outputCSV << contacts[i] << "(Nm), ";
      }
      outputCSV << "Maximum Link, Maximum Torque(Nm), " << endl;
    }
    
    private: void printResults(){
            // Get the name of the folder to store the result in
            const char* resultsFolder = std::getenv("RESULTS_FOLDER");
            if(resultsFolder == nullptr){
              cout << "Results folder not set. Using current directory." << endl;
              resultsFolder = "./";
            }
        
            const string resultsFileName = string(resultsFolder) + "/" + "results.csv";
            bool exists = boost::filesystem::exists(resultsFileName);
            ofstream outputCSV;
            outputCSV.open(resultsFileName, ios::out | ios::app);
            assert(outputCSV.is_open());
      
            if(!exists){
              writeHeader(outputCSV);
            }
        
            // Print max forces
            double overallMax = 0;
            std::string overallMaxLink;
            for(unsigned int i = 0; i < boost::size(contacts); ++i){
                #if(PRINT_SENSORS)
                  std::cout << contacts[i] << ": " << maxForces[contacts[i]] << "(N)" << std::endl;
                #endif
                
                outputCSV << maxForces[contacts[i]] << ", ";
                if(maxForces[contacts[i]] > overallMax){
                    overallMax = maxForces[contacts[i]];
                    overallMaxLink = contacts[i];
                }
            }
        
            #if(PRINT_SENSORS)
              std::cout << "Maximum force: " << std::endl;
              std::cout << overallMaxLink << ": " << overallMax << "(N)" << std::endl;
            #endif
        
            outputCSV << overallMaxLink << ", " << overallMax << ", " << maxForceTimes[overallMaxLink].Float() << ", " << collidingLink[overallMaxLink] << "," << maxForceVelocities[overallMaxLink].x << ", " << maxForceVelocities[overallMaxLink].y << ", " << maxForceVelocities[overallMaxLink].z << ", " << maxForceAccs[overallMaxLink].x << ", " << maxForceAccs[overallMaxLink].y << ", " << maxForceAccs[overallMaxLink].z << ", ";

            // Print HIC
            outputCSV << maximumHic << ", " << maximumHicTime << ", ";

            // Print max torques
            overallMax = 0;
            for(unsigned int i = 0; i < boost::size(contacts); ++i){
                #if(PRINT_SENSORS)
                std::cout << contacts[i] << ": " << maxTorques[contacts[i]] << "(N)" << std::endl;
                #endif
                outputCSV << maxTorques[contacts[i]] << ", ";
                if(maxTorques[contacts[i]] > overallMax){
                    overallMax = maxTorques[contacts[i]];
                    overallMaxLink = contacts[i];
                }
            }
        
            #if(PRINT_SENSORS)
            std::cout << "Maximum torque: " << std::endl;
            std::cout << overallMaxLink << ": " << overallMax << "(N)" << std::endl;
            #endif
            outputCSV << overallMaxLink << ", " << overallMax << endl;
        
            outputCSV.close();
        }

    private: static math::Vector3 accMean(vector<math::Vector3>::const_iterator begin, vector<math::Vector3>::const_iterator end) {
       math::Vector3 result;
       unsigned int n = 0;
       for(vector<math::Vector3>::const_iterator i = begin; i != end; ++i) {
          result += *i;
          n++;
       }
       result /= n;
       return result;
    }

    private: void updateMaximumHic(bool drain) {
       // Get the current velocity for the head link.
       physics::LinkPtr head = model->GetLink("head_neck");
       assert(head);

       if (!drain) {
          headLinearAcc.push_back(head->GetWorldLinearAccel());
       }
       if(headLinearAcc.size() > ACC_HISTORY_MAX || drain) {
          headLinearAcc.erase(headLinearAcc.begin(), headLinearAcc.begin() + 1);
       }

       assert(headLinearAcc.size() <= ACC_HISTORY_MAX);

       // Now search for maximum value across all size dimensions
       for (unsigned int i = 0; i < headLinearAcc.size(); ++i) {
          for (unsigned int j = i + MIN_HIC_ACC_TIME; j < headLinearAcc.size(); ++j) {
             double hicCurrMax = hic(static_cast<double>(i) / SEC_TO_MSEC, static_cast<double>(j) / SEC_TO_MSEC, accMean(headLinearAcc.begin() + i, headLinearAcc.begin() + j + 1));
             if (hicCurrMax >= maximumHic) {
                maximumHic = hicCurrMax;
                maximumHicTime = world->GetSimTime();
             }
          }
       }
    }

    private: static double hic(const double timeI, const double timeJ, const math::Vector3 accAverage){
       return pow(accAverage.GetLength() / G, 2.5) * (timeJ - timeI);
    }

    private: void worldUpdate(){
        physics::LinkPtr head = model->GetLink("head_neck");
        #if(PRINT_DEBUG)
        cout << "Updating HIC for time " << world->GetSimTime() << endl;
        #endif
        updateMaximumHic(false);

        if(world->GetSimTime().Float() >= 2.0){
          #if(PRINT_DEBUG)
          cout << "Scenario completed. Updating results" << endl;
          #endif
          event::Events::DisconnectWorldUpdateBegin(this->connection);

           // Drain the HIC calculation
           for (unsigned int i = 0; i < ACC_HISTORY_MAX; ++i) {
              updateMaximumHic(true);
           }

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

