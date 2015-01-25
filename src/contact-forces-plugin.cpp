#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/world.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/sensors/sensors.hh>


#include <iostream>
#include <fstream>

using namespace std;

namespace gazebo {
    
    // List of topics to listen on
    static const std::string topics[] = {
        "/gazebo/default/human/trunk/trunk_contact",
        "/gazebo/default/human/left_foot/left_foot_contact",
        "/gazebo/default/human/right_foot/right_foot_contact",
        "/gazebo/default/human/left_leg/left_leg_contact",
        "/gazebo/default/human/right_leg/right_leg_contact",
        "/gazebo/default/human/left_thigh/left_thigh_contact",
        "/gazebo/default/human/right_thigh/right_thigh_contact",
        "/gazebo/default/human/transpelvic_link/transpelvic_contact",
        "/gazebo/default/human/clavicular_link/clavicular_link_contact",
        "/gazebo/default/human/head_neck/head_neck_contact",
        "/gazebo/default/human/left_upper_arm/left_upper_arm_contact",
        "/gazebo/default/human/right_upper_arm/right_upper_arm_contact",
        "/gazebo/default/human/left_forearm/left_forearm_contact",
        "/gazebo/default/human/right_forearm/right_forearm_contact",
        "/gazebo/default/human/left_hand/left_hand_contact",
        "/gazebo/default/human/right_hand/right_hand_contact"
    };
    
    static const std::string contacts[] = {
        "trunk::collision",
        "left_foot::collision",
        "right_foot::collision",
        "left_leg::collision",
        "right_leg::collision",
        "left_thigh::collision",
        "right_thigh::collision",
        "transpelvic_link::collision",
        "clavicular_link::collision",
        "head_neck::collision",
        "left_upper_arm::collision",
        "right_upper_arm::collision",
        "left_forearm::collision",
        "right_forearm::collision",
        "left_hand::collision",
        "right_hand::collision",
    };
    
    class ContactForcesPlugin : public WorldPlugin {
        
    private: std::map<std::string, double> maxForces;
    private: std::map<std::string, double> maxTorques;

    private: ofstream outputCSV;
    private: gazebo::transport::NodePtr node;
    private: event::ConnectionPtr connection;
    private: vector<sensors::ContactSensor> sensors;
    private: physics::WorldPtr world;
    
    public: ContactForcesPlugin() : WorldPlugin() {
        std::cout << "Constructing the contact forces plugin" << std::endl;
    }
        
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      world = _world;
      for(unsigned int i = 0; i < boost::size(contacts); ++i){
        sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor(_world->GetName() + "::" + _world->GetModel("human")->GetScopedName()
                                                       + "::" + contacts[i]);
        sensor->ConnectUpdated(
        boost::bind(&ContactForcesPlugin::updateContacts, this, sensor));
      }
      connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ContactForcesPlugin::worldUpdate, this));
    }
    
    private: void updateContacts(sensors::SensorPtr sensor){
      // Get all the contacts.
      msgs::Contacts contacts = boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor)->GetContacts();
      
      for (unsigned int i = 0; i < contacts.contact_size(); ++i){
        cout << "Collision between[" << contacts.contact(i).collision1()
             << "] and [" << contacts.contact(i).collision2() << "]\n";
        cout << " t[" << this->world->GetSimTime()
             << "] i[" << i
             << "] s[" << contacts.contact(i).time().sec()
             << "] n[" << contacts.contact(i).time().nsec()
             << "] size[" << contacts.contact(i).position_size()
             << "]\n";


        math::Vector3 fTotal;
        math::Vector3 tTotal;
        for(unsigned int j = 0; j < contacts.contact(i).position_size(); ++j){
          cout << j << "  Position:"
               << contacts.contact(i).position(j).x() << " "
               << contacts.contact(i).position(j).y() << " "
               << contacts.contact(i).position(j).z() << "\n";
          cout << "   Normal:"
               << contacts.contact(i).normal(j).x() << " "
               << contacts.contact(i).normal(j).y() << " "
               << contacts.contact(i).normal(j).z() << "\n";
          cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
          fTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_wrench().force().x(),
                            contacts.contact(i).wrench(j).body_1_wrench().force().y(),
                            contacts.contact(i).wrench(j).body_1_wrench().force().z());
          tTotal += math::Vector3(
                            contacts.contact(i).wrench(j).body_1_wrench().torque().x(),
                            contacts.contact(i).wrench(j).body_1_wrench().torque().y(),
                            contacts.contact(i).wrench(j).body_1_wrench().torque().z());
        }
        maxForces[sensor->GetName()] += fTotal.GetLength();
        maxTorques[sensor->GetName()] += tTotal.GetLength();
    }
    }
    
    private: void printResults(){
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
          printResults();
          exit(0);
        }
    }
    };
    GZ_REGISTER_WORLD_PLUGIN(ContactForcesPlugin);
}

