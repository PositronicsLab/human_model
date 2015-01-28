#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/physics/joint.hh>
#include <gazebo/physics/world.hh>
#include <gazebo/physics/link.hh>
#include <stdio.h>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <iostream>
#include <iostream>
#include <fstream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

using namespace std;
using namespace KDL;

#define USE_FIXED_SEED 1
#define PRINT_POSITIONS 0
#define PRINT_DEBUG 1

namespace gazebo {

  class InitialJointPositionPlugin : public ModelPlugin {
    
    private: physics::ModelPtr model;
    
    public: InitialJointPositionPlugin() : ModelPlugin() {
      #if(PRINT_DEBUG)
      std::cout << "Constructing the initial position plugin" << std::endl;
      #endif
    }

    private: void writeHeader(ofstream& file){
      // TODO
      file << endl;
    }
    
    /**
     * Determine the joint type in KDL depending on the axis.
     */
    private: Joint::JointType jointType(const physics::JointPtr joint, const int axis){
            math::Vector3 localAxis = joint->GetLocalAxis(axis);
            Joint::JointType jointType;
            if(fabs(localAxis.x) == 1.0){
              jointType = Joint::RotX;
            }
            else if(fabs(localAxis.y) == 1.0){
                jointType = Joint::RotY;
            }
            else if(fabs(localAxis.z) == 1.0){
                jointType = Joint::RotZ;
            }
            else {
                assert(false);
            }
            return jointType;
    }
    
    private: string axisName(const Joint::JointType jointType){
      if(jointType == Joint::JointType::RotX){
        return "X";
      }
      if(jointType == Joint::RotY){
        return "Y";
      }
      if(jointType == Joint::RotZ){
        return "Z";
      }
      assert(false);
    }
    
    private: Chain constructChain(physics::JointPtr root, physics::LinkPtr endEffector, vector<double>& q){
        assert(root != nullptr && "Root joint is null");
        assert(endEffector != nullptr && "End effector is null");
        
        Chain chain;
        
        physics::JointPtr parent = root;
        physics::LinkPtr link = nullptr;
        do {
          cout << "Operating on joint: " << parent->GetName() << endl;
          link = parent->GetChild();
            
          // Iterate over degrees of freedom
          for(unsigned int i = 0; i < parent->GetAngleCount(); ++i){
            Joint::JointType jt = jointType(parent, i);
            Joint kdlJoint = Joint(jt);
            
            q.push_back(parent->GetAngle(i).Radian());
            
            // Construct a segment.
            bool isFinalAxis = i == parent->GetAngleCount() - 1;
            Frame linkLength = Frame(Vector(isFinalAxis ? link->GetBoundingBox().GetXLength() : 0.0, 0.0, 0.0));
            Segment segment = Segment(parent->GetName() + axisName(jt), kdlJoint, linkLength);
            chain.addSegment(segment);
          }
          
          assert(link->GetChildJoints().size() == 1 || (link->GetChildJoints().size() == 0 && link == endEffector));
          
          if(link->GetChildJoints().size() > 0){
            parent = link->GetChildJoints()[0];
          }
        } while (link != endEffector);
        
        // TODO: Set the joint offset for the base joint
        // TODO: Use forward kinematics to check the configuration
        return chain;
    }
    
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      model = _model;
      
      #if(PRINT_DEBUG)
      std::cout << "Loading the initial position plugin" << std::endl;
      #endif
      // Get the name of the folder to store the result in
      const char* resultsFolder = std::getenv("RESULTS_FOLDER");
      if(resultsFolder == nullptr){
        cout << "Results folder not set. Using current directory." << endl;
        resultsFolder = "./";
      }
      
      // Open a file to store the random values
      ofstream csvFile;
      
      const string resultsFileName = string(resultsFolder) + "/" + "joint_positions.csv";
      bool exists = boost::filesystem::exists(resultsFileName);
      csvFile.open(resultsFileName, ios::out | ios::app);
      assert(csvFile.is_open());
      
      if(!exists){
        writeHeader(csvFile);
      }

      // Create a random number generator. Note that this has a minute bias that it will
      // not generate 1.0
      boost::mt19937 rng;
      #if(!USE_FIXED_SEED)
        rng.seed(static_cast<unsigned int>(std::time(nullptr)));
      #else
        const char* scenarioNumber = std::getenv("i");
        if(scenarioNumber != nullptr){
            rng.seed(boost::lexical_cast<unsigned int>(scenarioNumber) + 1000);
        }
        else {
          cout << "No scenario number set. Using 0" << endl;
          rng.seed(0);
        }
      #endif
      boost::uniform_real<float> u(-1.0f, 1.0f);
      boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > gen(rng, u);

      // Construct the kinematic chains
      Chain leftArm = constructChain(model->GetJoint("left_shoulder"), model->GetLink("left_hand"));
      
      // TODO: Move to separate function
      ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(leftArm);
      
      // Create joint array
      unsigned int nj = leftArm.getNrOfJoints();
      KDL::JntArray jointPositions = JntArray(nj);
      
      // TODO: Copy in current joint positions
      // Create the frame that will contain the results
      KDL::Frame cartPos;
      
      // Calculate forward position kinematics
      bool status = fksolver.JntToCart(jointPositions, cartPos);
      if(status >= 0){
        std::cout << cartPos <<std::endl;
        printf("%s \n","Succes, thanks KDL!");
       } else {
        printf("%s \n","Error: could not calculate forward kinematics :(");
       }
    
      // TODO: Remaining three chains
      // TODO: Solve IK and set joint angles
      csvFile << endl;
      csvFile.close();
    }

  };
  GZ_REGISTER_MODEL_PLUGIN(InitialJointPositionPlugin)
}
