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
#include <boost/math/constants/constants.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

using namespace std;
using namespace KDL;

#define USE_FIXED_SEED 1
#define PRINT_POSITIONS 0
#define PRINT_DEBUG 1

template <class T>
bool rough_eq(T lhs, T rhs, T epsilon = 0.001){
  return fabs(lhs - rhs) < epsilon;
}

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
    
    private: vector<double> jointAngles(physics::JointPtr root, physics::LinkPtr endEffector) const {
        assert(root != nullptr && "Root joint is null");
        assert(endEffector != nullptr && "End effector is null");
        
        vector<double> q;
        physics::JointPtr parent = root;
        physics::LinkPtr link = nullptr;
        do {
          link = parent->GetChild();
            
          // Iterate over degrees of freedom
          for(unsigned int i = 0; i < parent->GetAngleCount(); ++i){
            math::Pose offset = parent->GetInitialAnchorPose();
            // TODO: This isn't right for alternate axis joints
            q.push_back(offset.rot.GetPitch());
          }
          
          assert(link->GetChildJoints().size() == 1 || (link->GetChildJoints().size() == 0 && link == endEffector));
          
          if(link->GetChildJoints().size() > 0){
            parent = link->GetChildJoints()[0];
          }
        } while (link != endEffector);
        return q;
    }
    
    private: JntArray calcInverse(Chain& chain, const vector<double> qInitD){
      // Creation of the solvers:
      ChainFkSolverPos_recursive fkSolver(chain);
      ChainIkSolverVel_pinv ikVelocitySolver(chain);
      ChainIkSolverPos_NR ikSolver(chain, fkSolver, ikVelocitySolver, 100, 1e-6);
 
      JntArray q(chain.getNrOfJoints());
      JntArray qInit(chain.getNrOfJoints());
        
      assert(qInitD.size() == chain.getNrOfJoints());
      for(unsigned int i = 0; i < qInitD.size(); ++i){
        qInit(i) = qInitD[i];
      }
      // Set destination frame
      Frame dest = Frame(Vector(0.644, 0, 0));
 
      int status = ikSolver.CartToJnt(qInit, dest, q);
      if(status >= 0){
        cout << "Solved IK!" << endl;
        for(unsigned int i = 0; i < chain.getNrOfJoints(); ++i){
          cout << q(i) << ", ";
        }
        cout << endl;
      }
      else {
        cout << "IK Failed " << endl;
      }
      return q;
    }
    
    private: Chain constructChain(physics::JointPtr root, physics::LinkPtr endEffector){
        assert(root != nullptr && "Root joint is null");
        assert(endEffector != nullptr && "End effector is null");
        
        Chain chain;
        
        physics::JointPtr parent = root;
        physics::LinkPtr link = nullptr;
        do {
          link = parent->GetChild();
            
          // Iterate over degrees of freedom
          for(unsigned int i = 0; i < parent->GetAngleCount(); ++i){
            Vector axis = Vector(parent->GetLocalAxis(i).x, parent->GetLocalAxis(i).y, parent->GetLocalAxis(i).z);
            Joint kdlJoint = Joint(parent->GetName() + axisName(jointType(parent, i)), Vector(), axis, Joint::RotAxis);
            
            // Construct a segment.
            bool isFinalAxis = i == parent->GetAngleCount() - 1;
            
            // Bounding boxes are always in the global frame. If this is a rotated box, we may not want to use
            // the z length.
            // TODO: Make this generally correct
            double length;
            if(rough_eq(abs(parent->GetInitialAnchorPose().rot.GetPitch()), boost::math::constants::pi<double>() / 2.0)){
                length = link->GetBoundingBox().GetXLength();
            } else {
                length = link->GetBoundingBox().GetZLength();
            }
            Frame linkLength = Frame(Vector(isFinalAxis ? length : 0.0, 0.0, 0.0));
            Segment segment = Segment(parent->GetName() + axisName(jointType(parent, i)), kdlJoint, linkLength);
            chain.addSegment(segment);
          }
          
          assert(link->GetChildJoints().size() == 1 || (link->GetChildJoints().size() == 0 && link == endEffector));
          
          if(link->GetChildJoints().size() > 0){
            parent = link->GetChildJoints()[0];
          }
        } while (link != endEffector);
        
        return chain;
    }
    
    private: bool checkFK(const Chain& chain, physics::LinkPtr leaf, physics::JointPtr root, bool flipped) const {
      // Get the current angles of all the joints
      vector<double> q = jointAngles(root, leaf);
        
      // Construct the solver
      ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
      
      // Create joint array
      unsigned int nj = chain.getNrOfJoints();
      assert(nj == q.size());
      
      KDL::JntArray jointPositions = JntArray(nj);
      for(unsigned int i = 0; i < nj; i++){
        jointPositions(i) = q[i];
      }
      
      // Create the frame that will contain the results
      KDL::Frame cartPos;
      
      // Calculate forward position kinematics
      // Note: FK is in local coordinates
      bool status = fksolver.JntToCart(jointPositions, cartPos);
      if(status >= 0){
        // Translate to the global frame.
        math::Vector3 origin = root->GetAnchor(0);
        // TODO: Remove flipped. Compensates for the model arms being built wrong
        Frame baseFrame = Frame(Rotation::RotY((flipped ? -1 : 1) * boost::math::constants::pi<double>() / 2.0), Vector(origin.x, origin.y, origin.z));

        Vector pos = (baseFrame * cartPos).p;
        
        // Compensate for the difference between center and tip of the end effector.
        // TODO: Compensate for additional cases here
        if(rough_eq(abs(jointPositions(nj - 1)), boost::math::constants::pi<double>() / 2.0)){
            pos = Vector(pos.x() - leaf->GetBoundingBox().GetXLength() / 2.0, pos.y(), pos.z());
        } else {
            pos = Vector(pos.x(), pos.y(), pos.z() - leaf->GetBoundingBox().GetZLength() / 2.0);
        }
        
        math::Vector3 endPos = leaf->GetBoundingBox().GetCenter();
        if(!rough_eq(pos.x(), endPos.x)){
          cout << "X values did not match: " << pos << " " << endPos << endl;
          return false;
        }
        if(!rough_eq(pos.y(), endPos.y)){
          cout << "Y values did not match: " << pos << " " << endPos << endl;
          return false;
        }
        if(!rough_eq(pos.z(), endPos.z)){
          cout << "Z values did not match: " << pos << " " << endPos << endl;
          return false;
        }
        return true;
       }
       cout << "Failed to calculate FK" << endl;
       return false;
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

      physics::LinkPtr leftHand = model->GetLink("left_hand");
      physics::JointPtr leftShoulder = model->GetJoint("left_shoulder");
      Chain leftArm = constructChain(leftShoulder, leftHand);
      assert(checkFK(leftArm, leftHand, leftShoulder, true));
      
      calcInverse(leftArm, jointAngles(leftShoulder, leftHand));
      
      physics::LinkPtr rightHand = model->GetLink("right_hand");
      physics::JointPtr rightShoulder = model->GetJoint("right_shoulder");
      Chain rightArm = constructChain(rightShoulder, rightHand);
      assert(checkFK(rightArm, rightHand, rightShoulder, true));

      physics::LinkPtr leftFoot = model->GetLink("left_foot");
      physics::JointPtr leftHip = model->GetJoint("left_hip");
      Chain leftLeg = constructChain(leftHip, leftFoot);
      assert(checkFK(leftLeg, leftFoot, leftHip, false));
    
      physics::LinkPtr rightFoot = model->GetLink("right_foot");
      physics::JointPtr rightHip = model->GetJoint("right_hip");
      Chain rightLeg = constructChain(rightHip, rightFoot);
      assert(checkFK(rightLeg, rightFoot, rightHip, false));
      
      // TODO: Remaining three chains
      // TODO: Solve IK and set joint angles
      csvFile << endl;
      csvFile.close();
    }

  };
  GZ_REGISTER_MODEL_PLUGIN(InitialJointPositionPlugin)
}
