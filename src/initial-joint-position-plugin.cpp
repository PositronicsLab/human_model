#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/model.hh>
#include <gazebo/physics/joint.hh>
#include <gazebo/physics/world.hh>
#include <gazebo/physics/link.hh>
#include <stdio.h>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/random/bernoulli_distribution.hpp>
#include <iostream>
#include <iostream>
#include <fstream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <boost/math/constants/constants.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include "chainiksolverpos_nr_jl_position_only.hpp"
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

  struct SetJointValue {
      virtual void setValue(const physics::JointPtr joint, const unsigned int index, const unsigned int globalIndex) = 0;
      
      void operator()(physics::JointPtr root, physics::LinkPtr endEffector) {
       assert(root != nullptr && "Root joint is null");
        assert(endEffector != nullptr && "End effector is null");
        
        physics::JointPtr parent = root;
        physics::LinkPtr link = nullptr;
        unsigned int j = 0;
        do {
          link = parent->GetChild();
            
          // Iterate over degrees of freedom
          for(unsigned int i = 0; i < parent->GetAngleCount(); ++i){
            setValue(parent, i, j);
            ++j;
          }
          
          assert(link->GetChildJoints().size() == 1 || (link->GetChildJoints().size() == 0 && link == endEffector));
          
          if(link->GetChildJoints().size() > 0){
            parent = link->GetChildJoints()[0];
          }
        } while (link != endEffector);
      }
    };
    
    struct SetJointAngles : public SetJointValue {
      private: const vector<double>& angles;
      public: SetJointAngles(const vector<double>& angles): angles(angles){
      }
      
      public: virtual void setValue(const physics::JointPtr joint, const unsigned int index, const unsigned int globalIndex){
        if(!joint->SetPosition(index, angles[globalIndex])){
          cout << "Failed to set position for joint: " << joint->GetName() << endl;
        }
      }
    };
    
    struct SetRandomAngles : public SetJointValue {
     
      private: boost::mt19937& rng;
      
      public: SetRandomAngles(boost::mt19937& rng):rng(rng){
      }
      
      public: virtual void setValue(const physics::JointPtr joint, const unsigned int index, const unsigned int globalIndex){
      
        // Generate a random double within the allowed range
        boost::uniform_real<double> u(joint->GetLowerLimit(index).Radian(), joint->GetUpperLimit(index).Radian());
        boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > gen(rng, u);
      
        if(!joint->SetPosition(index, gen())){
          cout << "Failed to set position for joint: " << joint->GetName() << endl;
        }
      }
    };
  
  struct JointValues {
      virtual double getValue(const physics::JointPtr joint, const unsigned int index) const = 0;
      vector<double> operator()(physics::JointPtr root, physics::LinkPtr endEffector) const {
       assert(root != nullptr && "Root joint is null");
        assert(endEffector != nullptr && "End effector is null");
        
        vector<double> limits;
        physics::JointPtr parent = root;
        physics::LinkPtr link = nullptr;
        do {
          link = parent->GetChild();
            
          // Iterate over degrees of freedom
          for(unsigned int i = 0; i < parent->GetAngleCount(); ++i){
            limits.push_back(getValue(parent, i));
          }
          
          assert(link->GetChildJoints().size() == 1 || (link->GetChildJoints().size() == 0 && link == endEffector));
          
          if(link->GetChildJoints().size() > 0){
            parent = link->GetChildJoints()[0];
          }
        } while (link != endEffector);
        return limits;
      }
    };
    
    struct GetAngles : public JointValues {
      virtual double getValue(const physics::JointPtr joint, const unsigned int index) const {
        // TODO: This isn't right for alternate axis joints
        return joint->GetInitialAnchorPose().rot.GetPitch();
      }
    };
    
    struct UpperLimits : public JointValues {
      virtual double getValue(const physics::JointPtr joint, const unsigned int index) const {
        return joint->GetUpperLimit(index).Radian();
      }
    };
    
    struct LowerLimits : public JointValues {
      virtual double getValue(const physics::JointPtr joint, const unsigned int index) const {
        return joint->GetLowerLimit(index).Radian();
      }
    };
    
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
    
    private: static JntArray toJntArray(const vector<double> values){
      JntArray jArray(values.size());
      for(unsigned int i = 0; i < values.size(); ++i){
        jArray(i) = values[i];
      }
      return jArray;
    }
    
    /**
     * Calculate the inverse.
     * Note: Target must be in the root link frame
     */
    private: vector<double> calcInverse(Chain& chain, physics::JointPtr root, physics::LinkPtr endEffector, const math::Pose target){
    
      // Creation of the solvers
      ChainFkSolverPos_recursive fkSolver(chain);
      ChainIkSolverVel_wdls ikVelocitySolver(chain, 0.01, 1000);
      ikVelocitySolver.setLambda(0.1);
      // Disable weighting for orientation
      Eigen::VectorXd weights(6);
      weights[0] = weights[1] = weights[2] = 1.0;
      weights[3] = weights[4] = weights[5] = 0.0;
      ikVelocitySolver.setWeightTS(weights.asDiagonal());

      LowerLimits lowerLimitsCalc;
      UpperLimits upperLimitsCalc;
      
      const vector<double> lowerLimits = LowerLimits()(root, endEffector);
      assert(lowerLimits.size() == chain.getNrOfJoints());

      const vector<double> upperLimits = UpperLimits()(root, endEffector);
      assert(upperLimits.size() == chain.getNrOfJoints());

      ChainIkSolverPos_NR_JL_PositionOnly ikSolver(chain, toJntArray(lowerLimits), toJntArray(upperLimits),fkSolver, ikVelocitySolver, 1000, 0.01);
  
      JntArray q(chain.getNrOfJoints());
      vector<double> qInit(chain.getNrOfJoints());

        // Initialize to midpoint of joint limits
        // TODO: Init virtual joints here
        for(unsigned int i = 0; i < qInit.size(); ++i){
            qInit[i] = upperLimits[i] - (upperLimits[i] - lowerLimits[i]) / 2.0;
        }

      // Set destination frame. Destination frame is in the root link frame.
      Frame dest = Frame(Vector(target.pos.x, target.pos.y, target.pos.z));
 
        int status = ikSolver.CartToJnt(toJntArray(qInit), dest, q);
      vector<double> angles(chain.getNrOfJoints());
      if(status >= 0){
        for(unsigned int i = 0; i < chain.getNrOfJoints(); ++i){
          angles[i] = q(i);
        }
      } else {
        cout << "IK Failed " << status << endl;
        // TODO: Better return code here?
      }
      return angles;
    }
    
   private: Chain constructChain(physics::LinkPtr trunk, physics::JointPtr root, physics::LinkPtr endEffector){
        assert(root != nullptr && "Root joint is null");
        assert(trunk != nullptr && "Trunk link is null");
        assert(endEffector != nullptr && "End effector is null");

        Chain chain;

       // Construct the virtual Segment for the trunk. Add two 0 length segments and one segment
       // equal to the distance between the trunk and the root joint
       math::Pose virtualLinkOffset = root->GetWorldPose() - trunk->GetWorldPose();
       chain.addSegment(Segment("VirtualX", Joint("VirtualX-Joint", Vector(), Vector(1, 0, 0), Joint::RotAxis), Frame(Vector(0, 0, 0))));
       chain.addSegment(Segment("VirtualY", Joint("VirtualY-Joint", Vector(), Vector(0, 1, 0), Joint::RotAxis), Frame(Vector(0, 0, 0))));
       chain.addSegment(Segment("VirtualY", Joint("VirtualY-Joint", Vector(), Vector(0, 0, 1), Joint::RotAxis), Frame(Vector(virtualLinkOffset.pos.x, virtualLinkOffset.pos.y, virtualLinkOffset.pos.z))));

        physics::JointPtr parent = root;
        do {
          physics::LinkPtr link = parent->GetChild();

          assert(link->GetChildJoints().size() == 1 || (link->GetChildJoints().size() == 0 && link == endEffector));

          // Iterate over degrees of freedom
          unsigned int angleCount = parent->GetAngleCount();
          for(unsigned int i = 0; i < angleCount; ++i){
            Vector axis = Vector(parent->GetLocalAxis(i).x, parent->GetLocalAxis(i).y, parent->GetLocalAxis(i).z);
            Joint kdlJoint = Joint(parent->GetName() + axisName(jointType(parent, i)), Vector(), axis, Joint::RotAxis);

            // For multi-DOF joints, only the final segment has length > 0
            bool isFinalAxis = i == parent->GetAngleCount() - 1;

            // Determine the next joint
            physics::JointPtr nextJoint;
            if(link == endEffector){
               nextJoint = nullptr;
            }
            else if(isFinalAxis){
                nextJoint = link->GetChildJoints()[0];
            }
            else {
               // Constructing multiple virtual joints from one multi-DOF joint
               nextJoint = parent;
            }

            Frame segmentVector;
            if(nextJoint != nullptr){
               // Find the distance to the next joint center
               math::Vector3 pos = (nextJoint->GetWorldPose() - parent->GetWorldPose()).pos;
               segmentVector.p = Vector(pos.x, pos.y, pos.z);
            }
            else {
               // End effector. Use the bounding box for the size.
               if(!isFinalAxis){
                  segmentVector.p = Vector(0, 0, 0);
               }
               else {
                  // Bounding boxes are always in the global frame. If this is a rotated box, we may not want to use
                  // the z length.
                  // TODO: Make this generally correct.
                  double length;
                  if(rough_eq(abs(parent->GetInitialAnchorPose().rot.GetPitch()), boost::math::constants::pi<double>() / 2.0)){
                     length = -link->GetBoundingBox().GetXLength();
                  } else {
                     length = -link->GetBoundingBox().GetZLength();
                  }
                  segmentVector = Frame(Vector(0.0, 0.0, length));
               }
            }

            Segment segment = Segment(parent->GetName() + axisName(jointType(parent, i)), kdlJoint, segmentVector);
            chain.addSegment(segment);
            parent = nextJoint;
          }
        } while (parent != nullptr);
        return chain;
    }
    
  private: bool checkFK(const Chain& chain, physics::LinkPtr trunk, physics::LinkPtr leaf, physics::JointPtr root) const {
      // Get the current angles of all the joints
      GetAngles getAngles;
      vector<double> q = getAngles(root, leaf);

      // Add zeros for the virtual joints
      // TODO: Handle cases where the trunk is not oriented normally?
      for(unsigned int i = 0; i < 3; ++i){
         q.insert(q.begin(), 0.0);
      }

      // Construct the solver
      ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
      
      // Create joint array
      unsigned int nj = chain.getNrOfJoints();
      assert(nj == q.size());
      
      KDL::JntArray jointPositions = toJntArray(q);
      
      // Create the frame that will contain the results
      KDL::Frame cartPos;
      
      // Calculate forward position kinematics
      // Note: FK is in local coordinates
      bool status = fksolver.JntToCart(jointPositions, cartPos);
      if(status >= 0){
        // Translate to the global frame.
         math::Quaternion q;
         q.SetToIdentity();
         math::Vector3 pos = (trunk->GetWorldPose() + math::Pose(math::Vector3(cartPos.p.x(), cartPos.p.y(), cartPos.p.z()), q)).pos;
        
        // Compensate for the difference between center and tip of the end effector.
        // TODO: Compensate for additional cases here
        if(rough_eq(abs(jointPositions(nj - 1)), boost::math::constants::pi<double>() / 2.0)){
           pos = math::Vector3(pos.x - leaf->GetBoundingBox().GetXLength() / 2.0, pos.y, pos.z);
        } else {
           pos = math::Vector3(pos.x, pos.y, pos.z - leaf->GetBoundingBox().GetZLength() / 2.0);
        }
        
        math::Vector3 endPos = leaf->GetBoundingBox().GetCenter();
        if(!rough_eq(pos.x, endPos.x)){
          cout << "X values did not match: " << pos << " " << endPos << endl;
          return false;
        }
        if(!rough_eq(pos.y, endPos.y)){
          cout << "Y values did not match: " << pos << " " << endPos << endl;
          return false;
        }
        if(!rough_eq(pos.z, endPos.z)){
          cout << "Z values did not match: " << pos << " " << endPos << endl;
          return false;
        }
        return true;
       }
       cout << "Failed to calculate FK" << endl;
       return false;
    }
    
    private: math::Pose transformGlobalToJointFrame(const math::Pose& pose, const physics::LinkPtr root) const {
      return root->GetWorldPose().GetInverse() * pose;
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

      // TODO: Set random RPY on trunk here
      // Set random RPY on arms
      SetRandomAngles randomAngleSetter(rng);
      randomAngleSetter(model->GetJoint("left_shoulder"), model->GetLink("left_hand"));
      randomAngleSetter(model->GetJoint("right_shoulder"), model->GetLink("right_hand"));
    
      // Now pick a leg to set randomly
      boost::bernoulli_distribution<> randomLeg(0.5);
      math::Quaternion identity;
      identity.SetToIdentity();

        physics::LinkPtr trunk = model->GetLink("trunk");

      if(randomLeg(rng)){
        // Set right leg randomly
        randomAngleSetter(model->GetJoint("right_hip"), model->GetLink("right_foot"));

        // Use IK for left leg
        physics::LinkPtr leftFoot = model->GetLink("left_foot");
        physics::JointPtr leftHip = model->GetJoint("left_hip");
        Chain leftLeg = constructChain(trunk, leftHip, leftFoot);
        assert(checkFK(leftLeg, trunk, leftFoot, leftHip));
    
        math::Pose leftFootPose = math::Pose(leftFoot->GetWorldCoGPose().pos, identity);
        vector<double> angles = calcInverse(leftLeg, leftHip, leftFoot, transformGlobalToJointFrame(leftFootPose, trunk));
      
        // Now apply joint angles to a chain
        SetJointAngles setAngles(angles);
        setAngles(leftHip, leftFoot);
      }
      else {
        // Set left leg randomly
        randomAngleSetter(model->GetJoint("left_hip"), model->GetLink("left_foot"));
        
        // Use IK for right leg
        physics::LinkPtr rightFoot = model->GetLink("right_foot");
        physics::JointPtr rightHip = model->GetJoint("right_hip");
        Chain rightLeg = constructChain(trunk, rightHip, rightFoot);
        assert(checkFK(rightLeg, trunk, rightFoot, rightHip));

        math::Pose rightFootPose = math::Pose(rightFoot->GetWorldCoGPose().pos, identity);
        vector<double> angles = calcInverse(rightLeg, rightHip, rightFoot, transformGlobalToJointFrame(rightFootPose, trunk));
      
        // Now apply joint angles to a chain
        SetJointAngles setAngles(angles);
        setAngles(rightHip, rightFoot);
      }

        

      
      csvFile << endl;
      csvFile.close();
    }

  };
  GZ_REGISTER_MODEL_PLUGIN(InitialJointPositionPlugin)
}
