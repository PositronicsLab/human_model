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
#include <fcl/BV/BV.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>

using namespace std;
using namespace KDL;

#define USE_FIXED_SEED 1
#define PRINT_POSITIONS 0
#define PRINT_DEBUG 1

// TODO: Reduce default
template <class T>
bool rough_eq(T lhs, T rhs, T epsilon = 0.025){
  return fabs(lhs - rhs) < epsilon;
}

namespace gazebo {
  const unsigned int VIRTUAL_JOINTS = 3;

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

  private: static vector<double> toVector(const JntArray jntArray){
     vector<double> values(jntArray.rows());
     for(unsigned int i = 0; i < jntArray.rows(); ++i){
        values.push_back(jntArray(i));
     }
     return values;
  }
    
    /**
     * Calculate the inverse.
     * Note: Target must be in the root link frame
     */
  private: vector<double> calcInverse(Chain& chain, physics::LinkPtr trunk, physics::JointPtr root, physics::LinkPtr endEffector, const math::Pose target){

      // Creation of the solvers
      ChainFkSolverPos_recursive fkSolver(chain);
      ChainIkSolverVel_wdls ikVelocitySolver(chain, 0.001, 1000);
      ikVelocitySolver.setLambda(0.1);

      // Disable weighting for orientation
      Eigen::VectorXd weights(6);
      weights[0] = weights[1] = weights[2] = 1.0;
      weights[3] = weights[4] = weights[5] = 0.0;
      ikVelocitySolver.setWeightTS(weights.asDiagonal());

      LowerLimits lowerLimitsCalc;
      UpperLimits upperLimitsCalc;

      math::Pose trunkPose = trunk->GetWorldPose();
      vector<double> lowerLimits = LowerLimits()(root, endEffector);

      // Reverse order due to inserting at the beginning each time
      lowerLimits.insert(lowerLimits.begin(), trunkPose.rot.GetYaw());
      lowerLimits.insert(lowerLimits.begin(), trunkPose.rot.GetPitch());
      lowerLimits.insert(lowerLimits.begin(), trunkPose.rot.GetRoll());
      assert(lowerLimits.size() == chain.getNrOfJoints());

      vector<double> upperLimits = UpperLimits()(root, endEffector);
      upperLimits.insert(upperLimits.begin(), trunkPose.rot.GetYaw());
      upperLimits.insert(upperLimits.begin(), trunkPose.rot.GetPitch());
      upperLimits.insert(upperLimits.begin(), trunkPose.rot.GetRoll());
      assert(upperLimits.size() == chain.getNrOfJoints());

      ChainIkSolverPos_NR_JL_PositionOnly ikSolver(chain, toJntArray(lowerLimits), toJntArray(upperLimits),fkSolver, ikVelocitySolver, 1000, 0.001);
  
      JntArray q(chain.getNrOfJoints());
      vector<double> qInit(chain.getNrOfJoints());

        // Initialize to midpoint of joint limits. Upper limits and lower limits account for virtual
        // joints
        for(unsigned int i = 0; i < qInit.size(); ++i){
            qInit[i] = upperLimits[i] - (upperLimits[i] - lowerLimits[i]) / 2.0;
        }

      // Set destination frame. Destination frame is in the trunk link frame.
      Frame dest = Frame(Vector(target.pos.x, target.pos.y, target.pos.z));
 
      int status = ikSolver.CartToJnt(toJntArray(qInit), dest, q);
      vector<double> angles;
      if(status >= 0){
         // Ignore virtual joints
        for(unsigned int i = VIRTUAL_JOINTS; i < chain.getNrOfJoints(); ++i){
          angles.push_back(q(i));
        }
         // TODO: Reenable
         // assert(checkFK(chain, trunk, endEffector, angles));
      } else {
        cerr << "IK Failed with status: " << status << endl;
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
                  double length, depth;
                  if(rough_eq(abs(parent->GetInitialAnchorPose().rot.GetPitch()), boost::math::constants::pi<double>() / 2.0)){
                     length = -link->GetBoundingBox().GetXLength();
                     depth = -link->GetBoundingBox().GetZLength() / 2.0;
                  } else {
                     length = -link->GetBoundingBox().GetZLength();
                     depth = -link->GetBoundingBox().GetXLength() / 2.0;
                  }
                  // Length compensates for the model being 3d and having a non-zero cylinder radius,
                  // but the KDL model has 0 radius links.
                  segmentVector = Frame(Vector(depth, 0.0, length));
               }
            }

            Segment segment = Segment(parent->GetName() + axisName(jointType(parent, i)), kdlJoint, segmentVector);
            chain.addSegment(segment);
            parent = nextJoint;
          }
        } while (parent != nullptr);
        return chain;
    }
    
  private: bool checkFK(const Chain& chain, physics::LinkPtr trunk, physics::LinkPtr leaf, const vector<double> qIn) const {

     // Add virtual joint
     math::Pose trunkPose = trunk->GetWorldPose();

     vector<double> q = qIn;

     // Reverse order due to inserting at the beginning each time
     q.insert(q.begin(), trunkPose.rot.GetYaw());
     q.insert(q.begin(), trunkPose.rot.GetPitch());
     q.insert(q.begin(), trunkPose.rot.GetRoll());

      // Construct the solver
      ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
      
      // Create joint array
      unsigned int nj = chain.getNrOfJoints();
      assert(nj == q.size());
      
      KDL::JntArray jointPositions = toJntArray(q);
      
      // Create the frame that will contain the results
      KDL::Frame cartPos;
      
      // Calculate forward position kinematics
      // Position is in a coordinate system defined by the center of the trunk with no rotation.
      bool status = fksolver.JntToCart(jointPositions, cartPos);
      if(status >= 0){
         math::Quaternion identity = math::Quaternion();
         identity.SetToIdentity();
         trunkPose.rot = identity;

         // Translate to the global frame.
         double x, y, z, w;
         cartPos.M.GetQuaternion(x, y, z, w);
         math::Pose endEffectorPoseInTrunkFrame = math::Pose(math::Vector3(cartPos.p.x(), cartPos.p.y(), cartPos.p.z()), math::Quaternion(x, y, z, w)) * trunkPose;

         math::Vector3 pos = endEffectorPoseInTrunkFrame.pos;

         // Determine the offset in the foot frame
        // Compensate for the difference between center and tip of the end effector.
        // TODO: Compensate for additional cases here
         math::Vector3 tipInFootFrame;
        if(rough_eq(abs(jointPositions(nj - 1)), boost::math::constants::pi<double>() / 2.0)){
           tipInFootFrame = math::Vector3(leaf->GetBoundingBox().GetZLength() / 2.0, 0.0, leaf->GetBoundingBox().GetXLength() / 2.0);
        } else {
           tipInFootFrame = math::Vector3(leaf->GetBoundingBox().GetXLength() / 2.0, 0.0, leaf->GetBoundingBox().GetZLength() / 2.0);
        }

        // Translate to the offset to the world frame
        math::Pose tipInWorldFrame =  math::Pose(tipInFootFrame, identity) + leaf->GetWorldPose();

         if(!rough_eq(pos.x, tipInWorldFrame.pos.x)){
          cout << "X values did not match: " << pos << " " << tipInWorldFrame.pos << endl;
          return false;
        }
        if(!rough_eq(pos.y, tipInWorldFrame.pos.y)){
          cout << "Y values did not match: " << pos << " " << tipInWorldFrame.pos << endl;
          return false;
        }
        if(!rough_eq(pos.z, tipInWorldFrame.pos.z)){
          cout << "Z values did not match: " << pos << " " << tipInWorldFrame.pos << endl;
          return false;
        }
        return true;
       }
       cout << "Failed to calculate FK" << endl;
       return false;
    }
    
    private: math::Pose transformGlobalToJointFrame(const math::Pose& pose, const physics::LinkPtr root) const {
       // We want to transform into the center of the trunk frame, but not the orientation of the trunk
       // frame because that is how the kinematic chain is rooted.
       math::Pose trunkPose = root->GetWorldPose();
       math::Quaternion identity = math::Quaternion();
       identity.SetToIdentity();
       trunkPose.rot = identity;
       return pose - trunkPose;
    }

  private: fcl::Transform3f poseToTransform(const math::Pose& pose){
     fcl::Quaternion3f q(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
     fcl::Vec3f t(pose.pos.x, pose.pos.y, pose.pos.z);
     return fcl::Transform3f(q, t);
  }

  private: bool isChildOf(const physics::LinkPtr parent, const physics::LinkPtr child) const {
     physics::Link_V children = parent->GetChildJointsLinks();
     for(unsigned int i = 0; i < children.size(); ++i){
        if(children[i]->GetId() == child->GetId()){
           return true;
        }
     }
     return false;
  }

  private: bool hasCollision(physics::ModelPtr model){
     vector<fcl::AABB> boxes;
     for(unsigned int i = 0; i < model->GetLinks().size(); ++i){
        // Use model bounding box, not link, which includes children
        fcl::Box boxModel(model->GetLinks()[i]->GetModel()->GetBoundingBox().GetXLength(), model->GetLinks()[i]->GetModel()->GetBoundingBox().GetYLength(), model->GetLinks()[i]->GetModel()->GetBoundingBox().GetZLength());
        fcl::AABB box;
        fcl::computeBV(boxModel, poseToTransform(model->GetLinks()[i]->GetWorldPose()), box);
        boxes.push_back(box);
     }

     assert(boxes.size() == model->GetLinks().size());

     for(unsigned int i = 0; i < boxes.size(); ++i){
        for(unsigned int j = i + 1; j < boxes.size(); ++j){
           if(boxes[i].overlap(boxes[j])){
              // TODO: Check self-collision setting
              if(!(isChildOf(model->GetLinks()[i], model->GetLinks()[j]) || isChildOf(model->GetLinks()[j], model->GetLinks()[i]))){
                 cout << "Collision between: " << model->GetLinks()[i]->GetName() << " and " << model->GetLinks()[j]->GetName() << endl;
                 return true;
              }
           }
        }
     }

     // Now check for ground collision
     // TODO: Allow contact with foot
     fcl::Halfspace groundPlaneSpace = fcl::Halfspace(fcl::Vec3f(0, 0, 1), 0.0);
     fcl::AABB groundPlane;
     fcl::computeBV(groundPlaneSpace, fcl::Transform3f(), groundPlane);
     for(unsigned int i = 0; i < boxes.size(); ++i){
        if(boxes[i].overlap(groundPlane)){
           cout << "Collision between: " << model->GetLinks()[i]->GetName() << " and ground plane" << endl;
           return true;
        }
     }

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
       math::Quaternion identity;
       identity.SetToIdentity();

       physics::LinkPtr trunk = model->GetLink("trunk");

       // Create the IK chains. Must be done prior to setting rpy on trunk.
       physics::LinkPtr rightFoot = model->GetLink("right_foot");
       physics::JointPtr rightHip = model->GetJoint("right_hip");
       Chain rightLeg = constructChain(trunk, rightHip, rightFoot);
       math::Pose rightFootPose = math::Pose(rightFoot->GetWorldPose().pos, identity);

       // Use IK for left leg
       physics::LinkPtr leftFoot = model->GetLink("left_foot");
       physics::JointPtr leftHip = model->GetJoint("left_hip");
       Chain leftLeg = constructChain(trunk, leftHip, leftFoot);
       math::Pose leftFootPose = math::Pose(leftFoot->GetWorldPose().pos, identity);

       GetAngles getAngles;
       assert(checkFK(leftLeg, trunk, leftFoot, getAngles(leftHip, leftFoot)));
       assert(checkFK(rightLeg, trunk, rightFoot, getAngles(rightHip, rightFoot)));

       // Limit pitch and roll to pi/4 but allow any yaw.
       boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > pitchRollGenerator(rng, boost::uniform_real<double>(-boost::math::constants::pi<double>() / 8.0, boost::math::constants::pi<double>() / 8.0));

       boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > yawGenerator(rng, boost::uniform_real<double>(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>()));

       bool foundLegalConfig = false;
       while (!foundLegalConfig){

          // Reset configuration to base
          model->SetLinkWorldPose(math::Pose(math::Vector3(trunk->GetWorldPose().pos), math::Quaternion(0, 0, 0)), trunk);

          // TODO: Reset joint angles?

          double roll = pitchRollGenerator();
          double pitch = pitchRollGenerator();
          double yaw = yawGenerator();

          math::Vector3 trunkPose = trunk->GetWorldPose().pos;
          model->SetLinkWorldPose(math::Pose(math::Vector3(trunk->GetWorldPose().pos), math::Quaternion(roll, pitch, yaw)), trunk);

          // Confirmation cartesian position did not change
          assert(trunk->GetWorldPose().pos == trunkPose);
          assert(transformGlobalToJointFrame(trunk->GetWorldPose(), trunk).pos == math::Vector3(0, 0, 0));

          // TODO: Reenable when frames are correct
          // assert(checkFK(leftLeg, trunk, leftFoot, getAngles(leftHip, leftFoot)));
          // assert(checkFK(rightLeg, trunk, rightFoot, getAngles(rightHip, rightFoot)));

          // Set random RPY on arms
          SetRandomAngles randomAngleSetter(rng);
          randomAngleSetter(model->GetJoint("left_shoulder"), model->GetLink("left_hand"));
          randomAngleSetter(model->GetJoint("right_shoulder"), model->GetLink("right_hand"));
    
          // Now pick a leg to set randomly
          boost::bernoulli_distribution<> randomLeg(0.5);
          if(randomLeg(rng)){
             // Set right leg randomly
             randomAngleSetter(model->GetJoint("right_hip"), model->GetLink("right_foot"));

             // Select the position equal to the current planar position of the foot at zero height
             // TODO: Compensate for difference between foot center and tip
             math::Pose leftFootPose = math::Pose(leftFoot->GetWorldPose().pos, identity);
             leftFootPose.pos.z = 0;
             vector<double> angles = calcInverse(leftLeg, trunk, leftHip, leftFoot, transformGlobalToJointFrame(leftFootPose, trunk));

             // Check for IK failure
             if(angles.size() == 0){
                // Next config
                continue;
             }
      
             // Now apply joint angles to a chain
             SetJointAngles setAngles(angles);
             setAngles(leftHip, leftFoot);
          }
          else {
             // Set left leg randomly
             randomAngleSetter(model->GetJoint("left_hip"), model->GetLink("left_foot"));

             // Use IK for right leg

             // Select the position equal to the current planar position of the foot at zero height
             // TODO: Compensate for difference between foot center and tip
             // TODO: Make function to get foot tip location
             rightFootPose.pos.z = 0;

             vector<double> angles = calcInverse(rightLeg, trunk, rightHip, rightFoot, transformGlobalToJointFrame(rightFootPose, trunk));

             // Check for IK failure
             if(angles.size() == 0){
                // Next config
                continue;
             }

             // Now apply joint angles to a chain
             SetJointAngles setAngles(angles);
             setAngles(rightHip, rightFoot);
          }

          // Check for intersection
          if(hasCollision(model)){
             cout << "Model has collision" << endl;
          }
          else {
             foundLegalConfig = true;
          }
       }
      
      csvFile << endl;
      csvFile.close();
    }

  };
  GZ_REGISTER_MODEL_PLUGIN(InitialJointPositionPlugin)
}
