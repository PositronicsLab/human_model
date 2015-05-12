#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
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
#include <map>

using namespace std;
using namespace KDL;

#define USE_FIXED_SEED 1
#define PRINT_DEBUG 0
#define ENABLE_SECOND_ARM 1

static const int FIXED_SEED = 0;

/**
 * Determine if two values are approximately equal with respect to epsilon
 */
template <class T>
bool rough_eq(T lhs, T rhs, double epsilon = 0.01) {
    return fabs(lhs - rhs) < epsilon;
}

bool vector_rough_eq(const vector<double>& lhs, const vector<double>& rhs, double epsilon = 0.01) {
   if(lhs.size() != rhs.size()) {
      return false;
   }

   for (unsigned int i = 0; i < lhs.size(); ++i) {
      if(!rough_eq(lhs[i], rhs[i], epsilon)){
         return false;
      }
   }
   return true;
}

bool pos_rough_eq(const gazebo::math::Vector3& lhs, const gazebo::math::Vector3& rhs){
   return rough_eq(lhs.x, rhs.x) && rough_eq(lhs.y, rhs.y) && rough_eq(lhs.z, rhs.z);
}

// Normalize an angle into the range [-pi, pi]
template <class T>
double normalize_range(T angle) {
   if (angle >= boost::math::constants::pi<double>()) {
      return angle - 2 * boost::math::constants::pi<double>();
   }
   if (angle <= -boost::math::constants::pi<double>()) {
      return angle + 2 * boost::math::constants::pi<double>();
   }
   return angle;
}

namespace gazebo {

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
            for(unsigned int i = 0; i < parent->GetAngleCount(); ++i) {
               limits.push_back(getValue(parent, i));
            }

            if(link->GetChildJoints().size() > 0) {
               assert(link->GetChildJoints().size() == 1 || link == endEffector);
               parent = link->GetChildJoints()[0];
            }
         } while (link != endEffector);
         return limits;
      }
   };

   struct GetAngles : public JointValues {
      virtual double getValue(const physics::JointPtr joint, const unsigned int index) const {
         return joint->GetAngle(index).Radian();
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

struct SetJointValue {
protected:
    virtual void setValue(const physics::JointPtr joint, const unsigned int index, const unsigned int globalIndex) = 0;

public:
    virtual void finished(unsigned int j, physics::JointPtr root, physics::LinkPtr endEffector) = 0;
    void operator()(physics::JointPtr root, physics::LinkPtr endEffector) {
        assert(root != nullptr && "Root joint is null");
        assert(endEffector != nullptr && "End effector is null");

        physics::JointPtr parent = root;
        physics::LinkPtr link = nullptr;
        unsigned int j = 0;
        do {
            link = parent->GetChild();

            // Iterate over degrees of freedom
            for(unsigned int i = 0; i < parent->GetAngleCount(); ++i) {
                setValue(parent, i, j);
                ++j;
            }

            if(link->GetChildJoints().size() > 0) {
                parent = link->GetChildJoints()[0];
            }
        } while (link != endEffector);
        finished(j, root, endEffector);
    }
};

struct SetJointAngles : public SetJointValue {
private:
    const vector<double> angles;
public:
    SetJointAngles(const vector<double>& angles): angles(angles) {
    }

protected:
    virtual void setValue(const physics::JointPtr joint, const unsigned int index, const unsigned int globalIndex) {
       // Check that the value is in bounds
       assert(angles[globalIndex] <= joint->GetUpperLimit(index).Radian() && angles[globalIndex] >= joint->GetLowerLimit(index).Radian());

        if(!joint->SetPosition(index, angles[globalIndex])) {
            cout << "Failed to set position for joint: " << joint->GetName() << endl;
        }

       // Confirm the value was set properly
       assert(rough_eq(normalize_range(joint->GetAngle(index).Radian()), normalize_range(angles[globalIndex])));
    }

protected:
    virtual void finished(unsigned int j, physics::JointPtr root, physics::LinkPtr endEffector) {
        assert(j == angles.size());
    }
};

struct SetRandomAngles : public SetJointValue {

private:
    boost::mt19937& rng;

public:
    SetRandomAngles(boost::mt19937& rng):rng(rng) {
    }

protected:
    virtual void setValue(const physics::JointPtr joint, const unsigned int index, const unsigned int globalIndex) {

        // Generate a random double within the allowed range
        boost::uniform_real<double> u(joint->GetLowerLimit(index).Radian(), joint->GetUpperLimit(index).Radian());
        boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > gen(rng, u);

        if(!joint->SetPosition(index, gen())) {
            cout << "Failed to set position for joint: " << joint->GetName() << endl;
        }
    }
protected:
    virtual void finished(unsigned int j, physics::JointPtr root, physics::LinkPtr endEffector) {}
};

   static const std::string joints[] = {
      "left_ankle",
      "right_ankle",
      "left_knee",
      "right_knee",
      "left_hip",
      "right_hip",
      "left_shoulder",
      "right_shoulder",
      "left_elbow",
      "right_elbow",
      "left_wrist",
      "right_wrist"
   };

class InitialJointPositionPlugin : public ModelPlugin {


private:
    physics::ModelPtr model;

public:
    InitialJointPositionPlugin() : ModelPlugin() {
#if(PRINT_DEBUG)
        std::cout << "Constructing the initial position plugin" << std::endl;
#endif
    }

private:
    void writeHeader(ofstream& file) {
       file << "Seed,Left Arm, Right Arm, Attempts, IK Attempts";
       for (unsigned int i = 0; i < boost::size(joints); ++i) {
          physics::JointPtr currJoint = model->GetJoint(joints[i]);
          for (unsigned int j = 0; j < currJoint->GetAngleCount(); ++j) {
             file << joints[i] << "(" << j << "),";
          }
       }
       file << endl;
    }

    /**
     * Convert a vector of joint angles to a JntArray
     */
private:
    static JntArray toJntArray(const vector<double> values) {
        JntArray jArray(values.size());
        for(unsigned int i = 0; i < values.size(); ++i) {
            jArray(i) = values[i];
        }
        return jArray;
    }

    /**
     * Convert a JntArray to a vector joint angles
     */
private:
    static vector<double> toVector(const JntArray jntArray) {
        vector<double> values(jntArray.rows());
        for(unsigned int i = 0; i < jntArray.rows(); ++i) {
            values[i] = jntArray(i);
        }
        return values;
    }

    /**
     * Calculate the inverse.
     * Note: Target must be in the root link frame
     */
private:
    vector<double> calcInverse(Chain& chain, physics::JointPtr root, physics::LinkPtr endEffector, const math::Pose target) {
#if(PRINT_DEBUG)
       cout << "Search for IK solution for link: " << endEffector->GetName() << " for position: " << target.pos << " in world frame: " << transformFrameToGlobal(target, root).pos << endl;
#endif
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

        vector<double> lowerLimits = LowerLimits()(root, endEffector);
        assert(lowerLimits.size() == chain.getNrOfJoints());

        vector<double> upperLimits = UpperLimits()(root, endEffector);
        assert(upperLimits.size() == chain.getNrOfJoints());

        ChainIkSolverPos_NR_JL_PositionOnly ikSolver(chain, toJntArray(lowerLimits), toJntArray(upperLimits),fkSolver, ikVelocitySolver, 1000, 0.001);

        JntArray q(chain.getNrOfJoints());
        vector<double> qInit(chain.getNrOfJoints());

        // Initialize to midpoint of joint limits. Upper limits and lower limits account for virtual
        // joints
        for(unsigned int i = 0; i < qInit.size(); ++i) {
            qInit[i] = upperLimits[i] - (upperLimits[i] - lowerLimits[i]) / 2.0;
        }

        // Set destination frame. Destination frame is in the trunk link frame.
        Frame dest = Frame(Vector(target.pos.x, target.pos.y, target.pos.z));

        int status = ikSolver.CartToJnt(toJntArray(qInit), dest, q);
        vector<double> angles;
        if(status >= 0) {
            angles = toVector(q);
        } else {
#if(PRINT_DEBUG)
            cerr << "IK Failed with status: " << status << endl;
#endif
        }
        return angles;
    }

private:
    /**
     * Construct a kinematic chain from the root to the end effector with a virtual joint equivalent to the RPY of the trunk
     */
   Chain constructChain(physics::JointPtr root, physics::LinkPtr endEffector) {
        assert(root != nullptr && "Root joint is null");
        assert(endEffector != nullptr && "End effector is null");

#if (PRINT_DEBUG)
      cout << "Constructing chain from root " << root->GetName() << " to " << endEffector->GetName() << endl;
#endif
        Chain chain;

        physics::JointPtr parent = root;
        do {
            physics::LinkPtr link = parent->GetChild();

            // Iterate over degrees of freedom
            unsigned int angleCount = parent->GetAngleCount();
            for(unsigned int i = 0; i < angleCount; ++i) {
               Vector axis = Vector(parent->GetGlobalAxis(i).x, parent->GetGlobalAxis(i).y, parent->GetGlobalAxis(i).z);
               Joint kdlJoint = Joint(parent->GetName() + "_" + boost::lexical_cast<string>(i), Vector(), axis, Joint::RotAxis);

                // For multi-DOF joints, only the final segment has length > 0
                bool isFinalAxis = i == parent->GetAngleCount() - 1;

                // Determine the next joint
                physics::JointPtr nextJoint;
                if(link == endEffector) {
                   // TODO: This would be wrong for multi-DOF final joints before the EE
                   if(link->GetChildJoints().size() > 0){
                      nextJoint = link->GetChildJoints()[0];
                   }
                   else {
                     nextJoint = nullptr;
                   }
                }
                else if(isFinalAxis) {
                    nextJoint = link->GetChildJoints()[0];
                }
                else {
                    // Constructing multiple virtual joints from one multi-DOF joint
                    nextJoint = parent;
                }

                Frame segmentVector;
                if(nextJoint != nullptr) {
                    // Find the distance to the next joint center
                    math::Vector3 pos = nextJoint->GetWorldPose().pos - parent->GetWorldPose().pos;
                    segmentVector.p = Vector(pos.x, pos.y, pos.z);
                }
                else {
                    // End effector. Use the bounding box for the size.
                    if(!isFinalAxis) {
                        segmentVector.p = Vector(0, 0, 0);
                    }
                    else {
                       // TODO: Refactor into function
                       // TODO: Search through XML to find this
                       // double length = cylinder->GetElement("length")->GetValueDouble();
                       // double radius = cylinder->GetElement("radius")->GetValueDouble();
                       double length = 0.082;
                       double height = 0.04;

                       math::Vector3 tipInFootFrame(height / 2.0, 0.0, length / 2.0);

                       // Translate to the offset to the world frame
                       math::Pose tipInWorldFrame =  math::Pose(tipInFootFrame, identityQuaternion()) + endEffector->GetWorldPose();

                        // TODO: Use XML here because bounding boxes are troublesome
                       math::Vector3 offset = tipInWorldFrame.pos - parent->GetWorldPose().pos;
                       segmentVector.p = Vector(offset.x, offset.y, offset.z);
                    }
                }

               Segment segment = Segment(parent->GetName() + "_" + boost::lexical_cast<string>(i), kdlJoint, segmentVector);
                chain.addSegment(segment);

               if(link != endEffector){
                 parent = nextJoint;
               }
               else {
                  parent = nullptr;
               }
            }
        } while (parent != nullptr);
        return chain;
    }

    /**
     * Convert a kdl frame to a gazebo pose
     */
private:
    static math::Pose frameToPose(const KDL::Frame& frame) {
        double x, y, z, w;
        frame.M.GetQuaternion(x, y, z, w);
        return math::Pose(math::Vector3(frame.p.x(), frame.p.y(), frame.p.z()), math::Quaternion(x, y, z, w));
    }

private:
   math::Pose getCenterToTip(physics::LinkPtr leaf) const {
      // Extract the width and height of the foot from the model
      const sdf::ElementPtr cylinder = leaf->GetModel()->GetSDF()->GetElement("link")->GetElement("collision")->GetElement("geometry")->GetElement("cylinder");

      // TODO: Search through XML to find this
      // double length = cylinder->GetElement("length")->GetValueDouble();
      // double radius = cylinder->GetElement("radius")->GetValueDouble();
      double length = 0.082;
      double height = 0.04;

      math::Vector3 tipInEEFrame(height / 2.0, 0.0, length / 2.0);
      return math::Pose(tipInEEFrame, identityQuaternion());
   }

   bool checkFK(const Chain& chain, physics::LinkPtr leaf, physics::JointPtr root, const vector<double> q, math::Pose targetPose, bool isPoseTip) const {
        assert(leaf != nullptr);
        assert(root != nullptr);

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
        int status = fksolver.JntToCart(jointPositions, cartPos);
        if(status >= 0) {
            // Translate to the global frame.
            math::Pose endEffectorPoseInGlobalFrame = transformFrameToGlobal(frameToPose(cartPos), root);

           math::Pose tipInWorldFrame;

           if(!isPoseTip && leaf->GetChildJoints().size() == 0){


            // Translate to the offset to the world frame
            tipInWorldFrame = getCenterToTip(leaf) + targetPose;
           }
           else {
              tipInWorldFrame = targetPose;
           }

            if(!rough_eq(endEffectorPoseInGlobalFrame.pos.x, tipInWorldFrame.pos.x)) {
                cout << "X values did not match. Actual: " << endEffectorPoseInGlobalFrame.pos << " Expected: " << tipInWorldFrame.pos << endl;
                return false;
            }
            if(!rough_eq(endEffectorPoseInGlobalFrame.pos.y, tipInWorldFrame.pos.y)) {
                cout << "Y values did not match. Actual: " << endEffectorPoseInGlobalFrame.pos << " Expected:" << tipInWorldFrame.pos << endl;
                return false;
            }
            if(!rough_eq(endEffectorPoseInGlobalFrame.pos.z, tipInWorldFrame.pos.z)) {
                cout << "Z values did not match. Actual: " << endEffectorPoseInGlobalFrame.pos << " Expected:" << tipInWorldFrame.pos << endl;
                return false;
            }
            return true;
        }
        cout << "Failed to calculate FK" << endl;
        return false;
    }

private:
    static const math::Quaternion identityQuaternion() {
        static math::Quaternion identity = math::Quaternion();
        identity.SetToIdentity();
        return identity;
    }

private:
    /**
     * Transform a pose in the global frame into a frame represented by the joint in the identity orientation
     */
    static math::Pose transformGlobalToJoint(const math::Pose& pose, const physics::JointPtr joint) {
        // We want to transform into the center of the joitn frame, but not the orientation
        return math::Pose(pose.pos - joint->GetWorldPose().pos, identityQuaternion());
    }

private:
    /**
     * Transform a pose from the fixed frame to the global frame
     */
    static math::Pose transformFrameToGlobal(const math::Pose& pose, physics::JointPtr joint) {
        return math::Pose(pose.pos + joint->GetWorldPose().pos, identityQuaternion());
    }

    /**
     * Convert a gazebo pose to a fcl Transform 3f
     */
private:
    fcl::Transform3f poseToTransform(const math::Pose& pose) {
        fcl::Quaternion3f q(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
        fcl::Vec3f t(pose.pos.x, pose.pos.y, pose.pos.z);
        return fcl::Transform3f(q, t);
    }

    /**
     * Determine whether two links have a parent child relationship in either direction
     */
private:
    bool isChildOf(const physics::LinkPtr a, const physics::LinkPtr b) const {
        physics::Link_V children = a->GetChildJointsLinks();
        for(unsigned int i = 0; i < children.size(); ++i) {
            if(children[i]->GetId() == b->GetId()) {
                return true;
            }
        }
        return false;
    }

private:
    static vector<physics::LinkPtr> allChildLinks(physics::LinkPtr root) {

        // Find all the joints.
        vector<physics::LinkPtr> childLinks;
        vector<physics::LinkPtr> linksToSearch;
        linksToSearch.push_back(root);

        while(!linksToSearch.empty()) {
            physics::LinkPtr curr = linksToSearch.back();
            linksToSearch.pop_back();
            childLinks.push_back(curr);
            vector<physics::LinkPtr> currChildJoints = curr->GetChildJointsLinks();
            linksToSearch.insert(linksToSearch.begin(), currChildJoints.begin(), currChildJoints.end());
        }

        return childLinks;
    }

private:
   static bool containsLink(const vector<physics::LinkPtr> links, physics::LinkPtr target) {
      for (unsigned int i = 0; i < links.size(); ++i){
         if(target->GetId() == links[i]->GetId()){
            return true;
         }
      }
      return false;
   }

    /**
     * Determine if a model has an internal collision or collides with the ground. Contact link
     * is exempted from ground collision detection
     */
private:
   bool hasCollision(physics::ModelPtr model, physics::LinkPtr trunk, physics::LinkPtr contactLink, const vector<physics::LinkPtr>& robotEndEffectors) {
        map<physics::LinkPtr, fcl::AABB> boxes;

        for(unsigned int i = 0; i < model->GetLinks().size(); ++i) {
            // Use model bounding box, not link, which includes children
            fcl::Box boxModel(model->GetLinks()[i]->GetModel()->GetBoundingBox().GetXLength(), model->GetLinks()[i]->GetModel()->GetBoundingBox().GetYLength(), model->GetLinks()[i]->GetModel()->GetBoundingBox().GetZLength());
            fcl::AABB box;
            fcl::computeBV(boxModel, poseToTransform(model->GetLinks()[i]->GetWorldPose()), box);
            boxes[model->GetLinks()[i]] = box;
        }

        assert(boxes.size() == model->GetLinks().size());


        // Check for collisions between the human joints and all other joints, but not self-collisions between
        // the robots joints
        vector<physics::LinkPtr> humanJoints = allChildLinks(trunk);

      // Find all links that are part of the robot end effectors
      vector<physics::LinkPtr> eeLinks;
      for (unsigned int i = 0; i < robotEndEffectors.size(); ++i){
         if(robotEndEffectors[i] == nullptr){
            continue;
         }
         vector<physics::LinkPtr> currLinks = allChildLinks(robotEndEffectors[i]);
         // There are duplicates here, but that is fine
         eeLinks.insert(eeLinks.begin(), currLinks.begin(), currLinks.end());
      }

        for(unsigned int i = 0; i < humanJoints.size(); ++i) {
            for(unsigned int j = i + 1; j < model->GetLinks().size(); ++j) {
                // Ignore self-collision
                if(humanJoints[i]->GetId() == model->GetLinks()[j]->GetId()) {
                    continue;
                }

                if(boxes[humanJoints[i]].overlap(boxes[model->GetLinks()[j]])) {
                    if(!(containsLink(eeLinks, model->GetLinks()[j]) || isChildOf(humanJoints[i], model->GetLinks()[j]) || isChildOf(model->GetLinks()[j], humanJoints[i]))) {
#if(PRINT_DEBUG)
                        cout << "Collision between: " << humanJoints[i]->GetName() << " and " << model->GetLinks()[j]->GetName() << endl;
#endif
                        return true;
                    }
                }
            }
        }

        // Now check for ground collision with only the human joints
        fcl::Halfspace groundPlaneSpace = fcl::Halfspace(fcl::Vec3f(0, 0, 1), 0.0);
        fcl::AABB groundPlane;
        fcl::computeBV(groundPlaneSpace, fcl::Transform3f(), groundPlane);
        for(unsigned int i = 0; i < humanJoints.size(); ++i) {
            if(contactLink->GetId() != humanJoints[i]->GetId() && boxes[humanJoints[i]].overlap(groundPlane)) {
#if(PRINT_DEBUG)
                cout << "Collision between: " << humanJoints[i]->GetName() << " and ground plane" << endl;
#endif
                return true;
            }
        }

        return false;
    }

private:
    void setWorldPoseIncludingChildren(physics::LinkPtr root, math::Pose pose) {
        // Save the model pose
        math::Pose modelPose = model->GetWorldPose();

        // Set the model to the desired pose
        model->SetLinkWorldPose(pose, root);

        // Record all the poses in the world frame
        vector<physics::LinkPtr> allLinks = allChildLinks(root);
        vector<math::Pose> poses;
        for(unsigned int i = 0; i < allLinks.size(); ++i) {
            poses.push_back(allLinks[i]->GetWorldPose());
        }

        // Restore the model pose
        model->SetWorldPose(modelPose, true, true);

        // Now set the poses for all the links
        for(unsigned int i = 0; i < allLinks.size(); ++i) {
            allLinks[i]->SetWorldPose(poses[i], true, true);
        }
    }

private:
   template<typename T>
   static void print(const vector<T>& v) {
      for (unsigned int i = 0; i < v.size(); ++i) {
        cout << v[i] << ", ";
      }
      cout << endl;
   }

private: Chain constructRobotArmChain(const string rootName, const string endEffectorName) {
   // Create the IK chains.
   physics::LinkPtr endEffector = model->GetLink(endEffectorName);
   physics::JointPtr root = model->GetJoint(rootName);

   // Check if this is a human only scenario
   if(endEffector == nullptr) {
      // Construct an empty chain
      return Chain();
   }

   assert(root != nullptr);
   Chain chain = constructChain(root, endEffector);
   GetAngles getAngles;

   assert(checkFK(chain, endEffector, root, getAngles(root, endEffector), endEffector->GetChildJoints()[0]->GetWorldPose(), true));

   return chain;
}

private:
   bool moveRobotArm(Chain& chain, const string rootName, const string endEffectorName, const string target){
      physics::LinkPtr endEffector = model->GetLink(endEffectorName);
      physics::JointPtr root = model->GetJoint(rootName);

      // Check if this is a human only scenario
      if(endEffector == nullptr) {
         return true;
      }

#if(PRINT_DEBUG)
      cout << "Performing IK on robot arm starting at " << rootName << " to " << endEffectorName << endl;
#endif
      vector<double> angles = calcInverse(chain, root, endEffector, transformGlobalToJoint(model->GetJoint(target)->GetWorldPose(), root));

      // Check for IK failure
      if(angles.size() == 0) {
         return false;
      }

      assert(checkFK(chain, endEffector, root, angles, model->GetJoint(target)->GetWorldPose(), true));

#if(PRINT_DEBUG)
      cout << "Moving robot arm to angles: ";
      print(angles);
#endif

      // Now apply joint angles to a chain
      SetJointAngles setAngles(angles);
      setAngles(root, endEffector);
      assert(checkFK(chain, endEffector, root, GetAngles()(root, endEffector), model->GetJoint(target)->GetWorldPose(), true));

      // Check whether the target posed was reached
      // TODO: Determine why this check is failing.
      // assert(pos_rough_eq(model->GetJoint("left_hip")->GetWorldPose().pos, endEffector->GetWorldPose().pos));
      return true;
   }

private: void connectVirtualJoint(const string& parentName, const string& childName) {
   // Setup the virtual joint if needed
   physics::LinkPtr parent = model->GetLink(parentName);
   if(parent) {
#if(PRINT_DEBUG)
      cout << "Connecting virtual joint" << endl;
#endif
      physics::JointPtr joint = model->GetWorld()->GetPhysicsEngine()->CreateJoint(/*"universal" */ "ball", model);
      physics::LinkPtr child = model->GetLink(childName);
      joint->Attach(parent, child);

      joint->Load(parent, child, math::Pose(math::Vector3(0, 0, 0.145), math::Quaternion()));
      joint->SetName("virtual_robot_human_connection_" + childName);
      bool set = joint->SetParam("erp", 0, 0.8);
      assert(set);
      set = joint->SetParam("cfm", 0, 0.0);
      assert(set);

      // Cannot get the angle count prior to init.
      for (unsigned int i = 0; i < 2; ++i) {
         bool set = joint->SetParam("friction", i, 100.0);
         assert(set);
      }
      joint->Init();
   }
}

private:
   double linkToJointDistance(const string linkName, const string jointName) {
      return model->GetLink(linkName)->GetWorldPose().pos.Distance(model->GetJoint(jointName)->GetWorldPose().pos);
   }

public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        model = _model;

#if(PRINT_DEBUG)
        std::cout << "Loading the initial position plugin" << std::endl;
#endif
        // Get the name of the folder to store the result in
        const char* resultsFolder = std::getenv("RESULTS_FOLDER");
        if(resultsFolder == nullptr) {
            cout << "Results folder not set. Using current directory." << endl;
            resultsFolder = "./";
        }

        // Open a file to store the random values
        ofstream csvFile;

        const string resultsFileName = string(resultsFolder) + "/" + "joint_positions.csv";
        bool exists = boost::filesystem::exists(resultsFileName);
        csvFile.open(resultsFileName, ios::out | ios::app);
        assert(csvFile.is_open());

        if(!exists) {
            writeHeader(csvFile);
        }

        // Create a random number generator. Note that this has a minute bias that it will
        // not generate 1.0
        boost::mt19937 rng;
        unsigned int seed = 0;
#if(!USE_FIXED_SEED)
        seed = static_cast<unsigned int>(std::time(nullptr));
#else
        const char* scenarioNumber = std::getenv("i");
        if(scenarioNumber != nullptr) {
            seed = boost::lexical_cast<unsigned int>(scenarioNumber) + 1000;
        }
        else {
            cout << "No scenario number set. Using 0" << endl;
            seed = FIXED_SEED;
        }
#endif
        rng.seed(seed);
        physics::LinkPtr trunk = model->GetLink("trunk");

        // Create the IK chains.
        physics::LinkPtr leftFoot = model->GetLink("left_foot");
        physics::JointPtr leftHip = model->GetJoint("left_hip");
        Chain leftLeg = constructChain(leftHip, leftFoot);
        math::Pose leftFootPose = math::Pose(leftFoot->GetWorldPose().pos, identityQuaternion());

        physics::LinkPtr rightFoot = model->GetLink("right_foot");
        physics::JointPtr rightHip = model->GetJoint("right_hip");
        Chain rightLeg = constructChain(rightHip, rightFoot);
        math::Pose rightFootPose = math::Pose(rightFoot->GetWorldPose().pos, identityQuaternion());

        GetAngles getAngles;

        assert(checkFK(leftLeg, leftFoot, leftHip, getAngles(leftHip, leftFoot), leftFoot->GetWorldPose(), false));
        assert(checkFK(rightLeg, rightFoot, rightHip, getAngles(rightHip, rightFoot), rightFoot->GetWorldPose(), false));

       // Construct the arm chain ahead of time to ensure links are in base positions
#if(ENABLE_SECOND_ARM)
        Chain lRobotArmChain = constructRobotArmChain("r_shoulder_pan_joint", "r_wrist_roll_link");
#endif
        Chain rRobotArmChain = constructRobotArmChain("l_shoulder_pan_joint", "l_wrist_roll_link");

        // Limit pitch and roll to pi/4 but allow any yaw.
        boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > pitchRollGenerator(rng, boost::uniform_real<double>(-boost::math::constants::pi<double>() / 4.0, boost::math::constants::pi<double>() / 4.0));

        boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > yawGenerator(rng, boost::uniform_real<double>(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>()));

       // Save the initial angles
       SetJointAngles setInitialLeftAngles(getAngles(leftHip, leftFoot));
       SetJointAngles setInitialRightAngles(getAngles(rightHip, rightFoot));

        bool leftArmMoved = false;
        bool rightArmMoved = false;
        bool foundLegalConfig = false;
        bool adjacent = true;
        unsigned int attempts = 0;
        unsigned int ikAttempts = 0;
        bool humanOnly = model->GetLink("l_wrist_roll_link") == nullptr;

        while (!foundLegalConfig) {
           attempts++;
           // Reset all angles
           setInitialLeftAngles(leftHip, leftFoot);
           setInitialRightAngles(rightHip, rightFoot);

           double roll = pitchRollGenerator();
           double pitch = pitchRollGenerator();
           double yaw = yawGenerator();

            math::Vector3 trunkPosition = trunk->GetWorldPose().pos;
            setWorldPoseIncludingChildren(trunk, math::Pose(trunkPosition, math::Quaternion(roll, pitch, yaw)));

            // Confirmation cartesian position did not change
            assert(trunk->GetWorldPose().pos == trunkPosition);

            // Check that the orientation was applied correctly
            assert(trunk->GetWorldPose().rot == math::Quaternion(roll, pitch, yaw));

            // Reconstruct kinematic chains with new orientation
            leftLeg = constructChain(leftHip, leftFoot);
            rightLeg = constructChain(rightHip, rightFoot);

            // Now check that FK still is correct
            assert(checkFK(leftLeg, leftFoot, leftHip, getAngles(leftHip, leftFoot), leftFoot->GetWorldPose(), false));
            assert(checkFK(rightLeg, rightFoot, rightHip, getAngles(rightHip, rightFoot), rightFoot->GetWorldPose(), false));

            // Set random RPY on arms
            SetRandomAngles randomAngleSetter(rng);
            randomAngleSetter(model->GetJoint("left_shoulder"), model->GetLink("left_hand"));
            randomAngleSetter(model->GetJoint("right_shoulder"), model->GetLink("right_hand"));

            // Now pick a leg to set randomly
            boost::bernoulli_distribution<> randomLeg(0.5);
            physics::LinkPtr contactLink;
            if(randomLeg(rng)) {
                // Set right leg randomly
                randomAngleSetter(rightHip, rightFoot);

                // Select the position equal to the current planar position of the foot at zero height
                // TODO: Compensate for difference between foot center and tip
                contactLink = leftFoot;
                leftFootPose.pos.z = 0;
                vector<double> angles = calcInverse(leftLeg, leftHip, leftFoot, transformGlobalToJoint(leftFootPose, leftHip));
                // Check for IK failure
                if(angles.size() == 0) {
                    // Next config
                    continue;
                }

               // Cannot easily check FK here because the calculation of the tip from the centroid
               // relies on the current pose.

                // Now apply joint angles to a chain
                SetJointAngles setAngles(angles);
                setAngles(leftHip, leftFoot);
                assert(checkFK(leftLeg, leftFoot, leftHip, getAngles(leftHip, leftFoot), leftFootPose, true));

                // Check whether the target posed was reached
                assert(pos_rough_eq(leftFootPose.pos, (getCenterToTip(leftFoot) + leftFoot->GetWorldPose()).pos));
            }
            else {
                // Set left leg randomly
                randomAngleSetter(leftHip, leftFoot);

                // Use IK for right leg

                // Select the position equal to the current planar position of the foot at zero height
                // TODO: Compensate for difference between foot center and tip
                // TODO: Make function to get foot tip location
                contactLink = rightFoot;
                rightFootPose.pos.z = 0;

                vector<double> angles = calcInverse(rightLeg, rightHip, rightFoot, transformGlobalToJoint(rightFootPose, rightHip));

                // Check for IK failure
                if(angles.size() == 0) {
                    // Next config
                    continue;
                }

                // Now apply joint angles to a chain
                SetJointAngles setAngles(angles);
                setAngles(rightHip, rightFoot);
                assert(checkFK(rightLeg, rightFoot, rightHip, getAngles(rightHip, rightFoot), rightFootPose, true));

                // Check whether the target posed was reached
                assert(pos_rough_eq(rightFootPose.pos, (getCenterToTip(rightFoot) + rightFoot->GetWorldPose()).pos));
            }

           // PR2 model is strange and it is difficult to find all the parts of the end effector
           vector<physics::LinkPtr> endEffectors;
           endEffectors.push_back(model->GetLink("r_gripper_l_parallel_link"));
           endEffectors.push_back(model->GetLink("r_wrist_roll_link"));
           endEffectors.push_back(model->GetLink("r_gripper_r_parallel_link"));
           endEffectors.push_back(model->GetLink("l_gripper_l_parallel_link"));
           endEffectors.push_back(model->GetLink("l_wrist_roll_link"));
           endEffectors.push_back(model->GetLink("l_gripper_r_parallel_link"));

            // Check for intersection
            if(hasCollision(model, trunk, contactLink, endEffectors)) {
#if(PRINT_DEBUG)
                cout << "Human has self or ground collision" << endl;
#endif
            }
            else if (humanOnly) {
               foundLegalConfig = true;
            }
            else {
               ikAttempts++;
               // Determine the minimum distance configuration for each robot hand.
               adjacent = (linkToJointDistance("r_wrist_roll_link", "right_hip") + linkToJointDistance("l_wrist_roll_link", "left_hip")
                                < linkToJointDistance("r_wrist_roll_link", "left_hip") + linkToJointDistance("l_wrist_roll_link", "right_hip"));

                                rightArmMoved = moveRobotArm(rRobotArmChain, "r_shoulder_pan_joint" , "r_wrist_roll_link", adjacent ? "right_hip" : "left_hip");

#if(ENABLE_SECOND_ARM)
                                leftArmMoved = moveRobotArm(lRobotArmChain, "l_shoulder_pan_joint" , "l_wrist_roll_link", adjacent ? "left_hip" : "right_hip");
#endif
               if(leftArmMoved || rightArmMoved) {
                  // Recheck collision
                  if(hasCollision(model, trunk, contactLink, endEffectors)) {
#if(PRINT_DEBUG)
                     cout << "Human has self or ground collision after IK" << endl;
#endif
                  }
                  else {
                     foundLegalConfig = true;
#if(PRINT_DEBUG)
                     cout << "Found a legal configuration" << endl;
#endif
                  }
               }
           }
        }

        // Setup the virtual joints if needed
       if(leftArmMoved){
          connectVirtualJoint("l_gripper_r_finger_link", adjacent ? "left_thigh" : "right_thigh");
       } else if (!humanOnly) {
          // Move left arm out of the way
          bool set = model->GetJoint("l_shoulder_lift_joint")->SetPosition(0, boost::math::constants::pi<double>() / 2.0);
          assert(set);
          set = model->GetJoint("l_elbow_flex_joint")->SetPosition(0, -3 * boost::math::constants::pi<double>() / 4.0);
          assert(set);
       }
       if(rightArmMoved){
          connectVirtualJoint("r_gripper_r_finger_link", adjacent ? "right_thigh" : "left_thigh");
       } else if (!humanOnly){
          // Move right arm out of the way
          bool set = model->GetJoint("r_shoulder_lift_joint")->SetPosition(0, boost::math::constants::pi<double>() / 2.0);
          assert(set);
          set = model->GetJoint("r_elbow_flex_joint")->SetPosition(0, -3 * boost::math::constants::pi<double>() / 4.0);
          assert(set);
       }

       csvFile << seed << "," << leftArmMoved << "," << rightArmMoved << "," << attempts << "," << ikAttempts << ",";
       // Write all joint angles
       for (unsigned int i = 0; i < boost::size(joints); ++i) {
          physics::JointPtr currJoint = model->GetJoint(joints[i]);
          for (unsigned int j = 0; j < currJoint->GetAngleCount(); ++j) {
             csvFile << model->GetJoint(joints[i])->GetAngle(j).Radian() << ",";
          }
       }

        csvFile << endl;
        csvFile.close();
    }

};
GZ_REGISTER_MODEL_PLUGIN(InitialJointPositionPlugin)
}
