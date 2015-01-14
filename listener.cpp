/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>

#include <iostream>
#include <fstream>

using namespace std;
using namespace gazebo;

// Global variables
std::map<std::string, double> maxForces;
ofstream outputCSV;
gazebo::transport::NodePtr node;

// List of topics to listen on
static const std::string topics[] = {
    "~/human/trunk/trunk_contact",
    "~/human/left_foot/left_foot_contact",
    "~/human/right_foot/right_foot_contact",
    "~/human/left_leg/left_leg_contact",
    "~/human/right_leg/right_leg_contact",
    "~/human/left_thigh/left_thigh_contact",
    "~/human/right_thigh/right_thigh_contact",
    "~/human/transpelvic_link/transpelvic_contact",
    "~/human/clavicular_link/clavicular_link_contact",
    "~/human/head_neck/head_neck_contact",
    "~/human/left_upper_arm/left_upper_arm_contact",
    "~/human/right_upper_arm/right_upper_arm_contact",
    "~/human/left_forearm/left_forearm_contact",
    "~/human/right_forearm/right_forearm_contact",
    "~human/left_hand/left_hand_contact",
    "~/human/right_hand/right_hand_contact"
};

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstContactsPtr &_msg)
{
  // Dump the message contents to stdout.
  // std::cout << _msg->DebugString();

  for(unsigned int i = 0; i < _msg->contact_size(); ++i){
    for(unsigned int j = 0; j < _msg->contact(i).wrench_size(); j++){
      // The human link should always be body1
      const gazebo::msgs::JointWrench& jw = _msg->contact(i).wrench(j);
      std::cout << "Contact between: " << jw.body_1_name() << " and " << jw.body_2_name() << std::endl;
      const gazebo::msgs::Wrench& w = jw.body_1_wrench();
      const gazebo::msgs::Vector3d& force = w.force();
      const gazebo::math::Vector3 v = gazebo::math::Vector3(force.x(), force.y(), force.z());
      double magnitude = v.GetLength();
      std::cout << "Body1 wrench: " << magnitude << std::endl;
      maxForces[jw.body_1_name()] = std::max(maxForces[jw.body_1_name()], magnitude); 
    }    
  }
}

void subscribeToContactTopics(){
  // Keep a reference to all subscriptions
  std::vector<gazebo::transport::SubscriberPtr> subs;
  
  for(unsigned int i = 0; i < boost::size(topics); ++i){
    // Register for topic
    subs.push_back(node->Subscribe(topics[i], cb));
  }
}

void finish(){
  // Make sure to shut everything down.
  // gazebo::shutdown();
  // 1.9 fix
  gazebo::fini();
  gazebo::stop();
  // End 1.9 fix

  // Print max forces
  double overallMax = 0;
  std::string overallMaxLink;
  for(unsigned int i = 0; i < boost::size(topics); ++i){
      std::cout << topics[i] << ": " << maxForces[topics[i]] << "(N)" << std::endl;
      outputCSV << maxForces[topics[i]] << ", ";
      if(maxForces[topics[i]] > overallMax){
        overallMax = maxForces[topics[i]];
        overallMaxLink = topics[i];
      }
    }

    std::cout << "Maximum force: " << std::endl;
    std::cout << overallMaxLink << ": " << overallMax << "(N)" << std::endl;
    outputCSV << overallMaxLink << ", " << overallMax << endl;
    outputCSV.close();
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Open the output file
  outputCSV.open("../output.csv", ios::out | ios::app);
  assert(outputCSV.is_open());

  // Load gazebo
  // gazebo::setupClient(_argc, _argv);
  // 1.9 fix
  gazebo::load();
  gazebo::init();
  gazebo::run();
  // End 1.9 fix

  // Create our node for communication
  node.reset(new gazebo::transport::Node());
  node->Init();
  
  // The world starts paused. When we come online, register for all our topics.
  subscribeToContactTopics();
  
  // Now unpause
  // Create a publisher on the ~/world_control topic
  transport::PublisherPtr worldControlPub = node->Advertise<msgs::WorldControl>("/gazebo/default/world_control");

  cout << "Sending world control message" << endl;
  msgs::WorldControl controlMsg;
  controlMsg.set_pause(false);
  worldControlPub->Publish(controlMsg);
    
  // Wait for 10 seconds.
  // TODO: Use a sim trigger here
  for(unsigned int i = 0; i < 15; ++i){
    gazebo::common::Time::MSleep(1000);
  }
  
  cout << "Completing contact sension" << endl;
  finish();
  exit(0);
}

