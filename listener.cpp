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

std::map<std::string, double> maxForces;

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

void signalHandler(int signum){
  // Make sure to shut everything down.
  // gazebo::shutdown();
  // 1.9 fix
  gazebo::fini();
  gazebo::stop();
  // End 1.9 fix

  // Print max forces
  double overallMax = 0;
  std::string overallMaxLink;
  for(std::map<std::string, double>::const_iterator it = maxForces.begin();
    it != maxForces.end(); ++it)
    {
      std::cout << it->first << ": " << it->second << "(N)" << std::endl;
      if(it->second > overallMax){
        overallMax = it->second;
        overallMaxLink = it->first;
      }
    }

    std::cout << "Maximum force: " << std::endl;
    std::cout << overallMaxLink << ": " << overallMax << "(N)" << std::endl;
    exit(signum);
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  // gazebo::setupClient(_argc, _argv);
  // 1.9 fix
  gazebo::load();
  gazebo::init();
  gazebo::run();
  // End 1.9 fix

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // List of topics to listen on
  const std::string topics[] = {
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
    "/gazebo/default/human/right_hand/right_hand_contact",
     "/gazebo/default/combined_human_pr2/trunk/trunk_contact",
     "/gazebo/default/combined_human_pr2/left_foot/left_foot_contact",
     "/gazebo/default/combined_human_pr2/right_foot/right_foot_contact",
     "/gazebo/default/combined_human_pr2/left_leg/left_leg_contact",
     "/gazebo/default/combined_human_pr2/right_leg/right_leg_contact",
     "/gazebo/default/combined_human_pr2/left_thigh/left_thigh_contact",
     "/gazebo/default/combined_human_pr2/right_thigh/right_thigh_contact",
     "/gazebo/default/combined_human_pr2/transpelvic_link/transpelvic_contact",
     "/gazebo/default/combined_human_pr2/clavicular_link/clavicular_link_contact",
     "/gazebo/default/combined_human_pr2/head_neck/head_neck_contact",
     "/gazebo/default/combined_human_pr2/left_upper_arm/left_upper_arm_contact",
     "/gazebo/default/combined_human_pr2/right_upper_arm/right_upper_arm_contact",
     "/gazebo/default/combined_human_pr2/left_forearm/left_forearm_contact",
     "/gazebo/default/combined_human_pr2/right_forearm/right_forearm_contact",
     "/gazebo/default/combined_human_pr2/left_hand/left_hand_contact",
     "/gazebo/default/combined_human_pr2/right_hand/right_hand_contact"
  };

  // Keep a reference to all subscriptions
  std::vector<gazebo::transport::SubscriberPtr> subs;
  
  for(unsigned int i = 0; i < boost::size(topics); ++i){
    // Register for topic
    subs.push_back(node->Subscribe(topics[i], cb));
  }

  signal(SIGINT, signalHandler);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);
}
