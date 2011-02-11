/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/poll_set.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	if ( ros::init_sockets() != 0 ) {
		std::cout << "Failed to initialise socket subsystem." << std::endl;
		return -1;
	} else {
		std::cout << "Initialised the socket subsystem." << std::endl;
	}

	ros::master::setURI("http://192.168.10.66:11311/");
	ros::master::setHost("192.168.10.67");
	//   ros::init(argc, argv, "talker",ros::init_options::NoRosout);
	ros::init(argc, argv, "talker");
	ros::start();

	ROS_INFO("Dude");
	Sleep(1000);
	ROS_INFO("Dude");

//  ros::NodeHandle n;
//  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
//  ros::Rate loop_rate(10);
//  int count = 0;
//  while (ros::ok() && (count < 4) )
//  {
//	  Sleep(1000);
//	  ROS_INFO_STREAM("Dude: " << count);
//	  ++count;
//    ros::spinOnce();
//  }
//  ros::shutdown();
//    std_msgs::String msg;
//    std::stringstream ss;
//    ss << "hello world " << count;
//    msg.data = ss.str();
//    ROS_INFO("%s", msg.data.c_str());
//    chatter_pub.publish(msg);
//    ros::spinOnce();
//    loop_rate.sleep();
//    ++count;
//  }


  return 0;
}
// %EndTag(FULLTEXT)%
