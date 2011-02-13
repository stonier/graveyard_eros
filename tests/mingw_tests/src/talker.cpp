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
#include <ros/poll_set.h>
#include <ros/network.h>

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	// This needs to happen before we start fooling around with logger levels.  Otherwise the level we set may be overwritten by
	// a configuration file
	ROSCONSOLE_AUTOINIT;
	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
	log4cxx::LoggerPtr ros_logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
	my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]); // Debug for this package only
	ros_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
	ros::console::notifyLoggerLevelsChanged(); // update loging level status

#if defined(WIN32)

//	ros::master::setURI("http://192.168.10.66:11311/");
//	ros::network::setHost("192.168.10.67");
	ros::master::setURI("http://192.168.1.3:11311/");
	ros::network::setHost("192.168.1.67");
//	ros::master::setURI("http://192.168.56.1:11311/");
//	ros::network::setHost("192.168.56.2");
#endif
	//   ros::init(argc, argv, "talker",ros::init_options::NoRosout);
	ros::init(argc, argv, "talker");
	ros::Time::init();

//	ros::V_string nodes;
//	if ( ros::master::getNodes(nodes) ) {
//		std::cout << "Node list:" << std::endl;
//		for (unsigned int i = 0; i < nodes.size(); ++i ) {
//			std::cout << "  " << nodes[i] << std::endl;
//		}
//	} else {
//		std::cout << "Couldn't contact the master" << std::endl;
//	}
//	ros::start();
    ros::NodeHandle n;
	ROS_INFO("Dude");
	ros::Duration(3,0).sleep();
	ros::Rate loop_rate(1);

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	int count = 0;
	while (ros::ok() && (count < 4) ) {
	    std_msgs::String msg;
	    std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	ROS_INFO("Shutdown commencing");
	ros::shutdown();


  return 0;
}
