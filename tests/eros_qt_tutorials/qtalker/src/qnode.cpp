/**
 * @file /eros_qtalker/src/qnode.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 25/02/2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include "../include/qnode.hpp"
#include <std_msgs/String.h>
#include <sstream>

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
	ros::shutdown();
	std::cout << "Waiting for ros thread to finish." << std::endl;
	wait();
}
void QNode::init(const std::string &master_url, const std::string &host_url, const std::string &topic_name) {
	// we need a check box to flick between manual and environmental settings.
	//	ros::init(init_argc,init_argv,"qtalker");
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"talker");
    ros::NodeHandle n;
    chatter_publisher = n.advertise<std_msgs::String>(topic_name, 1000);
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

	    std_msgs::String msg;
	    std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());

		outgoing.insertRows(0,1);
		std::stringstream outgoing_msg;
		outgoing_msg << "[ INFO] [" << ros::Time::now() << "]: " << msg.data;
		QVariant new_row(QString(outgoing_msg.str().c_str()));
		outgoing.setData(outgoing.index(0),new_row);

		chatter_publisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

}
