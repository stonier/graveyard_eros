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
void QNode::init(const std::string &master_url, /* const std::string &host_url, */ const std::string &topic_name) {
	ros::master::setURI(master_url); //	ros::network::setHost(host_url);
	ros::init(init_argc,init_argv,"qlistener");
    ros::NodeHandle n;
	chatter_subscriber = n.subscribe(topic_name, 1000, &QNode::chatterCallback, this);
	start();
}

void QNode::run() {
		ros::spin();
}

void QNode::chatterCallback(const std_msgs::String::ConstPtr &msg) {
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	incoming.insertRows(0,1);
	std::stringstream incoming_msg;
	incoming_msg << "[ INFO] [" << ros::Time::now() << "]: I heard: " << msg->data;
	QVariant new_row(QString(incoming_msg.str().c_str()));
	incoming.setData(incoming.index(0),new_row);
}