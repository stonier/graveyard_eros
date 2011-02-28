/**
 * @file /eros_qtalker/src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
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
//	ros::shutdown();
	std::cout << "Waiting for ros thread to finish." << std::endl;
	wait();
}

void QNode::init(const std::string &topic_name) {
	ros::init(init_argc,init_argv,"add_two_ints_client");
    ros::NodeHandle n;
    add_client = n.serviceClient<eros_qt_tutorials::TwoInts>("add_two_ints");
	start();
}

void QNode::init(const std::string &master_url, const std::string &host_url, const std::string &topic_name) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"add_two_ints_client");
    ros::NodeHandle n;
    add_client = n.serviceClient<eros_qt_tutorials::TwoInts>("add_two_ints");
	start();
}

void QNode::run() {
	ros::Rate loop_rate(1);
	eros_qt_tutorials::TwoInts srv;
	srv.response.sum = 0;
	int count = 1;
	while( count < 5 ) {
//	while( ros::ok() ) { // is this purely a problem because of that log4cxx appender?
		srv.request.a = count;
		srv.request.b = srv.response.sum;
		if ( add_client.call(srv) ) {
			ROS_INFO_STREAM("Sum: " << srv.response.sum);
		}
		logging.insertRows(0,1);
		std::stringstream logging_msg;
		logging_msg << "[ INFO] [" << ros::Time::now() << "]: " << srv.request.a << " + " << srv.request.b << " = " << srv.response.sum;
		QVariant new_row(QString(logging_msg.str().c_str()));
		logging.setData(logging.index(0),new_row);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

}

