/**
 * @file /mingw_tests/src/common.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Feb 13, 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tests {

/*****************************************************************************
** Functions
*****************************************************************************/

void set_debug_log_levels() {
	// This needs to happen before we start fooling around with logger levels.  Otherwise the level we set may be overwritten by
	// a configuration file
	ROSCONSOLE_AUTOINIT;
	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
	log4cxx::LoggerPtr ros_logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
	my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]); // Debug for this package only
	ros_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
	ros::console::notifyLoggerLevelsChanged(); // update loging level status
}

void set_urls() {
#if defined(WIN32)
	ros::master::setURI("http://192.168.10.66:11311/");
	ros::network::setHost("192.168.10.67");
//	ros::master::setURI("http://192.168.1.3:11311/");
//	ros::network::setHost("192.168.1.67");
//	ros::master::setURI("http://192.168.56.1:11311/");
//	ros::network::setHost("192.168.56.2");
#endif
}

}
