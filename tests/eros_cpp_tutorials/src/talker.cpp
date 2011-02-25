/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "common.h"

#include <sstream>

/*****************************************************************************
** Main
*****************************************************************************/

/**
 * Use to test pubsubs with the listener.
 */
int main(int argc, char **argv)
{
	tests::set_urls();
//	tests::set_debug_log_levels();
//	ros::init(argc, argv, "talker",ros::init_options::NoRosout);
	ros::init(argc, argv, "talker");
    ros::NodeHandle n;

	ros::Rate loop_rate(100);

    ROS_INFO("Setting up publisher");
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Duration(1,000000000).sleep(); // give it some time to make the subscriber connection.
    ROS_INFO("Commencing Loop.");
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
	ros::Duration(1,0).sleep();
//	ROS_INFO("Shutdown commencing");
///	ros::shutdown();


  return 0;
}
