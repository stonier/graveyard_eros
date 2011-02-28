/**
 * @file /include/%(package)s/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef %(package)s_QNODE_HPP_
#define %(package)s_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace %(package)s {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
public:
	QNode(int argc, char** argv );
	~QNode();
	void init(const std::string &topic_name);
	void init(const std::string &master_url, const std::string &host_url, const std::string &topic_name);
	void run();

	QStringListModel* loggingModel() { return &logging_model; }

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
};

}  // namespace %(package)s

#endif /* %(package)s_QNODE_HPP_ */
