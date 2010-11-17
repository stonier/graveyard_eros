/**
 * @file /eros_boost/src/shared_pointer.cpp
 *
 * @brief Test the boost compilation/install.
 *
 * @date Nov 17, 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <boost/shared_ptr.hpp>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
	boost::shared_ptr<int> ptr(new int(3));
	std::cout << "Int: " << *ptr << std::endl;
	return 0;
}

