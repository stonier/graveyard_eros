/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ros/io.h"
#include <ros/assert.h> // don't need if we dont call the pipe functions.
#include <errno.h> // for EFAULT and co.
#include <iostream>
#include <sstream>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros {

/**
 * Returns 0 if it successfully initialised the socket subsystem,
 * nonzero otherwise.
 */
int init_sockets() {
	#ifdef ECL_IS_WIN
		struct WSAData wsaData;
		int err;
		/* Initiates use of the Winsock DLL by a process. */
		if ( (err = WSAStartup(MAKEWORD(2, 0), &wsaData)) != 0) {
			return err;
		}
	#endif
	return 0;
}
std::string last_socket_error_string() {
	#ifdef WIN32
		// fix this to use FORMAT_MESSAGE and print a real string later.
		std::stringstream ostream;
		ostream << "WSA Error: " << WSAGetLastError();
		return ostream.str();
	#else
		return strerror(errno);
	#endif
}

/*****************************************************************************
** Service Robotics/Libssh Functions
*****************************************************************************/
/**
 * @brief A cross platform polling function for sockets.
 *
 * Windows doesn't have a polling function until Vista (WSAPoll) and even then
 * its implementation is not supposed to be great. This works for a broader
 * range of platforms and will suffice.
 * @param fds - the socket set (descriptor's and events) to poll for.
 * @param nfds - the number of descriptors to poll for.
 * @param timeout - timeout in milliseconds.
 * @return int : -1 on error, 0 on timeout, +ve number of structures with received events.
 */
int poll_sockets(socket_pollfd *fds, nfds_t nfds, int timeout) {
#if defined(WIN32)
	fd_set readfds, writefds, exceptfds;
	struct timeval tv, *ptv;
	int max_fd;
	int rc;
	nfds_t i;

	if (fds == NULL) {
		errno = EFAULT;
		return -1;
	}

	FD_ZERO (&readfds);
	FD_ZERO (&writefds);
	FD_ZERO (&exceptfds);

	/*********************
	** Compute fd sets
	**********************/
	// also find the largest descriptor.
	for (rc = -1, max_fd = 0, i = 0; i < nfds; i++) {
		if (fds[i].fd == ROS_INVALID_SOCKET) {
			continue;
		}
//			#ifndef WIN32
//			if (fds[i].fd >= FD_SETSIZE) {
//				rc = -1;
//				break;
//			}
//			#endif

		if (fds[i].events & (POLLIN | POLLRDNORM)) {
			FD_SET (fds[i].fd, &readfds);
		}
		if (fds[i].events & (POLLOUT | POLLWRNORM | POLLWRBAND)) {
			FD_SET (fds[i].fd, &writefds);
		}
		if (fds[i].events & (POLLPRI | POLLRDBAND)) {
			FD_SET (fds[i].fd, &exceptfds);
		}
		if (fds[i].fd > max_fd &&
			  (fds[i].events & (POLLIN | POLLOUT | POLLPRI |
								POLLRDNORM | POLLRDBAND |
								POLLWRNORM | POLLWRBAND))) {
			max_fd = fds[i].fd;
			rc = 0;
		}
	}

	if (rc == -1) {
		errno = EINVAL;
		return -1;
	}
	/*********************
	** Setting the timeout
	**********************/
	if (timeout < 0) {
		ptv = NULL;
	} else {
		ptv = &tv;
		if (timeout == 0) {
			tv.tv_sec = 0;
			tv.tv_usec = 0;
		} else {
			tv.tv_sec = timeout / 1000;
			tv.tv_usec = (timeout % 1000) * 1000;
		}
	}

	rc = select (max_fd + 1, &readfds, &writefds, &exceptfds, ptv);
	if (rc < 0) {
		return -1;
	} else if ( rc == 0 ) {
		return 0;
	} else {
//			std::cout << "select passed." << std::endl;
	}

	for (rc = 0, i = 0; i < nfds; i++) {
		if (fds[i].fd >= 0) {
			fds[i].revents = 0;

			if (FD_ISSET(fds[i].fd, &readfds)) {
				int save_errno = errno;
				char data[64] = {0};
				int ret;

				/* support for POLLHUP */
				// just check if there's incoming data, without removing it from the queue.
				ret = recv(fds[i].fd, data, 64, MSG_PEEK);
				#ifdef WIN32
				if ((ret == -1) &&
						(errno == WSAESHUTDOWN || errno == WSAECONNRESET ||
						(errno == WSAECONNABORTED) || errno == WSAENETRESET))
				#else
				if ((ret == -1) &&
						(errno == ESHUTDOWN || errno == ECONNRESET ||
						(errno == ECONNABORTED) || errno == ENETRESET))
				#endif
				{
					fds[i].revents |= POLLHUP;
				} else {
					fds[i].revents |= fds[i].events & (POLLIN | POLLRDNORM);
				}
				errno = save_errno;
			}
			if (FD_ISSET(fds[i].fd, &writefds)) {
				fds[i].revents |= fds[i].events & (POLLOUT | POLLWRNORM | POLLWRBAND);
			}

			if (FD_ISSET(fds[i].fd, &exceptfds)) {
				fds[i].revents |= fds[i].events & (POLLPRI | POLLRDBAND);
			}

			if (fds[i].revents & ~POLLHUP) {
				rc++;
			}
		} else {
				fds[i].revents = POLLNVAL;
		}
	}
	return rc;
#else // HAVE_POLL implementations
	int result = poll(&ufds_.front(), ufds_count, poll_timeout);
	// EINTR means that we got interrupted by a signal, and is not an error
	if ( result < 0 ) {
		if(errno != EINTR) {
			result = 0;
		}
	}
	return result;
#endif // poll_sockets functions
}
/*****************************************************************************
** Signal Pair
*****************************************************************************/
/**
 * This code is primarily from the msdn socket tutorials.
 * @param signal_pair : a pair of sockets linked to each other over localhost.
 * @return 0 on success, -1 on failure.
 */
int create_signal_pair(int signal_pair[2]) {
#ifdef WIN32 // use a socket pair
	/*********************
	** Notes
	**********************/
	// Using port 21 - is this a problem always using this port?

	/******************************************
	** Server
	*******************************************/
	struct addrinfo *result = NULL, *ptr = NULL, hints;

	// windows sockets can't do AF_LOCAL
	ZeroMemory(&hints, sizeof (hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	// AI_PASSIVE flag indicates the caller intends to use the
	// returned socket address structure in a call to the bind function.
	// When the AI_PASSIVE flag is set and nodename parameter to the
	// getaddrinfo function is a NULL pointer, the IP address portion
	// of the socket address structure is set to INADDR_ANY for IPv4
	// addresses or IN6ADDR_ANY_INIT for IPv6 addresses.
	hints.ai_flags = AI_PASSIVE;

	// Resolve the local address and port to be used by the server
	// first argument is the nodename parameter as described above.
	int wsock_error = getaddrinfo(NULL, "21", &hints, &result);
	if (wsock_error != 0) {
		return -1;
	}

	/*********************
	** Server Socket
	**********************/
	SOCKET listen_socket = INVALID_SOCKET;
	// Create a SOCKET for the server to listen for client connections
	listen_socket = ::socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (listen_socket == INVALID_SOCKET) {
	    freeaddrinfo(result);
	    return -1;
	}

	/*********************
	** Bind Server
	**********************/
	// Bind server to an address.
	wsock_error = bind( listen_socket, result->ai_addr, (int)result->ai_addrlen);
	if (wsock_error == SOCKET_ERROR) {
		freeaddrinfo(result);
		::closesocket(listen_socket);
		return -1;
	}
	freeaddrinfo(result);

	/*********************
	** Listen for client
	**********************/
	if ( listen( listen_socket, SOMAXCONN ) == SOCKET_ERROR ) {
	    closesocket(listen_socket);
	    return -1;
	}
	signal_pair[0] = INVALID_SOCKET;

	/******************************************
	** Client
	*******************************************/
	ZeroMemory( &hints, sizeof(hints) );
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	// Resolve the server address and port
	wsock_error = getaddrinfo("localhost", "21", &hints, &result);
	if (wsock_error != 0) {
	    ::closesocket(listen_socket);
	    return -1;
	}
	signal_pair[1] = INVALID_SOCKET;

	/*********************
	** Create client
	**********************/
	ptr=result;
	signal_pair[1] = ::socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
	if (signal_pair[1] == INVALID_SOCKET) {
	    ::closesocket(listen_socket);
	    freeaddrinfo(result);
	    return -1;
	}

	/*********************
	** Connect to server
	**********************/
	wsock_error = connect( signal_pair[1], ptr->ai_addr, (int)ptr->ai_addrlen);
	if (wsock_error == SOCKET_ERROR) {
	    ::closesocket(listen_socket);
	    closesocket(signal_pair[1]);
	    signal_pair[1] = INVALID_SOCKET;
	    return -1;
	}

	// Should really try the next address returned by getaddrinfo
	// if the connect call failed
	// But for this simple example we just free the resources
	// returned by getaddrinfo and print an error message

	freeaddrinfo(result);

	if (signal_pair[1] == INVALID_SOCKET) {
	    return -1;
	}

	/*********************
	** Accept connection
	**********************/
	signal_pair[0] = accept(listen_socket, NULL, NULL);
	if (signal_pair[0] == INVALID_SOCKET) {
	    ::closesocket(listen_socket);
	    ::closesocket(signal_pair[1]);
	    return -1;
	}

	/*********************
	** Nonblocking
	**********************/
	unsigned long non_blocking_flag = 1;
	if(ioctlsocket( signal_pair[0], FIONBIO, &non_blocking_flag ) != 0 ) {
		::closesocket(listen_socket);
		::closesocket(signal_pair[0]);
		::closesocket(signal_pair[1]);
		return -1;
	}
	if(ioctlsocket( signal_pair[1], FIONBIO, &non_blocking_flag ) != 0 ) {
		::closesocket(listen_socket);
		::closesocket(signal_pair[0]);
		::closesocket(signal_pair[1]);
		return -1;
	}
	return 0;
#else // use a pipe pair
	if(pipe(signal_pair) != 0) {
		ROS_FATAL( "pipe() failed");
		return -1;
	}
	if(fcntl(signal_pair[0], F_SETFL, O_NONBLOCK) == -1) {
		ROS_FATAL( "fcntl() failed");
		return -1;
	}
	if(fcntl(signal_pair[1], F_SETFL, O_NONBLOCK) == -1) {
		ROS_FATAL( "fcntl() failed");
		return -1;
	}
	return 0;
#endif // create_pipe
}

} // namespace ros
