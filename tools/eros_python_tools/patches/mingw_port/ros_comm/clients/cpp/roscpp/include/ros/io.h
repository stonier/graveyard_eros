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
#ifndef ROSCPP_POLL_H_
#define ROSCPP_POLL_H_

/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef WIN32
	#define WIN32
#endif
#ifndef WIN32
	#define HAVE_POLL // better to get cmake to catch this for us later.
#endif

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>

#ifdef HAVE_POLL
	#include <poll.h>
#else
	#ifdef WIN32
		#include <winsock2.h> // For struct timeval
		#include <ws2tcpip.h> // After winsock2.h because MS didn't put proper inclusion guards in their headers.
		#include <sys/types.h>
		#include <io.h>
		#include <fcntl.h>
		#include <process.h> // for _getpid
	#else
		#include <sys/poll.h>
		#include <arpa/inet.h>
		#include <netdb.h>
	#endif
#endif

/*****************************************************************************
** Macros
*****************************************************************************/

#ifdef WIN32
//  #include <process.h>
//  #define pid_t int
  #define getpid _getpid
#endif
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros {

/*****************************************************************************
** Cross Platform Macros
*****************************************************************************/

/* Macro emulation */
#ifndef HAVE_POLL
	#ifdef WIN32
		#ifndef POLLRDNORM
			#define POLLRDNORM  0x0100
		#endif
		#ifndef POLLRDBAND
			#define POLLRDBAND  0x0200
		#endif
		#ifndef POLLIN
			#define POLLIN      (POLLRDNORM | POLLRDBAND)
		#endif
		#ifndef POLLPRI
			#define POLLPRI     0x0400
		#endif

		#ifndef POLLWRNORM
			#define POLLWRNORM  0x0010
		#endif
		#ifndef POLLOUT
			#define POLLOUT     (POLLWRNORM)
		#endif
		#ifndef POLLWRBAND
			#define POLLWRBAND  0x0020
		#endif
		#ifndef POLLERR
			#define POLLERR     0x0001
		#endif
		#ifndef POLLHUP
			#define POLLHUP     0x0002
		#endif
		#ifndef POLLNVAL
			#define POLLNVAL    0x0004
		#endif
	#else
		#ifndef POLLIN
			#define POLLIN    0x001  /* There is data to read.  */
		#endif
		#ifndef POLLPRI
			#define POLLPRI   0x002  /* There is urgent data to read.  */
		#endif
		#ifndef POLLOUT
			#define POLLOUT   0x004  /* Writing now will not block.  */
		#endif

		#ifndef POLLERR
			#define POLLERR   0x008  /* Error condition.  */
		#endif
		#ifndef POLLHUP
			#define POLLHUP   0x010  /* Hung up.  */
		#endif
		#ifndef POLLNVAL
			#define POLLNVAL  0x020  /* Invalid polling request.  */
		#endif

		#ifndef POLLRDNORM
			#define POLLRDNORM  0x040 /* mapped to read fds_set */
		#endif
		#ifndef POLLRDBAND
			#define POLLRDBAND  0x080 /* mapped to exception fds_set */
		#endif
		#ifndef POLLWRNORM
			#define POLLWRNORM  0x100 /* mapped to write fds_set */
		#endif
		#ifndef POLLWRBAND
			#define POLLWRBAND  0x200 /* mapped to write fds_set */
		#endif
	#endif
#endif /* Macro emulation */

#define ROS_INVALID_SOCKET ((int) -1)

/* Some systems dont define this constant in the system headers */
#ifndef NFDBITS
	#define	NFDBITS (8 * sizeof(unsigned long))
#endif

/*****************************************************************************
** Cross Platform Types
*****************************************************************************/

typedef unsigned long int nfds_t;

/*****************************************************************************
** Cross Platform Structures
*****************************************************************************/

#ifdef HAVE_POLL
  typedef struct pollfd socket_pollfd;
#else
  /* poll emulation support */

  typedef struct socket_pollfd {
    int fd;      /* file descriptor */
    short events;     /* requested events */
    short revents;    /* returned events */
  } socket_pollfd;

  typedef unsigned long int nfds_t;
#endif

/*****************************************************************************
** Functions
*****************************************************************************/

int init_sockets(); // actually not needed since xmlrpcpp has its own version it autostarts.
std::string last_socket_error_string();
int poll_sockets(socket_pollfd *fds, nfds_t nfds, int timeout);
int create_signal_pair(int signal_pair[2]);

/*****************************************************************************
** Hackery
*****************************************************************************/

} // namespace ros

#endif /* ROSCPP_POLL_H_ */

//#ifndef WIN32
//# include <poll.h>
//#else
//// All code from here until the end of this #ifdef came from Player's poll
//// replacement, which in turn came, via Brian, from glibc. As glibc is licensed
//// under the GPL, this is almost certainly a licensing problem. A better
//// solution is to replace the call to poll() above with whatever boost has to
//// replace it (there's got to be something in there).
////
//// See also poll_set.cpp for more win32 errata.
//
///* Event types that can be polled for.  These bits may be set in `events'
//   to indicate the interesting event types; they will appear in `revents'
//   to indicate the status of the file descriptor.  */
//# define POLLIN          01              /* There is data to read.  */
//# define POLLPRI         02              /* There is urgent data to read.  */
//# define POLLOUT         04              /* Writing now will not block.  */
//
///* Some aliases.  */
//# define POLLWRNORM      POLLOUT
//# define POLLRDNORM      POLLIN
//# define POLLRDBAND      POLLPRI
//
///* Event types always implicitly polled for.  These bits need not be set in
//   `events', but they will appear in `revents' to indicate the status of
//   the file descriptor.  */
//# define POLLERR         010             /* Error condition.  */
//# define POLLHUP         020             /* Hung up.  */
//# define POLLNVAL        040             /* Invalid polling request.  */
//
///* Canonical number of polling requests to read in at a time in poll.  */
//# define NPOLLFILE       30
//#endif

