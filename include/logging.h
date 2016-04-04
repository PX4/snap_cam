#ifndef LOGGING_H_
#define LOGGING_H_

#ifdef CATKIN_BUILD

#include <ros/console.h>
#define LOG_INFO ROS_INFO
#define LOG_WARN ROS_WARN
#define LOG_ERROR ROS_ERROR

#else

#define LOG_INFO puts
#define LOG_WARN puts
#define LOG_ERROR puts

#endif // CATKIN_BUILD

#endif
