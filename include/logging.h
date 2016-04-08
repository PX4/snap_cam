#ifndef SNAP_LOGGING_H_
#define SNAP_LOGGING_H_

#ifdef CATKIN_BUILD

#include <ros/console.h>
#define INFO ROS_INFO
#define WARN ROS_WARN
#define ERROR ROS_ERROR

#else

#define INFO(...)   printf("[ INFO] "); printf(__VA_ARGS__); printf(" [%s:%d]\n", __FILE__, __LINE__);
#define WARN(...)   printf("[ WARN] "); printf(__VA_ARGS__); printf(" [%s:%d]\n", __FILE__, __LINE__);
#define ERROR(...)  printf("[ERROR] "); printf(__VA_ARGS__); printf(" [%s:%d]\n", __FILE__, __LINE__);

#endif // CATKIN_BUILD

#endif
