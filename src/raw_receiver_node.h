
#ifndef RAW_REC_NODE_H
#define RAW_REC_NODE_H

#include <ros/ros.h>

//#include "std_msgs/String.h" //
#include "iri_asterx1_gps/GPS_meas.h"


class RawReceiverNode
{
protected:
    // ROS node handle
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber obsSub; // observations (measurements) subscriber

public:
    RawReceiverNode();
    ~RawReceiverNode();

    void obsCallback(const iri_asterx1_gps::GPS_meas::ConstPtr& msg);

protected:
    void process();

};
#endif
