
#ifndef RAW_REC_NODE_H
#define RAW_REC_NODE_H

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "iri_asterx1_gps/GPS_time.h"


class RawReceiverNode
{
protected:
    // ROS node handle
    ros::NodeHandle nh;

    // Subscriber (measurements)
    ros::Subscriber obsSub;

public:
    RawReceiverNode();
    ~RawReceiverNode();

    void obsCallback(const iri_asterx1_gps::GPS_time::ConstPtr& msg);//TODO mettici messaggio personalizzato
                        //  iri_asterx1_gps/GPS_nav

protected:
    void process();

};
#endif
