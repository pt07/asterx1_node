
#ifndef RAW_REC_NODE_H
#define RAW_REC_NODE_H


/***************************
 *      GPStk includes     *
 ***************************/
// Class defining GPS system constants
#include "GNSSconstants.hpp"

// Class for storing "broadcast-type" ephemerides
#include "GPSEphemerisStore.hpp"



/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>

// ROS messages
#include "iri_asterx1_gps/GPS_meas.h"
#include "iri_asterx1_gps/GPS_nav.h"


// TODO ¡¡¡ fix cmakelist, in order to require gpstk installed !!!
// TODO ¡¡¡ fix cmakelist, in order to require gpstk installed !!!
// TODO ¡¡¡ fix cmakelist, in order to require gpstk installed !!!
// TODO ¡¡¡ fix cmakelist, in order to require gpstk installed !!!

class RawReceiverNode
{
protected:
    // ROS node handle
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber obsSub; // obs (measurements) subscriber
    ros::Subscriber navSub; // nav subscriber


    gpstk::GPSEphemerisStore bcestore;

public:
    RawReceiverNode();
    ~RawReceiverNode();

    void obsCallback(const iri_asterx1_gps::GPS_meas::ConstPtr& msg);
    void navCallback(const iri_asterx1_gps::GPS_nav::ConstPtr& msg);

protected:
//    void processObservations();

};
#endif
