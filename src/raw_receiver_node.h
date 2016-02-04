#ifndef RAW_REC_NODE_H
#define RAW_REC_NODE_H

/***************************
 *      GPStk includes     *
 ***************************/
#include <EngEphemeris.hpp>

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>

// ROS messages
#include "iri_asterx1_gps/GPS_meas.h"
#include "iri_asterx1_gps/GPS_raw_frames.h"
#include "iri_asterx1_gps/NavSatFix_ecef.h"
#include "sensor_msgs/NavSatFix.h"

#include "asterx1_node/SatPr.h"
#include "asterx1_node/SatPrArray.h"

/**************************
 *      STD includes      *
 **************************/
#include <vector>


//TODO check if cmakelist deny make if some libraries are not present

class RawReceiverNode
{
public:
    RawReceiverNode();
    ~RawReceiverNode();

    asterx1_node::SatPr createSatMsg(short sat_id, ros::Time &timeROS, double pr, gpstk::Xvt &sat);

    void obsCallback(const iri_asterx1_gps::GPS_meas::ConstPtr& msg);
    void navCallback(const iri_asterx1_gps::GPS_raw_frames::ConstPtr& msg);

protected:
    gpstk::GPSWeekSecond getTimeGPS(unsigned int tow, unsigned short wnc);


protected:
    // ROS node handle
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber obsSub; // obs (measurements) subscriber
    ros::Subscriber navSub; // nav subscriber
    ros::Subscriber fixLlaSub; // fix long lat alt subscriber
    ros::Subscriber fixEcefSub; // fix ecef subscriber

    // Publishers
    ros::Publisher observationPub;


    // GPStk stuff
    std::vector<gpstk::EngEphemeris> ephStore;

};
#endif
