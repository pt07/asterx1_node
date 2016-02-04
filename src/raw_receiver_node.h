#ifndef RAW_REC_NODE_H
#define RAW_REC_NODE_H

/***************************
 *      GPStk includes     *
 ***************************/
// Class defining GPS system constants
//#include "GNSSconstants.hpp"
// To convert coords from ecef to lla
#include <Position.hpp>
#include <Triple.hpp>
#include "WGS84Ellipsoid.hpp"
// To create satellites with ID and satellite system (GPS in our case)
#include <SatID.hpp>
//TODO tentativo con le eng ephemeris
#include <EngEphemeris.hpp>

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>

// ROS messages
#include "iri_asterx1_gps/GPS_meas.h"
#include "iri_asterx1_gps/GPS_nav.h"
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

    asterx1_node::SatPr getSatMsg(gpstk::SatID &prn, ros::Time &time, double pr, double x, double y, double z, double vx, double vy, double vz);

    void obsCallback(const iri_asterx1_gps::GPS_meas::ConstPtr& msg);
    void navCallback(const iri_asterx1_gps::GPS_nav::ConstPtr& msg);

protected:
    gpstk::CivilTime getTimeGPS(unsigned int tow, unsigned short wnc);

//    void calculateFixGPStk(const iri_asterx1_gps::GPS_meas::ConstPtr &msg);

protected:
    // ROS node handle
    ros::NodeHandle nh;
    ros::Time currentTime;

    // Subscribers
    ros::Subscriber obsSub; // obs (measurements) subscriber
    ros::Subscriber navSub; // nav subscriber
    ros::Subscriber fixLlaSub; // fix long lat alt subscriber
    ros::Subscriber fixEcefSub; // fix ecef subscriber

    // Publishers
    ros::Publisher observationPub;


    // GPStk stuff


};
#endif
