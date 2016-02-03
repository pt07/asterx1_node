
#ifndef RAW_REC_NODE_H
#define RAW_REC_NODE_H

#include <vector>

/***************************
 *      GPStk includes     *
 ***************************/
// Class defining GPS system constants
//#include "GNSSconstants.hpp"
// Class for storing "broadcast-type" ephemerides
#include "GPSEphemerisStore.hpp"
// Class for handling RAIM
#include "PRSolution2.hpp"
// Class for handling tropospheric models
#include "TropModel.hpp"
// To convert coords from ecef to lla
#include <Position.hpp>
#include <Triple.hpp>
#include "WGS84Ellipsoid.hpp"
// To create satellites with ID and satellite system (GPS in our case)
#include <SatID.hpp>
//TODO tentativo con rinex nav data per le eph. se non funziona, togli sto include
#include "Rinex3NavData.hpp"
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



//TODO check if cmakelist deny make if some libraries are not present

class RawReceiverNode
{
public:
    RawReceiverNode();
    ~RawReceiverNode();

    asterx1_node::SatPr getSatMsg(gpstk::SatID &prn, ros::Time &time, double pr, double x, double y, double z, double vx, double vy, double vz);

    void obsCallback(const iri_asterx1_gps::GPS_meas::ConstPtr& msg);
    void navCallback(const iri_asterx1_gps::GPS_nav::ConstPtr& msg);
    void fixLlaCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void fixEcefCallback(const iri_asterx1_gps::NavSatFix_ecef::ConstPtr &msg);

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
    gpstk::GPSEphemerisStore bcestore; //object for storing ephemerides
    gpstk::PRSolution2 raimSolver; //object for handling RAIM
    gpstk::ZeroTropModel noTropModel;// Object for void-type tropospheric model (in case no meteorological RINEX is available)
    gpstk::ZeroTropModel *tropModelPtr;// Pointer to one of the two available tropospheric models. It points to the void model by default

    //forse sta gamma non serve, perchè non ho la frequenza l2 e di conseguenza non posso calcolare la ionocorr
    //const double gamma = (gpstk::L1_FREQ_GPS/gpstk::L2_FREQ_GPS)*(gpstk::L1_FREQ_GPS/gpstk::L2_FREQ_GPS);

    //to add only new ephemeris
    unsigned short iodcs[32];

    // Debug stuff
    bool printFixEcef = false;
    bool printFixLla = false;
};
#endif
