
#ifndef RAW_REC_NODE_H
#define RAW_REC_NODE_H

#include <vector>

/***************************
 *      GPStk includes     *
 ***************************/
// Class defining GPS system constants
#include "GNSSconstants.hpp"
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
//tentativo con rinex nav data per le eph. se non funziona, togli sto include
#include "Rinex3NavData.hpp"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
// ROS messages
#include "iri_asterx1_gps/GPS_meas.h"
#include "iri_asterx1_gps/GPS_nav.h"
#include "iri_asterx1_gps/NavSatFix_ecef.h"
#include "sensor_msgs/NavSatFix.h"

//TODO check if cmakelist deny make if some libraries are not present

class RawReceiverNode
{
protected:
    // ROS node handle
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber obsSub; // obs (measurements) subscriber
    ros::Subscriber navSub; // nav subscriber
    ros::Subscriber fixLlaSub; // fix long lat alt subscriber
    ros::Subscriber fixEcefSub; // fix long lat alt subscriber

    // GPStk stuff
    gpstk::GPSEphemerisStore bcestore; //object for storing ephemerides
    gpstk::PRSolution2 raimSolver; //object for handling RAIM
    gpstk::ZeroTropModel noTropModel;// Object for void-type tropospheric model (in case no meteorological RINEX is available)
    gpstk::ZeroTropModel *tropModelPtr;// Pointer to one of the two available tropospheric models. It points to the void model by default

    //forse sta gamma non serve, perchè non ho la frequenza l2 e di conseguenza non posso calcolare la ionocorr
    const double gamma = (gpstk::L1_FREQ_GPS/gpstk::L2_FREQ_GPS)*(gpstk::L1_FREQ_GPS/gpstk::L2_FREQ_GPS);
    int indexP1; //TODO vedi a cosa serve
    const int indexP2 = -1; //TODO vedi a cosa serve

    std::vector<gpstk::SatID> prnVec;
    std::vector<double> rangeVec;

    //Robe di debug
    int numRAIMNotValid;
    bool printFixEcef = false;
    bool printFixLla = false;


    /*
     * 2 modalità:
     *      calcSatPosition = true -->  calcola la posizione dei satelliti
     *      calcSatPosition = false --> calcola un gps fix
     */
    bool calcSatPosition = true;

public:
    RawReceiverNode();
    ~RawReceiverNode();

    void obsCallback(const iri_asterx1_gps::GPS_meas::ConstPtr& msg);
    void navCallback(const iri_asterx1_gps::GPS_nav::ConstPtr& msg);
    void navCallback2(const iri_asterx1_gps::GPS_nav::ConstPtr& msg);
    void fixLlaCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void fixEcefCallback(const iri_asterx1_gps::NavSatFix_ecef::ConstPtr &msg);

protected:
    gpstk::CivilTime getTime(unsigned int tow, unsigned short wnc);

    void calculateFix(const iri_asterx1_gps::GPS_meas::ConstPtr& msg);
    void calculateSatPosition(const iri_asterx1_gps::GPS_meas::ConstPtr& msg);

};
#endif
