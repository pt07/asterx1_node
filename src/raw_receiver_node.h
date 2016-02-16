#ifndef RAW_REC_NODE_H
#define RAW_REC_NODE_H

/***************************
 *      GPStk includes     *
 ***************************/
#include <EngEphemeris.hpp>
// Class for storing "broadcast-type" ephemerides
#include "GPSEphemerisStore.hpp"
// Class for handling RAIM
#include "PRSolution2.hpp"
// Class for handling tropospheric models
#include "TropModel.hpp"
#include "Rinex3NavData.hpp"
//#include "GPSEphemeris.hpp"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>

// ROS messages
#include "iri_common_drivers_msgs/GPS_meas.h"
#include "iri_common_drivers_msgs/GPS_raw_frames.h"
#include "iri_common_drivers_msgs/NavSatFix_ecef.h"
//#include "sensor_msgs/NavSatFix.h"

#include "iri_common_drivers_msgs/SatellitePseudorangeArray.h"

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

    iri_common_drivers_msgs::SatellitePseudorange createSatMsg(unsigned short sat_id, double pr, double x, double y, double z, double vx, double vy, double vz);

    void obsCallback(const iri_common_drivers_msgs::GPS_meas::ConstPtr &msg);
    void navCallback(const iri_common_drivers_msgs::GPS_raw_frames::ConstPtr& msg);

protected:
    gpstk::GPSWeekSecond getTimeGPS(const iri_common_drivers_msgs::GPS_time timestamp);


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
    ros::Publisher raimFixPub;

    // GPStk stuff
    gpstk::GPSEphemerisStore bcestore; //object for storing ephemerides
    gpstk::PRSolution2 raimSolver; //object for handling RAIM
    gpstk::ZeroTropModel noTropModel;// Object for void-type tropospheric model (in case no meteorological RINEX is available)
    gpstk::ZeroTropModel *tropModelPtr;// Pointer to one of the two available tropospheric models. It points to the void model by default


};
#endif
