//
// Created by ptirindelli on 28/01/16.
//

#ifndef ASTERX1_NODE_VIZ_HELPER_NODE_H
#define ASTERX1_NODE_VIZ_HELPER_NODE_H

#include <ros/ros.h>

// Messages
#include "iri_common_drivers_msgs/SatellitePseudorangeArray.h"
#include "iri_common_drivers_msgs/NavSatFix_ecef.h"

#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

//to manage quaternions
#include <Eigen/Geometry>

class VizHelperNode
{
public:
    VizHelperNode();
    ~VizHelperNode();

    void pseudorangeCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr &msg);
    void realFixCallback(const iri_common_drivers_msgs::NavSatFix_ecef::ConstPtr &msg);
    void estFixCallback(const iri_common_drivers_msgs::NavSatFix_ecef::ConstPtr &msg);


    Eigen::Quaterniond rotateSatelliteFrame(const iri_common_drivers_msgs::SatellitePseudorange &sat, ros::Time time_ros);

    tf::TransformBroadcaster tf_b;

protected:
    void publishSat(const iri_common_drivers_msgs::SatellitePseudorange &sat, ros::Time time_ros);
    void publishSatVelocity(const iri_common_drivers_msgs::SatellitePseudorange &sat, ros::Time time_ros);
    void publishOdometry(const iri_common_drivers_msgs::SatellitePseudorange &sat, const Eigen::Quaterniond &rotation, ros::Time time_ros);
    void publishSatSphere(const iri_common_drivers_msgs::SatellitePseudorange &sat, ros::Time time_ros);
    void publishRealFix(double x, double y, double z);
    void publishEstFix(double x, double y, double z);

    void publishEarth();

    std::string getSatelliteFrame(int index);

public:
    const std::string WORLD_FRAME = "world";
    const std::string FIX_SENSOR_FRAME = "fix";

    const double EARTH_RADIUS = 6371000; // meters
    const double METERS = 1;
    const double KILOMETERS = METERS/1000;

protected:
    double scale; // provv: per gestire le dimensioni di stampa.

    // ROS node handle
    ros::NodeHandle nh;

    // Subscriber (pseudoranges)
    ros::Subscriber pseudorangeSub;
    ros::Subscriber realFixSub; // subscriber of fix position in ecef, computed by sensor
    ros::Subscriber realFixSub2; // subscriber of fix position in ecef, computed by gpstk
    ros::Subscriber estFixSub; // subscriber of fix position in ecef, computed by trilateration node
    int sensor_fix_received, trilat_fix_received;

    // Publisher (markers)
    ros::Publisher markerPub;

    ros::Publisher odomAllPub;
    std::vector<ros::Publisher> odomPub;

    tf::TransformBroadcaster transBroadcaster;


//    const ros::Duration LIFETIME_SHORT = ros::Duration(20.5);
    const ros::Duration LIFETIME_SHORT = ros::Duration(0);
    const double SMALL_MARKER_SIZE = 0.0008;

    bool trajectory_mode;

    double map_p_x,map_p_y,map_p_z;
    void broadcastTFmap();

    int sampling_rate_real, sampling_rate_est;
};


#endif //ASTERX1_NODE_VIZ_HELPER_NODE_H
