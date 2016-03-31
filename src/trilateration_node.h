//
// Created by ptirindelli on 29/01/16.
//

#ifndef ASTERX1_NODE_TRILATERATION_NODE_H
#define ASTERX1_NODE_TRILATERATION_NODE_H

#include <ros/ros.h>

// ROS messages
#include "iri_common_drivers_msgs/SatellitePseudorangeArray.h"
#include "iri_common_drivers_msgs/NavSatFix_ecef.h"
#include "sensor_msgs/NavSatFix.h"

//TODO è meglio installare la libreria e poi fare un include normale?
#include "../include/trilateration/src/trilateration.h"

// To convert ecef coords to lla
#include <Position.hpp>
#include <Triple.hpp>
#include "WGS84Ellipsoid.hpp"

//To write file
#include <iostream>
#include <fstream>

class TrilaterationNode
{
public:
    TrilaterationNode();
    ~TrilaterationNode();
    void pseudorangeCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr &msg);
    void fixEcefCallback(const iri_common_drivers_msgs::NavSatFix_ecef::ConstPtr &msg);
    void raimEcefCallback(const iri_common_drivers_msgs::NavSatFix_ecef::ConstPtr &msg);
    void wolfEcefCallback(const iri_common_drivers_msgs::NavSatFix_ecef::ConstPtr &msg);
    void fixLlaCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    Point<double> ecefToLla(double x, double y, double z);
    Point<double> ecefToLla(const Point<double> &ecef);

    Receiver computePosition(std::vector<SatelliteMeasurement> &measurements);

protected:
    /*
     * Just for debug purpose.
     *
     * It prints the solutions in some files,
     * then they can be visualized through:
     * http://www.gpsvisualizer.com/map_input?form=google
     */
    bool writeOnFile(std::string path, double x, double y, double z);
    bool writeOnFile(std::string path, Point<double> p);


    bool saveOnDisk = true;
    const std::string PATH_REAL_POS = "/home/ptirindelli/Desktop/gps_visualizer/real.txt";
    const std::string PATH_EST_POS = "/home/ptirindelli/Desktop/gps_visualizer/est.txt";
    const std::string PATH_RAIM_POS = "/home/ptirindelli/Desktop/gps_visualizer/raim.txt";
    const std::string PATH_WOLF_POS = "/home/ptirindelli/Desktop/gps_visualizer/wolf.txt";
    const std::string PATH_LLA = "/home/ptirindelli/Desktop/gps_visualizer/lla.txt";

    int counterReal = 0;
    int counterEst = 0;
    int counterRaim = 0;
    int counterWolf = 0;
    int counterLLA = 0;
    int sampling_rate_est, sampling_rate_real;

protected:
    Trilateration tr;
    Point<double> lastFixECEF;
    Point<double> lastRaimECEF;
    Point<double> lastWolfECEF;
    Point<double> lastFixLLA;

    // ROS node handle
    ros::NodeHandle nh;

    // Subscriber (pseudoranges)
    ros::Subscriber pseudorangeSub;
    ros::Subscriber fixEcefSub; // fix ecef subscriber
    ros::Subscriber raimEcefSub; // raim ecef subscriber
    ros::Subscriber wolfEcefSub; // wolf ecef subscriber
    ros::Subscriber llaSub; // fix lla subscriber

    // Publisher
    ros::Publisher estFixPub;

};


#endif //ASTERX1_NODE_TRILATERATION_NODE_H
