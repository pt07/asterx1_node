//
// Created by ptirindelli on 29/01/16.
//

#ifndef ASTERX1_NODE_TRILATERATION_NODE_H
#define ASTERX1_NODE_TRILATERATION_NODE_H

#include <ros/ros.h>

// ROS messages
#include "asterx1_node/SatPr.h"
#include "asterx1_node/SatPrArray.h"
#include "iri_asterx1_gps/NavSatFix_ecef.h"
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
    void pseudorangeCallback(const asterx1_node::SatPrArray::ConstPtr &msg);
    void fixEcefCallback(const iri_asterx1_gps::NavSatFix_ecef::ConstPtr &msg);
    void raimEcefCallback(const iri_asterx1_gps::NavSatFix_ecef::ConstPtr &msg);

    Point<double> ecefToLla(double x, double y, double z);
    Point<double> ecefToLla(const Point<double> &ecef);

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
    const std::string PATH_REAL_POS = "/home/ptirindelli/bagfiles/output/real.txt";
    const std::string PATH_EST_POS = "/home/ptirindelli/bagfiles/output/est.txt";
    const std::string PATH_RAIM_POS = "/home/ptirindelli/bagfiles/output/raim.txt";

    int counterReal = 0;
    int counterEst = 0;
    int counterRaim = 0;
    const int SAMPLING_RATE = 1;

protected:
    Trilateration tr;
    Point<double> lastFixECEF;
    Point<double> lastRaimECEF;

    // ROS node handle
    ros::NodeHandle nh;

    // Subscriber (pseudoranges)
    ros::Subscriber pseudorangeSub;
    ros::Subscriber fixEcefSub; // fix ecef subscriber
    ros::Subscriber raimEcefSub; // fix ecef subscriber

    // Publisher
    ros::Publisher estFixPub;

};


#endif //ASTERX1_NODE_TRILATERATION_NODE_H
