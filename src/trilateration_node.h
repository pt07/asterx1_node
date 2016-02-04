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


class TrilaterationNode
{
public:
    TrilaterationNode();
    ~TrilaterationNode();
    void pseudorangeCallback(const asterx1_node::SatPrArray::ConstPtr &msg);
    void fixEcefCallback(const iri_asterx1_gps::NavSatFix_ecef::ConstPtr &msg);

    Point<double> ecefToLla(const Point<double> &ecef);

protected:
    Trilateration tr;
    Point<double> lastFixECEF;

    // ROS node handle
    ros::NodeHandle nh;

    // Subscriber (pseudoranges)
    ros::Subscriber pseudorangeSub;
    ros::Subscriber fixEcefSub; // fix ecef subscriber

    // Publisher
    ros::Publisher estFixPub;

};


#endif //ASTERX1_NODE_TRILATERATION_NODE_H
