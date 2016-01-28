//
// Created by ptirindelli on 28/01/16.
//

#ifndef ASTERX1_NODE_VISUALIZATION_HELPER_NODE_H
#define ASTERX1_NODE_VISUALIZATION_HELPER_NODE_H

#include <ros/ros.h>

#include "asterx1_node/SatPr.h"
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class VisualizationHelperNode
{
public:
    VisualizationHelperNode();
    ~VisualizationHelperNode();

    void pseudorangeCallback(const asterx1_node::SatPr::ConstPtr &msg);

protected:
    void publishSat(const asterx1_node::SatPr::ConstPtr &msg);
    void publishEarth();


public:
    const std::string WORLD_FRAME = "world";

    const double EARTH_RADIUS = 6371000; // meters
    const double METERS = 1;
    const double KILOMETERS = METERS/1000;

protected:
    double scale; // provv: per gestire le dimensioni di stampa.

    // ROS node handle
    ros::NodeHandle nh;

    // Subscriber (pseudoranges)
    ros::Subscriber pseudorangeSub;

    // Publisher (markers)
    ros::Publisher markerPub;

    ros::Publisher odomAllPub;
    std::vector<ros::Publisher> odomPub;

    tf::TransformBroadcaster transBroadcaster;
};


#endif //ASTERX1_NODE_VISUALIZATION_HELPER_NODE_H
