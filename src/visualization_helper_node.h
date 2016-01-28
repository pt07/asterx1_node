//
// Created by ptirindelli on 28/01/16.
//

#ifndef ASTERX1_NODE_VISUALIZATION_HELPER_NODE_H
#define ASTERX1_NODE_VISUALIZATION_HELPER_NODE_H

#include <ros/ros.h>

#include "asterx1_node/SatPr.h"
#include <visualization_msgs/Marker.h>


class VisualizationHelperNode
{
public:
    VisualizationHelperNode();
    virtual ~VisualizationHelperNode();

    void pseudorangeCallback(const asterx1_node::SatPr::ConstPtr &msg);

protected:
    // ROS node handle
    ros::NodeHandle nh;

    // Subscriber (pseudoranges)
    ros::Subscriber pseudorangeSub;

    // Publisher (markers)
    ros::Publisher markerPub;

    void publishSat(const asterx1_node::SatPr::ConstPtr &msg);
};


#endif //ASTERX1_NODE_VISUALIZATION_HELPER_NODE_H
