//
// Created by ptirindelli on 28/01/16.
//

#include "visualization_helper_node.h"

VisualizationHelperNode::VisualizationHelperNode() :
        nh(ros::this_node::getName())
{
    pseudorangeSub = nh.subscribe("/sat_pseudoranges", 1000), VisualizationHelperNode::pseudorangeCallback, this);

    markerPub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);
}

virtual VisualizationHelperNode::~VisualizationHelperNode()
{

}

void VisualizationHelperNode::pseudorangeCallback(const asterx1_node::SatPr::ConstPtr &msg)
{
    //publishSat(msg);
}


