//
// Created by ptirindelli on 29/01/16.
//

#ifndef ASTERX1_NODE_TRILATERATION_NODE_H
#define ASTERX1_NODE_TRILATERATION_NODE_H

#include <ros/ros.h>

#include "asterx1_node/SatPr.h"
#include "asterx1_node/SatPrArray.h"

//TODO è meglio installare la libreria e poi fare un include normale?
#include "../include/trilateration/src/trilateration.h"

class TrilaterationNode
{
public:
    TrilaterationNode();
    ~TrilaterationNode();
    void pseudorangeCallback(const asterx1_node::SatPrArray::ConstPtr &msg);


protected:
    Trilateration tr;

    // ROS node handle
    ros::NodeHandle nh;

    // Subscriber (pseudoranges)
    ros::Subscriber pseudorangeSub;

};


#endif //ASTERX1_NODE_TRILATERATION_NODE_H
