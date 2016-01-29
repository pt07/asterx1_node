//
// Created by ptirindelli on 29/01/16.
//

#include "trilateration_node.h"

TrilaterationNode::TrilaterationNode():
        nh(ros::this_node::getName())
{
    pseudorangeSub = nh.subscribe("/sat_pseudoranges", 1000, &TrilaterationNode::pseudorangeCallback, this);

}

TrilaterationNode::~TrilaterationNode() { }

void TrilaterationNode::pseudorangeCallback(const asterx1_node::SatPrArray::ConstPtr &msg)
{

}
