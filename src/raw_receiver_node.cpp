#include "raw_receiver_node.h"


RawReceiverNode::RawReceiverNode() :
        nh(ros::this_node::getName())
{
    //TODO arriva a questo:
    // obsSub = nh.subscribe("/iri_asterx1_gps/gps_nav", 1000, &RawReceiverNode::obsCallback, this);
    obsSub = nh.subscribe("/chatter", 1000, &RawReceiverNode::obsCallback, this);

}

RawReceiverNode::~RawReceiverNode()
{

}

void RawReceiverNode::obsCallback(const iri_asterx1_gps::GPS_time::ConstPtr& msg)
{
    std::cout << "New observations received! tow: " << msg->tow << std::endl;


    //TODO do stuff
    process();
}


void RawReceiverNode::process()
{

}