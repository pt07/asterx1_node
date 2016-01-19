#include "raw_receiver_node.h"


RawReceiverNode::RawReceiverNode() :
        nh(ros::this_node::getName())
{
    obsSub = nh.subscribe("/iri_asterx1_gps/gps_meas", 1000, &RawReceiverNode::obsCallback, this);

}

RawReceiverNode::~RawReceiverNode()
{

}

void RawReceiverNode::obsCallback(const iri_asterx1_gps::GPS_meas::ConstPtr& msg)
{
    std::cout << "New observations at tow: " << msg->time_stamp.tow << std::endl;

    for (int i = 0; i < msg->type1_info.size(); ++i)
    {
        const iri_asterx1_gps::GPS_meas_type1 meas_t1 = msg->type1_info[i];

        std::cout << "\t- sat_id: " << (short)meas_t1.sat_id// << std::endl
                  << std::setprecision(12)
                  << "\tpseudo_range: " << (long double)meas_t1.pseudo_range << std::endl;

    }

    //TODO do stuff
    process();
}


void RawReceiverNode::process()
{

}