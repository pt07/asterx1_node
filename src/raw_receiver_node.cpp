#include "raw_receiver_node.h"


RawReceiverNode::RawReceiverNode() :
        nh(ros::this_node::getName())
{
    obsSub = nh.subscribe("/iri_asterx1_gps/gps_meas", 1000, &RawReceiverNode::obsCallback, this);
    navSub = nh.subscribe("/iri_asterx1_gps/gps_nav", 1000, &RawReceiverNode::navCallback, this);

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

    //TODO processObservations();
}


void RawReceiverNode::navCallback(const iri_asterx1_gps::GPS_nav::ConstPtr& msg)
{
    std::cout << "### NAV data for sat " << (short)msg->sat_id << " at tow: " << msg->time_stamp.tow << std::endl;

    //TODO processNav();
}

