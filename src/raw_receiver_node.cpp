#include "raw_receiver_node.h"


RawReceiverNode::RawReceiverNode() :
        nh(ros::this_node::getName()),
        ephStore(36, gpstk::EngEphemeris())//36 is the number of satellite (32 actual + 4 maybe in the future)
{
    obsSub = nh.subscribe("/iri_asterx1_gps/gps_meas", 1000, &RawReceiverNode::obsCallback, this);
    navSub = nh.subscribe("/iri_asterx1_gps/gps_raw_data", 1000, &RawReceiverNode::navCallback, this);

    observationPub = nh.advertise<asterx1_node::SatPrArray>("/sat_pseudoranges", 5000);

}

RawReceiverNode::~RawReceiverNode()
{

}


void RawReceiverNode::obsCallback(const iri_asterx1_gps::GPS_meas::ConstPtr& msg)
{
    gpstk::GPSWeekSecond time_gps = getTimeGPS(msg->time_stamp.tow, msg->time_stamp.wnc);
    ros::Time time_ros = ros::Time::now();

    std::cout << "OBS callback: " << msg->type1_info.size() << " obs received at " << gpstk::CivilTime(time_gps) << std::endl;

    // The ROS message
    asterx1_node::SatPrArray observation;
    observation.timestamp = time_ros;

    for (int i = 0; i < msg->type1_info.size(); ++i)
    {
        const iri_asterx1_gps::GPS_meas_type1 meas_t1 = msg->type1_info[i];
        double P1 = meas_t1.pseudo_range;

        // non abbiamo le frequenze p2, quindi non possiamo calcolare una correzione atmosferica

        // If the ephemeris is present
        //TODO che if ephemeris isn't too old!!         e.g. obs.receivedTime() < eph.getTimestamp()[credo sta funzione restituisca il periodo di validità]
        if(ephStore[meas_t1.sat_id].PRNID == meas_t1.sat_id)
        {
            gpstk::Xvt sat = ephStore[meas_t1.sat_id].svXvt(time_gps);

            std::cout << "\tPRN " << (short)meas_t1.sat_id
                      << "\t pr = " << P1
                      << "\tecef " << sat.x << std::endl;

            observation.measurements.push_back(createSatMsg((short) meas_t1.sat_id, time_ros, P1, sat));
        }
        else
        {
            std::cout << "\tPRN " << (short)meas_t1.sat_id << " is a BAD ENTRY" << std::endl;
        }

    }

//    std::cout << "\t% of good entries = " << goodEntries << "/" << msg->type1_info.size() << "\n";

    observationPub.publish(observation);

}


void RawReceiverNode::navCallback(const iri_asterx1_gps::GPS_raw_frames::ConstPtr& msg)
{
    std::cout << "### NAV callback: sat " << (short) msg->sat_id << "  \tseq# " << msg->header.seq;

    // Bits of subframes 1, 2 and 3
    std::vector<long> sf1, sf2, sf3;
    for (int i = 0; i < msg->subframe1.size(); ++i)
    {
        sf1.push_back(msg->subframe1[i]);
        sf2.push_back(msg->subframe2[i]);
        sf3.push_back(msg->subframe3[i]);
    }

    gpstk::EngEphemeris ee;
    try
    {
        // Initialize the ephemeris through subframes
        //    long int sf1_bits[10];
        ee.addSubframe(&sf1[0], msg->wn, msg->sat_id, msg->track_id);
        ee.addSubframe(&sf2[0], msg->wn, msg->sat_id, msg->track_id);
        ee.addSubframe(&sf3[0], msg->wn, msg->sat_id, msg->track_id);

        // Save the Ephemeris
        ephStore[ee.PRNID] = ee;
    }
    catch (gpstk::InvalidParameter &e)
    {
        std::cerr << "InvalidParameter!\n" << e.what() << std::endl;
    }
    catch(gpstk::Exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Caught an unexpected exception." << std::endl;
    }


    std::cout << "   \ttime " << gpstk::CivilTime(ee.getTransmitTime()) << std::endl;

    // Print the ephemeris
//    ee.dump();
}

////TODO check again
//TODO fa che basti passare l'oggetto timestamp
gpstk::GPSWeekSecond RawReceiverNode::getTimeGPS(unsigned int tow, unsigned short wnc)
{
    return gpstk::GPSWeekSecond(wnc, (double)tow/1000, gpstk::TimeSystem::GPS);
}

asterx1_node::SatPr RawReceiverNode::createSatMsg(short sat_id, ros::Time &timeROS, double pr, gpstk::Xvt &sat)
{
    asterx1_node::SatPr satPr;

    satPr.sat_id = sat_id;
    satPr.timestamp = timeROS;
    satPr.pseudorange = pr;
    satPr.x = sat.x[0];
    satPr.y = sat.x[1];
    satPr.z = sat.x[2];
    satPr.v_x = sat.v[0];
    satPr.v_y = sat.v[1];
    satPr.v_z = sat.v[2];

    return satPr;
}
