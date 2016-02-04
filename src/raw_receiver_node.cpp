#include "raw_receiver_node.h"


RawReceiverNode::RawReceiverNode() :
        nh(ros::this_node::getName())
{
    obsSub = nh.subscribe("/iri_asterx1_gps/gps_meas", 1000, &RawReceiverNode::obsCallback, this);
    navSub = nh.subscribe("/iri_asterx1_gps/gps_nav", 1000, &RawReceiverNode::navCallback, this);

    observationPub = nh.advertise<asterx1_node::SatPrArray>("/sat_pseudoranges", 5000);

    //GPStk stuff

}

RawReceiverNode::~RawReceiverNode()
{

}


asterx1_node::SatPr RawReceiverNode::getSatMsg(gpstk::SatID &prn, ros::Time &time, double pr, double x, double y, double z, double vx, double vy, double vz)
{
    asterx1_node::SatPr satPr;

    satPr.sat_id = prn.id;
    satPr.timestamp = time;
    satPr.pseudorange = pr;
    satPr.x = x;
    satPr.y = y;
    satPr.z = z;
    satPr.v_x = vx;
    satPr.v_y = vy;
    satPr.v_z = vz;

    return satPr;

}



void RawReceiverNode::obsCallback(const iri_asterx1_gps::GPS_meas::ConstPtr& msg)
{
    std::cout << "OBS callback: time " << getTimeGPS(msg->time_stamp.tow, msg->time_stamp.wnc)
              << "\t" << msg->type1_info.size() << " obs received" << std::endl;

    currentTime = ros::Time::now();


    for (int i = 0; i < msg->type1_info.size(); ++i)
    {
        const iri_asterx1_gps::GPS_meas_type1 meas_t1 = msg->type1_info[i];
        double P1 = meas_t1.pseudo_range;

        // non abbiamo le frequenze p2, quindi non possiamo calcolare una correzione atmosferica

        //TODO fai qualcosa col pseudorange
    }

    //TODO do the math to calculate sat position

//    int goodEntries = 0;
//    for (int i = 0; i < prnVec.size(); ++i)
//    {
//        if(prnVec[i].id>0)
//            ++goodEntries;
//      else std::cout << "Sat #" << prnVec[i].id << " is a bad entry\n";
//    }
//    std::cout << "\t% of good entries = " << goodEntries << "/" << prnVec.size() << "\n";

    // TODO Compose the message
//    asterx1_node::SatPrArray observation;
//
//    observation.timestamp = currentTime;
//
//    for (size_t i = 0; i < prnVec.size(); ++i)
//    {
//        if(prnVec[i].id > 0)
//        {
//            std::cout << "\tPRN " << prnVec[i].id
//            << "\tnew pr = " << calcPos[i][3]
//            << "\tecef (" << calcPos[i][0] << ", " << calcPos[i][1] << ", " << calcPos[i][2] << ") "
//            << std::endl;
//
//            gpstk::Triple vel = bcestore.getXvt(prnVec[i], getTimeGPS(msg->time_stamp.tow, msg->time_stamp.wnc)).getVel();
//
//            observation.measurements.push_back(getSatMsg(prnVec[i], currentTime, rangeVec[i]/*TODO questo è il pseudorange corretto da raim solver calcPos[i][3]*/, calcPos[i][0], calcPos[i][1], calcPos[i][2], vel[0], vel[1], vel[2]));
//        }
//    }
//
//
//    observationPub.publish(observation);

}


void RawReceiverNode::navCallback(const iri_asterx1_gps::GPS_nav::ConstPtr& msg) {
    std::cout << "### NAV callback: sat " << (short) msg->sat_id
    << ", time " << getTimeGPS(msg->time_stamp.tow, msg->time_stamp.wnc)
    << std::endl;

    /*
     * TODO
     * new msg
     * read 3 subframe
     * create EngEphemeris
     * store it
     * in obs don't do RAIM.prepare but sat_i.svXvt(obs time)
     */


}
//    gpstk::Rinex3NavData eph;
//    eph.time = getTimeGPS(msg->time_stamp.tow, msg->time_stamp.wnc);
//    eph.satSys = "G";
//    eph.PRNID = (short)msg->sat_id;
//    eph.sat = gpstk::RinexSatID((short)msg->sat_id, gpstk::SatID::systemGPS);
//
//    try
//    {
//        if( ! bcestore.isPresent(eph.sat))
//        {
//            std::cout << "###BCESTORE: sat" << eph.sat.id << "'s eph NOT present --> ADDING it\n";
//            // Add the ephemeris just created to the ephemerides store
//            bcestore.addEphemeris(eph);
//
//            // just to remember the last ephemeris added for each satellite
//            iodcs[eph.sat.id] = (unsigned short)eph.IODC;
//        }
//        else //if is already present
//        {
//            if (iodcs[eph.sat.id] != msg->iodc)
//            {
//                std::cout << "###BCESTORE: sat" << eph.sat.id << "'s eph is old --> UPDATING it\n";
//                bcestore.addEphemeris(eph);
//                iodcs[eph.sat.id] = (unsigned short)eph.IODC;
//            }
//            else
//            {
////                std::cout << "###BCESTORE: sat" << eph.sat.id << "'s eph ALREADY PRESENT\n";
//            }
//        }
//    }
//    catch(gpstk::Exception &e)
//    {
//        std::cerr << e.what() << std::endl;
//    }
//    catch (...)
//    {
//        std::cerr << "Caught an unexpected exception." << std::endl;
//    }

//TODO check again
gpstk::CivilTime RawReceiverNode::getTimeGPS(unsigned int tow, unsigned short wnc)
{
    return gpstk::CivilTime(gpstk::GPSWeekSecond(wnc, (double)tow/1000, gpstk::TimeSystem::GPS));
}

