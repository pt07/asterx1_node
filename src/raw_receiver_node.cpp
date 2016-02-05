#include "raw_receiver_node.h"


RawReceiverNode::RawReceiverNode() :
        nh(ros::this_node::getName())
{
    obsSub = nh.subscribe("/iri_asterx1_gps/gps_meas", 1000, &RawReceiverNode::obsCallbackRAIM, this);
    navSub = nh.subscribe("/iri_asterx1_gps/gps_raw_data", 1000, &RawReceiverNode::navCallback, this);

    observationPub = nh.advertise<asterx1_node::SatPrArray>("/sat_pseudoranges", 5000);
    raimFixPub = nh.advertise<iri_asterx1_gps::NavSatFix_ecef>("/raim_fix", 5000);


    //GPStk stuff
    tropModelPtr=&noTropModel;//if there is not a tropospheric model
    // I'm not using meteorological file, so it will be always like this
    // TODO vedi se puoi usare un trop model presente nella classe .h
    bcestore.SearchNear();// Setting the criteria for looking up ephemeris

}

RawReceiverNode::~RawReceiverNode()
{

}



void RawReceiverNode::obsCallbackRAIM(const iri_asterx1_gps::GPS_meas::ConstPtr& msg)
{
    gpstk::GPSWeekSecond time_gps = getTimeGPS(msg->time_stamp);
    ros::Time time_ros = ros::Time::now();

    std::cout << "OBS callback: " << msg->type1_info.size() << " obs received at " << gpstk::CivilTime(time_gps) << std::endl;

    std::vector<gpstk::SatID> prnVec;
    std::vector<double> rangeVec;

    for (int i = 0; i < msg->type1_info.size(); ++i)
    {
        const iri_asterx1_gps::GPS_meas_type1 meas_t1 = msg->type1_info[i];
        double P1 = meas_t1.pseudo_range;

        // non abbiamo le frequenze p2, quindi non possiamo calcolare una correzione atmosferica

        prnVec.push_back(gpstk::SatID((short)meas_t1.sat_id, gpstk::SatID::systemGPS));
        rangeVec.push_back(P1);
    }

    raimSolver.RMSLimit = 3e6;

    /************************************************************************
     *              DO the math to calculate the sat position               *
     *                     and correct the pseudoranges                     *
     ************************************************************************/
    gpstk::Matrix<double> calcPos;

    int ret = raimSolver.PrepareAutonomousSolution(getTimeGPS(msg->time_stamp),
                                               prnVec,
                                               rangeVec,
                                               bcestore,
                                               calcPos); //satellite positions at transmit time
    //Return values:  0 ok
    //               -4 ephemeris not found for all the satellites

    if(ret == -4)
    {
        std::cout << "\t% of good entries = 0/" << prnVec.size() << "\n";
        return;
    }

    int goodEntries = 0;
    for (int i = 0; i < prnVec.size(); ++i)
    {
        if(prnVec[i].id>0)
            ++goodEntries;
//      else std::cout << "Sat #" << prnVec[i].id << " is a bad entry\n";
    }
    std::cout << "\t% of good entries = " << goodEntries << "/" << prnVec.size() << "\n";

    // Compose the message
    asterx1_node::SatPrArray observation;

    observation.timestamp = time_ros;

    for (size_t i = 0; i < prnVec.size(); ++i)
    {
        if(prnVec[i].id > 0)
        {
            std::cout << "\tPRN " << prnVec[i].id
                    << "\tnew pr = " << calcPos[i][3] // pseudorange corrected by raimsolver
                    << "\tecef (" << calcPos[i][0] << ", " << calcPos[i][1] << ", " << calcPos[i][2] << ") "
                    << std::endl;


            gpstk::Triple vel = bcestore.getXvt(prnVec[i], getTimeGPS(msg->time_stamp)).getVel();

            // Use pr corrected by RAIM
            observation.measurements.push_back(createSatMsg(prnVec[i].id, time_ros, calcPos[i][3], calcPos[i][0], calcPos[i][1], calcPos[i][2], vel[0], vel[1], vel[2]));
//            // Use pr as received
//            observation.measurements.push_back(createSatMsg(prnVec[i].id, time_ros, rangeVec[i], calcPos[i][0], calcPos[i][1], calcPos[i][2], vel[0], vel[1], vel[2]));
        }
    }


    observationPub.publish(observation);




    /*****************************************************
     * calcola la posizione attuale tramite RAIM solve.. *
     *                  ¡¡¡ funziona !!!                 *
     *****************************************************/

    try {

        raimSolver.RAIMCompute(
                getTimeGPS(msg->time_stamp),
                prnVec,
                rangeVec,
                bcestore,
                tropModelPtr);

        if (raimSolver.isValid())
        {
            // Vector "Solution" holds the coordinates, expressed in meters
            // in an Earth Centered, Earth Fixed (ECEF) reference frame.
            //std::cout << std::setprecision(12) << raimSolver.Solution[0] << " " << raimSolver.Solution[1] << " " << raimSolver.Solution[2];

            gpstk::Triple sol_xyz, sol_llr;
            sol_xyz[0] = raimSolver.Solution[0];
            sol_xyz[1] = raimSolver.Solution[1];
            sol_xyz[2] = raimSolver.Solution[2];
            gpstk::WGS84Ellipsoid WGS84;
            double AEarth = WGS84.a();
            double eccSquared = WGS84.eccSquared();

            gpstk::Position::convertCartesianToGeodetic(sol_xyz, sol_llr, AEarth, eccSquared);

            //std::cout << " --> LLR: " << sol_llr[0] << " " << sol_llr[1] << " " << sol_llr[2] << std::endl;


            // Publishing the raim fix
            iri_asterx1_gps::NavSatFix_ecef raimFixMsg;
            //TODO fill up header etc
            raimFixMsg.x = raimSolver.Solution[0];
            raimFixMsg.y = raimSolver.Solution[1];
            raimFixMsg.z = raimSolver.Solution[2];

            raimFixPub.publish(raimFixMsg);
        }
        else
        {
            std::cout << "raimSolver NOT Valid" << std::endl;
        }
    }
    catch (gpstk::Exception& e)
    {
        std::cerr << e.what() << std::endl;
    }




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

    try
    {
        // Create ephemeris
        gpstk::EngEphemeris ee;
        // Initialize the ephemeris through subframes
        ee.addSubframe(&sf1[0], msg->wn, msg->sat_id, msg->track_id);
        ee.addSubframe(&sf2[0], msg->wn, msg->sat_id, msg->track_id);
        ee.addSubframe(&sf3[0], msg->wn, msg->sat_id, msg->track_id);

        // Save the Ephemeris
        bcestore.addEphemeris(gpstk::RinexNavData(ee));

        std::cout << "   \ttime " << gpstk::CivilTime(ee.getTransmitTime()) << std::endl;

        // Print the ephemeris
//        ee.dump();
    }
    catch (gpstk::InvalidParameter &e)
    {
        std::cerr << "InvalidParameter!\n" << e.what() << std::endl;
    }
    catch (gpstk::Exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Caught an unexpected exception." << std::endl;
    }
}

gpstk::GPSWeekSecond RawReceiverNode::getTimeGPS(const iri_asterx1_gps::GPS_time timestamp)
{
    return gpstk::GPSWeekSecond(timestamp.wnc, (double)timestamp.tow/1000, gpstk::TimeSystem::GPS);
}

asterx1_node::SatPr RawReceiverNode::createSatMsg(unsigned short sat_id, ros::Time &time, double pr, double x, double y, double z, double vx, double vy, double vz)
{
    asterx1_node::SatPr satPr;

    satPr.sat_id = sat_id;
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
