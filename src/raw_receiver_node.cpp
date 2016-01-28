#include "raw_receiver_node.h"


RawReceiverNode::RawReceiverNode() :
        nh(ros::this_node::getName())
{
    obsSub = nh.subscribe("/iri_asterx1_gps/gps_meas", 1000, &RawReceiverNode::obsCallback, this);
    navSub = nh.subscribe("/iri_asterx1_gps/gps_nav", 1000, &RawReceiverNode::navCallback, this);
    fixLlaSub = nh.subscribe("/iri_asterx1_gps/gps", 1000, &RawReceiverNode::fixLlaCallback, this);
    fixEcefSub = nh.subscribe("/iri_asterx1_gps/gps_ecef", 1000, &RawReceiverNode::fixEcefCallback, this);

    satPrPub = nh.advertise<asterx1_node::SatPr>("/sat_pseudoranges", 5000);

    //GPStk stuff
    tropModelPtr=&noTropModel;//if there is not a tropospheric model
                              // I'm not using meteorological file, so it will be always like this
    bcestore.SearchNear();// Setting the criteria for looking up ephemeris

    numRAIMNotValid = 0;

}

RawReceiverNode::~RawReceiverNode()
{

}


void RawReceiverNode::publishSat(gpstk::SatID &prn, double pr, double x, double y, double z, double vx, double vy, double vz)
{
    asterx1_node::SatPr satPr;

    satPr.timestamp = currentTime;
    satPr.sat_id = prn.id;
    satPr.pseudorange = pr;
    satPr.x = x;
    satPr.y = y;
    satPr.z = z;
    satPr.v_x = vx;
    satPr.v_y = vy;
    satPr.v_z = vz;

    satPrPub.publish(satPr);

}



void RawReceiverNode::obsCallback(const iri_asterx1_gps::GPS_meas::ConstPtr& msg)
{
    std::cout << "OBS callback: time " << getTime(msg->time_stamp.tow, msg->time_stamp.wnc) << std::endl;

    currentTime = ros::Time::now();

    prnVec.clear();
    rangeVec.clear();

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
     ************************************************************************/
    int ret;
    gpstk::Matrix<double> calcPos;

    ret = raimSolver.PrepareAutonomousSolution(getTime(msg->time_stamp.tow, msg->time_stamp.wnc),
                                               prnVec,
                                               rangeVec,
                                               bcestore,
                                               calcPos); //satellite positions at transmit time, and the corrected pseudorange
    //Return values:  0 ok
    //               -4 ephemeris not found for all the satellites

    if(ret == -4)
    {
        std::cout << "\t% of good entries = 0/" << prnVec.size() << "\n";
        return;
    }
    //NB: verify that the number of good entries (Satellite[.] > 0) is > 4 before proceeding
    int goodEntries = 0;
    for (int i = 0; i < prnVec.size(); ++i)
    {
        if(prnVec[i].id>0)
        {
            ++goodEntries;

        }
//            else { std::cout << "Sat #" << prnVec[i].id << " is a bad entry\n"; }
    }

    std::cout << "\t% of good entries = " << goodEntries << "/" << prnVec.size() << "\n";

    for (size_t i = 0; i < prnVec.size(); ++i)
    {
        if(prnVec[i].id > 0)
        {
            std::cout << "\tPRN " << prnVec[i].id
            << "\tnew pr = " << calcPos[i][3]
            << "\tecef (" << calcPos[i][0] << ", " << calcPos[i][1] << ", " << calcPos[i][2] << ") "
            << std::endl;

            gpstk::Triple vel = bcestore.getXvt(prnVec[i], getTime(msg->time_stamp.tow, msg->time_stamp.wnc)).getVel();

            publishSat(prnVec[i], calcPos[i][3], calcPos[i][0], calcPos[i][1], calcPos[i][2], vel[0], vel[1], vel[2]);
        }
    }

    //TODO credo sia meglio pubblicare un vettore di SatPR, e non i singoli.
}

/*
 * TODO finish the filling part and test it
 */
void RawReceiverNode::navCallback(const iri_asterx1_gps::GPS_nav::ConstPtr& msg)
{
    std::cout << "### NAV callback: sat " << (short)msg->sat_id
              << ", time " << getTime(msg->time_stamp.tow, msg->time_stamp.wnc)
              << std::endl;


    gpstk::Rinex3NavData eph;

    eph.time = getTime(msg->time_stamp.tow, msg->time_stamp.wnc);
    eph.satSys = "G";
    eph.PRNID = msg->sat_id;
    eph.sat = gpstk::RinexSatID((short)msg->sat_id, gpstk::SatID::systemGPS);
    eph.HOWtime = (int)msg->time_stamp.tow / 1000;
    eph.weeknum = msg->wn;//msg->time_stamp.wnc; //******** quale dei 2?
    eph.accuracy = 1;//***************************************
    eph.health = (short)msg->health;
    eph.codeflgs = msg->ca_or_pon_l2;//***********(credo di si)
    eph.L2Pdata = msg->l2_data_flag;
    eph.IODC = (double)msg->iodc;
    eph.IODE = (double)msg->iode2;

    eph.accCode = msg->ura;//**********

    eph.Toc = msg->t_oc;
    eph.af0 = msg->a_f0;
    eph.af1 = msg->a_f1;
    eph.af2 = msg->a_f2;
    eph.Tgd = msg->t_gd;
    eph.Tgd2 = 0.0;
    eph.Cuc = msg->c_uc;
    eph.Cus = msg->c_us;
    eph.Crc = msg->c_rc;
    eph.Crs = msg->c_rs;
    eph.Cic = msg->c_ic;
    eph.Cis = msg->c_is;
    eph.Toe = msg->t_oe;
    eph.M0 = msg->m_0;
    eph.dn = msg->delta_n;
    eph.ecc = msg->e;
    eph.Ahalf = msg->sqrt_a;
    eph.OMEGA0 = msg->omega_0;
    eph.i0 = msg->i_0;
    eph.w = msg->omega;
    eph.OMEGAdot = msg->omega_dot;
    eph.idot = msg->i_dot;
    eph.fitint = 4;//msg->fit_int_flag;//****************** è 4 di solito?
//    std::cout << "oggetto creato\n";

    try
    {

        //TODO soluzione temporanea al problema che se aggiungo una eph che è gia presente crasha tutto
        if(! bcestore.isPresent(eph.sat))
        {
//            std::cout << "bcestore: adding sat " << eph.sat.id << " at time " << eph.time << " NOT present\n";
            // Add the ephemeris just created to the ephemerides store
            bcestore.addEphemeris(eph);
            std::cout << "\toggetto aggiunto\n";
        }

    }
    catch(gpstk::Exception& e)
    {
        std::cerr << eph << std::endl;
        exit(0);
    }
    catch (...)
    {
        std::cerr << "Caught an unexpected exception." << std::endl;
        exit(0);
    }

    // print recap of eph store
    //bcestore.dump(std::cout, 2);

}


void RawReceiverNode::calculateFixGPStk(const iri_asterx1_gps::GPS_meas::ConstPtr &msg)
{

    try {

        raimSolver.RAIMCompute(
                getTime(msg->time_stamp.tow, msg->time_stamp.wnc),
                prnVec,
                rangeVec,
                bcestore,
                tropModelPtr);

        if (raimSolver.isValid())
        {
            // Vector "Solution" holds the coordinates, expressed in meters
            // in an Earth Centered, Earth Fixed (ECEF) reference frame.
            std::cout << std::setprecision(12) << raimSolver.Solution[0] << " ";
            std::cout << raimSolver.Solution[1] << " ";
            std::cout << raimSolver.Solution[2];
            std::cout << " --> LLR: ";

            gpstk::Triple sol_xyz, sol_llr;
            sol_xyz[0] = raimSolver.Solution[0];
            sol_xyz[1] = raimSolver.Solution[1];
            sol_xyz[2] = raimSolver.Solution[2];
            gpstk::WGS84Ellipsoid WGS84;
            double AEarth = WGS84.a();
            double eccSquared = WGS84.eccSquared();

            gpstk::Position::convertCartesianToGeodetic(sol_xyz, sol_llr, AEarth, eccSquared);

            std::cout << sol_llr[0] << " ";
            std::cout << sol_llr[1] << " ";
            std::cout << sol_llr[2];
            std::cout << std::endl;

            numRAIMNotValid = 0; //azzera counter risultati non validi consecutivi
        }
        else
        {
            std::cout << "raimSolver NOT Valid" << std::endl;
            numRAIMNotValid++;



            if(numRAIMNotValid>10)
                exit(0);
        }
    }
    catch (gpstk::Exception& e)
    {
        std::cerr << e << std::endl;
        exit(0);
    }
}

gpstk::CivilTime RawReceiverNode::getTime(unsigned int tow, unsigned short wnc)
{
    return gpstk::CivilTime(gpstk::GPSWeekSecond(wnc, (double)tow/1000, gpstk::TimeSystem::GPS));
}

void RawReceiverNode::fixLlaCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if(printFixLla)
    {
        std::cout << "%%%%%%%%%%%%%%%%    FIX long lat alt = (" << msg->longitude << ", " << msg->latitude << ", " << msg->altitude << ")\n";
    }
}

void RawReceiverNode::fixEcefCallback(const iri_asterx1_gps::NavSatFix_ecef::ConstPtr &msg)
{
    if(printFixEcef)
    {
        std::cout << "%%%%%%%%%%%%%%%%    FIX ecef = (" << msg->x << ", " << msg->y << ", " << msg->z << ")\n";
    }
}

