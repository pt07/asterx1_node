#include "raw_receiver_node.h"


RawReceiverNode::RawReceiverNode() :
        nh(ros::this_node::getName())
{
    obsSub = nh.subscribe("/iri_asterx1_gps/gps_meas", 1000, &RawReceiverNode::obsCallback, this);
    navSub = nh.subscribe("/iri_asterx1_gps/gps_nav", 1000, &RawReceiverNode::navCallback, this);

    //GPStk stuff
    tropModelPtr=&noTropModel;//if there is not a tropospheric model
                              // I'm not using meteorological file, so it will be always like this
    bcestore.SearchNear();// Setting the criteria for looking up ephemeris

    numNavMsgRec = 0;
    numRAIMNotValid = 0;


}

RawReceiverNode::~RawReceiverNode()
{

}

void RawReceiverNode::obsCallback(const iri_asterx1_gps::GPS_meas::ConstPtr& msg)
{
    if(numNavMsgRec>=4)
    {
        std::cout << "New observations at tow: " << msg->time_stamp.tow << std::endl;

        for (int i = 0; i < msg->type1_info.size(); ++i)
        {
            const iri_asterx1_gps::GPS_meas_type1 meas_t1 = msg->type1_info[i];

            std::cout << "\t- sat_id: " << (short)meas_t1.sat_id// << std::endl
            << std::setprecision(12)
            << "\tpseudo_range: " << (long double)meas_t1.pseudo_range << std::endl;

            double P1 = meas_t1.pseudo_range;

            // non abbiamo le frequenze p2, quindi non possiamo calcolare una correzione atmosferica

            prnVec.push_back(gpstk::SatID((short)meas_t1.sat_id, gpstk::SatID::systemGPS));
            rangeVec.push_back(P1);

        }

        // Solve a GPS fix
        raimSolver.RMSLimit = 3e6;


        try {

            raimSolver.RAIMCompute(
                    getTime(msg->time_stamp.tow,
                            msg->time_stamp.wnc), //TODO controlla che sia questo il tempo richiesto!!
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


    } else {
//        std::cout << ".";
    }
}

/*
 * TODO finish the filling part and test it
 */
void RawReceiverNode::navCallback(const iri_asterx1_gps::GPS_nav::ConstPtr& msg)
{
    std::cout << "### NAV data for sat " << (short)msg->sat_id
              << " at tow: " << msg->time_stamp.tow
              << "\t| iodt: " << (int)msg->iodc
              << std::endl;


    gpstk::GPSEphemeris eph;



    /*
     * Filling GPSEphemeris eph
     *
     *      commented row means not found in gps data
     *      //*********** means check again
     */
//    eph.transmitTime = ;
    eph.HOWtime = (int)msg->time_stamp.tow /1000;
    eph.IODE = msg->iode2; //******
    eph.IODC = msg->iodc;
    eph.health = msg->health;
    eph.accuracyFlag = msg->ura;//*****************
//    eph.accuracy = msg->;
    eph.Tgd = msg->t_gd;//***********
//    eph.codeflags = msg->;
    eph.L2Pdata = msg->l2_data_flag;
//    eph.fitDuration = msg->;
//    eph.fitint = msg->;

    eph.satID.id = msg->sat_id;
    eph.satID.system = gpstk::SatID::systemGPS;

//    eph.obsID = msg->;Defines carrier and tracking code.
//    eph.ctToe = msg->;Ephemeris epoch.
//    eph.ctToc = msg->;Clock Epoch
    eph.af0 = msg->a_f0;
    eph.af1 = msg->a_f1;
    eph.af2 = msg->a_f2;
    eph.M0 = msg->m_0;
    eph.dn = msg->delta_n;
    eph.ecc = msg->e;
//    eph.A = msg->;Semi-major axis (m)
    eph.OMEGA0 = msg->omega_0;
    eph.i0 = msg->i_0;
//    eph.w = msg->;Argument of perigee (rad)
    eph.OMEGAdot = msg->omega_dot;
    eph.idot = msg->i_dot;
//    eph.dndot = msg->; NON C'È
//    eph.Adot = msg->;Rate of semi-major axis (m/sec)
    eph.Cuc = msg->c_uc;
    eph.Cus = msg->c_us;
    eph.Crc = msg->c_rc;
    eph.Crs = msg->c_rs;
    eph.Cic = msg->c_ic;
    eph.Cis = msg->c_is;
//    eph.beginValid = msg->;
//    eph.endValid = msg->;


/*
 * Fields from gps data not used
 *
    Header header
    int16 wn
    uint8 ca_or_pon_l2
    uint8 fit_int_flag
    uint32 t_oc
    float64 sqrt_a
    uint32 t_oe
    float64 omega
    uint16 wnt_oc
    uint16 wnt_oe
*/



    // Add the ephemeris just created to the ephemerides store
    bcestore.addEphemeris(eph);

    numNavMsgRec++;
}

/**
 * tow: time of week in milliseconds
 * wnc: week number count
 */
gpstk::CommonTime RawReceiverNode::getTime(long tow, int wnc)
{
    //TODO sta conversione è fatta a caso

    gpstk::CommonTime ts(gpstk::TimeSystem::GPS);

    double fsod = (tow % 1000)/1000;
    tow /= 1000;
    long sod = tow % (24*60*60);
    long day = tow / (24*60*60) + 7*wnc;

    ts.set(day, sod, fsod, gpstk::TimeSystem::GPS);



    return ts;
}
