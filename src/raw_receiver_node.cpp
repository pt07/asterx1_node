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
//    eph.satID.system = TODO SatelliteSystem.GPS o qualcosa del genere

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


}

