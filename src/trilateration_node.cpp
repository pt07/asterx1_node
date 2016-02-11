//
// Created by ptirindelli on 29/01/16.
//

#include "trilateration_node.h"

TrilaterationNode::TrilaterationNode():
        nh(ros::this_node::getName())
{
    // Listeners
    pseudorangeSub = nh.subscribe("/sat_pseudoranges", 1000, &TrilaterationNode::pseudorangeCallback, this);
    fixEcefSub = nh.subscribe("/iri_asterx1_gps/gps_ecef", 1000, &TrilaterationNode::fixEcefCallback, this);
    raimEcefSub = nh.subscribe("/raim_fix", 1000, &TrilaterationNode::raimEcefCallback, this);
    wolfEcefSub = nh.subscribe("/wolf_fix", 1000, &TrilaterationNode::wolfEcefCallback, this);

    //Publisher
    estFixPub = nh.advertise<iri_asterx1_gps::NavSatFix_ecef>("/est_fix", 5000);


    //To clean the files
    std::ofstream myfile (PATH_EST_POS); myfile<<"latitude,longitude,altitude\n";  myfile.close();
    myfile.open(PATH_RAIM_POS);  myfile<<"latitude,longitude,altitude\n";   myfile.close();
    myfile.open(PATH_REAL_POS);  myfile<<"latitude,longitude,altitude\n";   myfile.close();
    myfile.open(PATH_WOLF_POS);  myfile<<"latitude,longitude,altitude\n";   myfile.close();

}


TrilaterationNode::~TrilaterationNode() { }


void TrilaterationNode::pseudorangeCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr &msg)
{
    std::cout << "Received " << msg->measurements.size() << " sat obs at " << msg->time_ros << "\n";

    if(msg->measurements.size() < 4)
    {
        std::cout << "Not enough to trilaterate.\n";
        return;
    }

    std::vector<SatelliteMeasurement> measurements;


    for (int i = 0; i < msg->measurements.size(); ++i)
    {
        const iri_common_drivers_msgs::SatellitePseudorange sat = msg->measurements[i];

        SatelliteMeasurement m = {
                Point<double>(sat.x, sat.y, sat.z),
                sat.pseudorange
        };

        measurements.push_back(m);
    }


    Receiver estRec = tr.computePosition(measurements);
    Point<double> estRecLLA = ecefToLla(estRec.pos);


    std::cout << " ---> Estimation:\t" << estRec.toString() << std::endl;
    std::cout << "            real:\t " << lastFixECEF.toString() << std::endl;
    std::cout << "     diff coords:\t " << (estRec.pos + -lastFixECEF).toString() << std::endl;
    std::cout << "           error:\t  " << estRec.pos.distanceTo(lastFixECEF) << std::endl;
    std::cout << "      raim error:\t  " << estRec.pos.distanceTo(lastRaimECEF) << std::endl;
    std::cout << "     (LLA)  real:\t " << (ecefToLla(lastFixECEF)).toString() << std::endl;
    std::cout << "     (LLA)   est:\t " << estRecLLA.toString() << std::endl;


    if(saveOnDisk)
    {
        if(counterEst==SAMPLING_RATE)
        {
            writeOnFile(PATH_EST_POS, estRecLLA);
            counterEst = 0;
        }
        counterEst++;
    }

    //*************** debug purpose ********************
//    std::cout << "\nRANGES:" << std::endl;
//    double sum_r = 0, sum_e = 0;
//    for (int i = 0; i < msg->measurements.size(); ++i)
//    {
//        const asterx1_node::SatPr sat = msg->measurements[i];
//
//        Point<double> sat_pos(sat.x, sat.y, sat.z);
//
//        double sq_r = pow(sat_pos.distanceTo(lastFixECEF) - sat.pseudorange, 2);
//        double sq_e = pow(sat_pos.distanceTo(estRec.pos) - sat.pseudorange, 2);
//
//        std::cout << "\t*(*)\t*sat" << sat.sat_id << ": " << sat.pseudorange << "\tpr sent" << std::endl;
//        std::cout << "\t (R)\t-sat" << sat.sat_id << ": " << sat_pos.distanceTo(lastFixECEF) << "\trange with REAL pos" << std::endl;
//        std::cout << "\t (E)\t sat" << sat.sat_id << ": " << sat_pos.distanceTo(estRec.pos) << "\trange with est pos" << std::endl;
//
//        sum_r += sq_r;
//        sum_e += sq_e;
//    }
//    std::cout << std::endl;
//    std::cout << "\t*(R)\t*mean error " << sqrt(sum_r/msg->measurements.size()) << std::endl;
//    std::cout << "\t (E)\t mean error " << sqrt(sum_e/msg->measurements.size()) << std::endl;


    //************ END DEBUG PRINTINGS ***********


    // Sets the guess for the next simulation in the actual position
    //TODO vedi se serve effetivamente
    tr.setInitialReceiverGuess(estRec);

    // publish result

    iri_asterx1_gps::NavSatFix_ecef estFixMsg;
    //TODO fill up header etc
    estFixMsg.x = estRec.pos.getX();
    estFixMsg.y = estRec.pos.getY();
    estFixMsg.z = estRec.pos.getZ();

    estFixPub.publish(estFixMsg);
}

Point<double> TrilaterationNode::ecefToLla(const Point<double> &ecef)
{
    return ecefToLla(ecef.coords[0], ecef.coords[1], ecef.coords[2]);
}


Point<double> TrilaterationNode::ecefToLla(double x, double y, double z)
{
    gpstk::Triple sol_ecef, sol_llr;
    sol_ecef[0] = x;
    sol_ecef[1] = y;
    sol_ecef[2] = z;

    // conversione in coordinate geodetic
    gpstk::WGS84Ellipsoid WGS84;
    double AEarth = WGS84.a();
    double eccSquared = WGS84.eccSquared();

    gpstk::Position::convertCartesianToGeodetic(sol_ecef, sol_llr, AEarth , eccSquared);

    return Point<double>(sol_llr[0], sol_llr[1], sol_llr[2]);
}

void TrilaterationNode::fixEcefCallback(const iri_asterx1_gps::NavSatFix_ecef::ConstPtr &msg)
{
    //std::cout << "%%%%  FIX ecef = (" << msg->x << ", " << msg->y << ", " << msg->z << ")\n";
    lastFixECEF.setX(msg->x);
    lastFixECEF.setY(msg->y);
    lastFixECEF.setZ(msg->z);

    if(saveOnDisk)
    {
        if(counterReal==SAMPLING_RATE)
        {
            writeOnFile(PATH_REAL_POS, ecefToLla(lastFixECEF));
            counterReal = 0;
        }
        counterReal++;
    }
}


void TrilaterationNode::raimEcefCallback(const iri_asterx1_gps::NavSatFix_ecef::ConstPtr &msg)
{
    //std::cout << "%%%%  RAIM ecef = (" << msg->x << ", " << msg->y << ", " << msg->z << ")\n";
    lastRaimECEF.setX(msg->x);
    lastRaimECEF.setY(msg->y);
    lastRaimECEF.setZ(msg->z);

    if(saveOnDisk)
    {
        if(counterRaim==SAMPLING_RATE)
        {
            writeOnFile(PATH_RAIM_POS, ecefToLla(lastRaimECEF));
            counterRaim = 0;
        }
        counterRaim++;
    }
}


void TrilaterationNode::wolfEcefCallback(const iri_asterx1_gps::NavSatFix_ecef::ConstPtr &msg)
{
    //std::cout << "%%%%  RAIM ecef = (" << msg->x << ", " << msg->y << ", " << msg->z << ")\n";
    lastWolfECEF.setX(msg->x);
    lastWolfECEF.setY(msg->y);
    lastWolfECEF.setZ(msg->z);

    if(saveOnDisk)
    {
        if(counterRaim==SAMPLING_RATE)
        {
            writeOnFile(PATH_WOLF_POS, ecefToLla(lastWolfECEF));
            counterWolf = 0;
        }
        counterWolf++;
    }
}


bool TrilaterationNode::writeOnFile(std::string path, Point<double> p)
{
    return writeOnFile(path, p.getX(), p.getY(), p.getZ());
}

bool TrilaterationNode::writeOnFile(std::string path, double x, double y, double z)
{
    Point<double> p(x, y, z);
    if(abs(x)+abs(y)+abs(z) > 5000.0)
    {
        std::cout << "-_çḉ34235!·$$%&/&%;  " << p.toString() << " non stampato\n";
    }


    std::ofstream myfile (path, std::ios::app);
    if (myfile.is_open())
    {
        std::cout << "WRITING " << p.toString() << "\n";
        // latitude,longitude,elevation
        myfile << std::setprecision(12);
        myfile << x << "," << y << "," << z << "\n";
        myfile.close();
        return true;
    }

    return false;
}
