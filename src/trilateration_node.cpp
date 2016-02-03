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

    //Publisher
    estFixPub = nh.advertise<iri_asterx1_gps::NavSatFix_ecef>("/est_fix", 5000);

}


TrilaterationNode::~TrilaterationNode() { }


void TrilaterationNode::pseudorangeCallback(const asterx1_node::SatPrArray::ConstPtr &msg)
{
    std::cout << "Received " << msg->measurements.size() << " sat obs at " << msg->timestamp << "\n";


    std::vector<SatelliteMeasurement> measurements;


    for (int i = 0; i < msg->measurements.size(); ++i)
    {
        const asterx1_node::SatPr sat = msg->measurements[i];

        SatelliteMeasurement m = {
                Point<double>(sat.x, sat.y, sat.z),
                sat.pseudorange
        };

        measurements.push_back(m);
    }


    Receiver estRec = tr.computePosition(measurements);

    std::cout << " ---> Estimation:\t" << estRec.toString() << std::endl;
    std::cout << "            real:\t " << lastFix.toString() << std::endl;
    std::cout << "     diff coords:\t " << (estRec.pos + -lastFix).toString() << std::endl;
    std::cout << "           error:\t  " << estRec.pos.distanceTo(lastFix) << std::endl;


    //*************** debug purpose ********************
    std::cout << "\nRANGES:" << std::endl;
    double sum_r = 0, sum_e = 0;
    for (int i = 0; i < msg->measurements.size(); ++i)
    {
        const asterx1_node::SatPr sat = msg->measurements[i];

        Point<double> sat_pos(sat.x, sat.y, sat.z);

        double sq_r = pow(sat_pos.distanceTo(lastFix)    - sat.pseudorange, 2);
        double sq_e = pow(sat_pos.distanceTo(estRec.pos) - sat.pseudorange, 2);

        std::cout << "\t*(*)\t*sat" << sat.sat_id << ": " << sat.pseudorange << "\tpr sent" << std::endl;
        std::cout << "\t (R)\t-sat" << sat.sat_id << ": " << sat_pos.distanceTo(lastFix) << "\trange with REAL pos" << std::endl;
        std::cout << "\t (E)\t sat" << sat.sat_id << ": " << sat_pos.distanceTo(estRec.pos) << "\trange with est pos" << std::endl;

        sum_r += sq_r;
        sum_e += sq_e;
    }


    std::cout << std::endl;
    std::cout << "\t*(R)\t*mean error " << sqrt(sum_r/msg->measurements.size()) << std::endl;
    std::cout << "\t (E)\t mean error " << sqrt(sum_e/msg->measurements.size()) << std::endl;


    //************ END DEBUG PRINTINGS ***********




    // Sets the guess for the next simulation in the actual position
    tr.setInitialReceiverGuess(estRec);

    // publish result

    iri_asterx1_gps::NavSatFix_ecef estFixMsg;
    //TODO fill up header etc
    estFixMsg.x = estRec.pos.getX();
    estFixMsg.y = estRec.pos.getY();
    estFixMsg.z = estRec.pos.getZ();

    estFixPub.publish(estFixMsg);
}


void TrilaterationNode::fixEcefCallback(const iri_asterx1_gps::NavSatFix_ecef::ConstPtr &msg)
{
    //std::cout << "%%%%  FIX ecef = (" << msg->x << ", " << msg->y << ", " << msg->z << ")\n";
    lastFix.setX(msg->x);
    lastFix.setY(msg->y);
    lastFix.setZ(msg->z);
}

