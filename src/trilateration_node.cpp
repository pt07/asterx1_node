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

