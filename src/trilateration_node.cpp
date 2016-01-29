//
// Created by ptirindelli on 29/01/16.
//

#include "trilateration_node.h"

TrilaterationNode::TrilaterationNode():
        nh(ros::this_node::getName())
{
    pseudorangeSub = nh.subscribe("/sat_pseudoranges", 1000, &TrilaterationNode::pseudorangeCallback, this);

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

    // Sets the guess for the next simulation in the actual position
    tr.setInitialReceiverGuess(estRec);

}
