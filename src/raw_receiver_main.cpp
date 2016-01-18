#include <iostream>
#include "raw_receiver_node.h"


int main(int argc, char **argv)
{
    std::cout << "Ciao\n";

    //init ros
    ros::init(argc, argv, "raw_receiver_node");

    // Trilateration node
    RawReceiverNode rrNode;

    ros::spin();

    return 0;
}