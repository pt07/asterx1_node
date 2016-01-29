#include <iostream>
#include "raw_receiver_node.h"

/*
 * Steps:
 *
 * rviz  (world main frame)
 * rosbag play src/asterx1_node/rosbags/2016-01-26-12-01-25.bag --r 5
 * rosrun asterx1_node raw_receiver_node
 * rosrun asterx1_node viz_helper
 * rosrun asterx1_node trilat_node
 */

int main(int argc, char **argv)
{
    std::cout << "\n\n\t\t\tListening...\n\n\n";

    //init ros
    ros::init(argc, argv, "raw_receiver_node");

    // Trilateration node
    RawReceiverNode rrNode;

    ros::spin();

    return 0;
}