/*
 * Steps:
 *
 * rviz  (world main frame)
 * rosbag play ~/bagfiles/asd.bag -r 5 -l
 * rosrun asterx1_node raw_receiver_node
 * rosrun asterx1_node trilat_node
 * rosrun asterx1_node viz_helper
 */

#include <iostream>
#include "raw_receiver_node.h"


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