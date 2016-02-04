//
// Created by ptirindelli on 28/01/16.
//

#include <iostream>
#include "viz_helper_node.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "viz_helper_node");

    VizHelperNode vhNode;

    ros::spin();

    return 0;
}