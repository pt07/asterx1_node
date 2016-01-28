//
// Created by ptirindelli on 28/01/16.
//

#include <iostream>
#include "viz_helper_node.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualization_helper_node");

    VisualizationHelperNode vhNode;

    ros::spin();

    return 0;
}