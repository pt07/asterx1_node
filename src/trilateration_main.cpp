//
// Created by ptirindelli on 29/01/16.
//


#include <iostream>
#include "trilateration_node.h"


int main(int argc, char **argv)
{
    std::cout << "ASD";

    ros::init(argc, argv, "trilat_node");

    TrilaterationNode trNode;

    ros::spin();


    return 0;
}