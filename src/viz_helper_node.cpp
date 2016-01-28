//
// Created by ptirindelli on 28/01/16.
//

#include "viz_helper_node.h"

VisualizationHelperNode::VisualizationHelperNode() :
        nh(ros::this_node::getName())
{
    pseudorangeSub = nh.subscribe("/sat_pseudoranges", 1000, &VisualizationHelperNode::pseudorangeCallback, this);

    markerPub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);


    scale = KILOMETERS;
}

VisualizationHelperNode::~VisualizationHelperNode(){ }

void VisualizationHelperNode::pseudorangeCallback(const asterx1_node::SatPrArray::ConstPtr &msg)
{
    std::cout << "Visualizing " << msg->measurements.size() << " sats at " << msg->timestamp << "\n";

    for (int i = 0; i < msg->measurements.size(); ++i)
    {
        const asterx1_node::SatPr sat = msg->measurements[i];

        publishSat(sat);

    }

    //TODO eliminare da rviz i satelliti non più presenti


    publishEarth();

}

void VisualizationHelperNode::publishSat(const asterx1_node::SatPr &sat)
{
    visualization_msgs::Marker m;
    m.header.frame_id = WORLD_FRAME;
    m.header.stamp = sat.timestamp;

    m.ns = "sats";
    m.id = sat.sat_id;

    m.type = visualization_msgs::Marker::CUBE;//SPHERE;

    m.action = visualization_msgs::Marker::ADD;

    // Pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    m.pose.position.x = sat.x * scale;
    m.pose.position.y = sat.y * scale;
    m.pose.position.z = sat.z * scale;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0; // todo dovrei mettere la stessa posa del suo frame

    m.scale.x =  m.scale.y = m.scale.z = 1000;

    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 0.5;

    m.lifetime = ros::Duration();

    markerPub.publish(m);


    /*
     * TODO publish
     *          velocity
     *          odometry
     *          tutto il resto che puo interessare
     */
}

void VisualizationHelperNode::publishEarth()
{
    visualization_msgs::Marker m;
    m.header.frame_id = WORLD_FRAME;
    m.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    m.ns = "earth";
    m.id = 0;

    // Set the marker type.
    m.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    m.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    m.scale.x = EARTH_RADIUS * scale;
    m.scale.y = EARTH_RADIUS * scale;
    m.scale.z = EARTH_RADIUS * scale;

    // Set the color -- be sure to set alpha to something non-zero!
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 0.1;

    m.lifetime = ros::Duration();

    markerPub.publish(m);
}
