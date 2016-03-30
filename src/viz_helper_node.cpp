//
// Created by ptirindelli on 28/01/16.
//

#include "viz_helper_node.h"

VizHelperNode::VizHelperNode() :
        nh(ros::this_node::getName())
{
    nh.param<double>("map_p_x", map_p_x, 4789373.84373);
    nh.param<double>("map_p_y", map_p_y, 177039.859069);
    nh.param<double>("map_p_z", map_p_z, 4194527.79395);
    nh.param<bool>("trajectory_mode", trajectory_mode, false);



    // Publishers
    pseudorangeSub = nh.subscribe("/sat_pseudoranges", 1000, &VizHelperNode::pseudorangeCallback, this);
    estFixSub = nh.subscribe("/est_fix", 1000, &VizHelperNode::estFixCallback, this);
//    realFixSub = nh.subscribe("/iri_asterx1_gps/gps_ecef", 1000, &VizHelperNode::realFixCallback, this);// both publisher receive the real fix. there are 2 because i have 2 different ros node that can publish
    realFixSub2 = nh.subscribe("/real_fix", 1000, &VizHelperNode::realFixCallback, this);// both publisher receive the real fix. there are 2 because i have 2 different ros node that can publish
    realFixSub = nh.subscribe("/teo/sensors/gps/gps_ecef", 1000, &VizHelperNode::realFixCallback, this);// both publisher receive the real fix. there are 2 because i have 2 different ros node that can publish

    // Listeners
    markerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
    odomAllPub = nh.advertise<nav_msgs::Odometry>("odom_all", 50);

    scale = KILOMETERS;

    sensor_fix_received = trilat_fix_received = 0;
    std::cout << std::setprecision(12);


    broadcastTFmap();

}

void VizHelperNode::broadcastTFmap()
{
    tf::Transform world2map(tf::createQuaternionFromRPY(0, 60*M_PI/180, 0), tf::Vector3(map_p_x*scale, map_p_y*scale, map_p_z*scale));
    tf_b.sendTransform(tf::StampedTransform(world2map, ros::Time::now(), WORLD_FRAME, "map"));
}

VizHelperNode::~VizHelperNode(){ }

void VizHelperNode::pseudorangeCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr &msg)
{
    if(trajectory_mode)
        return;

    std::cout << "Visualizing " << msg->measurements.size() << " sats at " << msg->time_ros << "\n";

    for (int i = 0; i < msg->measurements.size(); ++i)
    {
        const iri_common_drivers_msgs::SatellitePseudorange sat = msg->measurements[i];

        publishSat(sat, msg->time_ros);

    }

    publishEarth();

}


void VizHelperNode::publishSat(const iri_common_drivers_msgs::SatellitePseudorange &sat, ros::Time time_ros)
{
    visualization_msgs::Marker m;
    m.header.frame_id = WORLD_FRAME;
    m.header.stamp = time_ros;

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

    m.lifetime = LIFETIME_SHORT; //after tot seconds satellites are deleted

    markerPub.publish(m);


    publishSatVelocity(sat, time_ros);    // publish velocity
    // calculate rotation quaternion.
    // rotate (vedi cosa va la funzione rotateSatelliteFrame)
    Eigen::Quaterniond rotation = rotateSatelliteFrame(sat, time_ros);



    // publish odometry
    publishOdometry(sat, rotation, time_ros);


    // publish earth vector
    //TODO

    // publish sat sphere
    publishSatSphere(sat, time_ros);

}



void VizHelperNode::publishOdometry(const iri_common_drivers_msgs::SatellitePseudorange &sat, const Eigen::Quaterniond &rotation, ros::Time time_ros)
{
    /// publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = time_ros;
    odom.header.frame_id = WORLD_FRAME;

    //set the position
    odom.pose.pose.position.x = sat.x * scale;
    odom.pose.pose.position.y = sat.y * scale;
    odom.pose.pose.position.z = sat.z * scale;

    //set the orientation
    odom.pose.pose.orientation.x = rotation.x();
    odom.pose.pose.orientation.y = rotation.y();
    odom.pose.pose.orientation.z = rotation.z();
    odom.pose.pose.orientation.w = rotation.w();

    //set the velocity
    odom.child_frame_id = getSatelliteFrame(sat.sat_id);
    odom.twist.twist.linear.x = sat.v_x * scale;
    odom.twist.twist.linear.y = sat.v_y * scale;
    odom.twist.twist.linear.z = sat.v_z * scale;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;

    //publish the message
    //TODO fare i singoli odom publisher
    // odomPub[index].publish(odom);
    odomAllPub.publish(odom);
}


void VizHelperNode::publishSatVelocity(const iri_common_drivers_msgs::SatellitePseudorange &sat, ros::Time time_ros)
{
    visualization_msgs::Marker m;
    m.header.frame_id = getSatelliteFrame(sat.sat_id);
    m.header.stamp = time_ros;

    m.ns = "velocity";
    m.id = sat.sat_id;

    m.type = visualization_msgs::Marker::ARROW;

    m.action = visualization_msgs::Marker::ADD;

    // Pose of the marker.
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 1;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 0;

    m.scale.x = 3000;//2000;
    m.scale.y = 200;//100;
    m.scale.z = 200;//100;

    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
    m.color.a = 1.0;

    m.lifetime = LIFETIME_SHORT;

    markerPub.publish(m);
}


void VizHelperNode::publishSatSphere(const iri_common_drivers_msgs::SatellitePseudorange &sat, ros::Time time_ros)
{
    //std::cout << "publishing sat sphere of radius " << sat.pseudorange << "\n";

    visualization_msgs::Marker m;
    m.header.frame_id = WORLD_FRAME;
    m.header.stamp = time_ros;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    m.ns = "sat_sphere";
    m.id = sat.sat_id;

    // Set the marker type.
    m.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    m.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    m.pose.position.x = sat.x * scale;
    m.pose.position.y = sat.y * scale;
    m.pose.position.z = sat.z * scale;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 0.1;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    m.scale.x = 2 * sat.pseudorange * scale;
    m.scale.y = 2 * sat.pseudorange * scale;
    m.scale.z = 2 * sat.pseudorange * scale;

    // Set the color -- be sure to set alpha to something non-zero!
    m.color.r = 0.0f;
    m.color.g = 0.5f;
    m.color.b = 0.5f;
    m.color.a = 0.2;

    m.lifetime = LIFETIME_SHORT;

    markerPub.publish(m);

}

void VizHelperNode::publishEarth()
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
    m.scale.x = 2 * EARTH_RADIUS * scale;
    m.scale.y = 2 * EARTH_RADIUS * scale;
    m.scale.z = 2 * EARTH_RADIUS * scale;

    // Set the color -- be sure to set alpha to something non-zero!
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 0.38;

    m.lifetime = ros::Duration();

    markerPub.publish(m);
}


std::string VizHelperNode::getSatelliteFrame(int index)
{
    std::stringstream ss;
    ss << "sat_" << index;
    return ss.str();
}

Eigen::Quaterniond VizHelperNode::rotateSatelliteFrame(const iri_common_drivers_msgs::SatellitePseudorange &sat, ros::Time time_ros)
{
    Eigen::Quaterniond rotation;

    Eigen::Vector3d satVelocity(sat.v_x * scale, sat.v_y * scale, sat.v_z * scale);

    //quaternione che fa ruotare l'asse x in modo che combaci con il vettore velocita'
    rotation.setFromTwoVectors(Eigen::Vector3d::UnitX(), satVelocity);


    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = rotation.x();
    odom_quat.y = rotation.y();
    odom_quat.z = rotation.z();
    odom_quat.w = rotation.w();


    // Publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = time_ros;
    odom_trans.header.frame_id = WORLD_FRAME;				//frame padre
    odom_trans.child_frame_id = getSatelliteFrame(sat.sat_id);	//frame figlio (che sto creando ora)
    odom_trans.transform.translation.x = sat.x * scale;//traslazione dell'origine
    odom_trans.transform.translation.y = sat.y * scale;
    odom_trans.transform.translation.z = sat.z * scale;
    odom_trans.transform.rotation = odom_quat;							//rotazione

    //send the transform
    transBroadcaster.sendTransform(odom_trans);


    return rotation;
}

void VizHelperNode::realFixCallback(const iri_common_drivers_msgs::NavSatFix_ecef::ConstPtr &msg)
{
    //std::cout << "real fix " << msg->x << ", " <<msg->y << ", " <<msg->z << ", " << "\n";
    publishRealFix(msg->x, msg->y, msg->z);

    tf::Transform world2sensorFix(tf::Quaternion(0, 0, 0, 1),
                                  tf::Vector3(msg->x*scale, msg->y*scale, msg->z*scale));
    tf_b.sendTransform(tf::StampedTransform(world2sensorFix, ros::Time::now(), WORLD_FRAME, FIX_SENSOR_FRAME));

    std::cout << "publishing tf world to " << FIX_SENSOR_FRAME << ". arrivati: " << sensor_fix_received << std::endl;

    broadcastTFmap();

//    tf::Transform world2map(tf::Quaternion(0, 0, 0, 1), tf::Vector3(map_p_x*scale, map_p_y*scale, map_p_z*scale));
//    tf_b.sendTransform(tf::StampedTransform(world2map, ros::Time::now(), WORLD_FRAME, "map"));

}


void VizHelperNode::publishRealFix(double x, double y, double z)
{
    visualization_msgs::Marker m;

    m.header.frame_id = WORLD_FRAME;
    m.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    m.ns = "realFix";
    m.id = 1;

    // Set the marker type.
    m.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    m.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    m.pose.position.x = x * scale;
    m.pose.position.y = y * scale;
    m.pose.position.z = z * scale;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    m.scale.x = EARTH_RADIUS / 10 * scale;
    m.scale.y = EARTH_RADIUS / 10 * scale;
    m.scale.z = EARTH_RADIUS / 10 * scale;

    // Set the color -- be sure to set alpha to something non-zero!
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 1.0;

    m.lifetime = LIFETIME_SHORT;


    if(!trajectory_mode)
        markerPub.publish(m);


    /*
     * publish a small marker, visible only from FIX_SENSOR_FRAME
     */
    m.ns = "real_fix_detail";
    m.id = sensor_fix_received++;
    m.scale.x = m.scale.y = m.scale.z = SMALL_MARKER_SIZE;
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 0.6;
    markerPub.publish(m);
}

void VizHelperNode::estFixCallback(const iri_common_drivers_msgs::NavSatFix_ecef::ConstPtr &msg)
{
    std::cout << "estimated fix " << msg->x << ", " <<msg->y << ", " <<msg->z << ", " << "\n";
    publishEstFix(msg->x, msg->y, msg->z);
}

void VizHelperNode::publishEstFix(double x, double y, double z)
{
    visualization_msgs::Marker m;
    m.header.frame_id = WORLD_FRAME;
    m.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    m.ns = "estFix";
    m.id = 1;

    // Set the marker type.
    m.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    m.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    m.pose.position.x = x * scale;
    m.pose.position.y = y * scale;
    m.pose.position.z = z * scale;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    m.scale.x = EARTH_RADIUS / 13 * scale;
    m.scale.y = EARTH_RADIUS / 13 * scale;
    m.scale.z = EARTH_RADIUS / 13 * scale;

    // Set the color -- be sure to set alpha to something non-zero!
    m.color.r = 0.0f;
    m.color.g = 0.0f;
    m.color.b = 1.0f;
    m.color.a = 1.0;

    m.lifetime = LIFETIME_SHORT;


    if(trajectory_mode)
        markerPub.publish(m);

    /*
     * publish a small marker, visible only from FIX_SENSOR_FRAME
     */
    m.ns = "est_fix_detail";
    m.id = trilat_fix_received++;
    m.scale.x = m.scale.y = m.scale.z = SMALL_MARKER_SIZE;
    m.color.a = 0.5;
    markerPub.publish(m);
}
