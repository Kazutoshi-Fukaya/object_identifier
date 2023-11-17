#ifndef OBJECT_IDENTIFIER_H_
#define OBJECT_IDENTIFIER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
// #include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

// Custom msg
#include "object_detector_msgs/ObjectPositionsWithImage.h"
#include "object_identifier_msgs/ObjectPositionsWithID.h"

namespace object_identifier
{
class ObjectIdentifier {
public:
    ObjectIdentifier();
    void process();

private:

    // node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber ops_with_img_in_;
    ros::Subscriber odom_sub_;

    // publisher
    ros::Publisher ops_with_id_out_;
    ros::Publisher rops_with_id_out_;

    // callback
    void ops_with_img_callback(const object_detector_msgs::ObjectPositionsWithImageConstPtr& msg);
    // void odom_callback(const nav_msgs::OdometryConstPtr& msg);

    // tf
    boost::shared_ptr<tf2_ros::Buffer> buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> listener_;

    object_identifier_msgs::ObjectPositionsWithID last_ops_with_id_;
    int object_id_counter_;

    // parameter
    bool IS_ID_DEBUG_;
    int HZ_;
    double OBJECT_DISTANCE_THRESHOLD_;
    std::string MAP_FRAME_ID_;
    std::string BASE_LINK_FRAME_ID_;
    std::string CAMERA_FRAME_ID_;
};
}

#endif