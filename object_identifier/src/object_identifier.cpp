#include "object_identifier/object_identifier.h"

using namespace object_identifier;

ObjectIdentifier::ObjectIdentifier()
    : private_nh_("~")
{
    // parameter
    private_nh_.param("IS_ID_DEBUG", IS_ID_DEBUG_, false);
    private_nh_.param("HZ", HZ_, 10);
    private_nh_.param("OBJECT_DISTANCE_THRESHOLD", OBJECT_DISTANCE_THRESHOLD_, 0.1);
    private_nh_.param("MAP_FRAME_ID", MAP_FRAME_ID_, std::string("map"));
    private_nh_.param("BASE_LINK_FRAME_ID", BASE_LINK_FRAME_ID_, std::string("base_link"));
    private_nh_.param("CAMERA_FRAME_ID", CAMERA_FRAME_ID_, std::string("base_link"));

    // subscriber
    ops_with_img_in_ = nh_.subscribe("ops_with_img_in", 1, &ObjectIdentifier::ops_with_img_callback, this);

    // publisher
    ops_with_id_out_ = nh_.advertise<object_identifier_msgs::ObjectPositionsWithID>("ops_with_id_out", 1);
    rops_with_id_out_ = nh_.advertise<object_identifier_msgs::ObjectPositionsWithID>("rops_with_id_out", 1);

    // tf
    buffer_.reset(new tf2_ros::Buffer);
    listener_.reset(new tf2_ros::TransformListener(*buffer_));

    object_id_counter_ = 0;
}

void ObjectIdentifier::ops_with_img_callback(const object_detector_msgs::ObjectPositionsWithImageConstPtr& msg)
{    
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = buffer_->lookupTransform(MAP_FRAME_ID_, CAMERA_FRAME_ID_, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }

    object_identifier_msgs::ObjectPositionsWithID ops_with_id;
    ops_with_id.header = msg->header;
    ops_with_id.header.frame_id = MAP_FRAME_ID_;

    object_identifier_msgs::ObjectPositionsWithID rops_with_id;
    rops_with_id.header = msg->header;

    std::vector<int> ids;
    for(const auto &op : msg->object_positions_with_img)
    {
        geometry_msgs::PoseStamped relative_pose;
        relative_pose.pose.position.x = op.x;
        relative_pose.pose.position.y = op.y;
        relative_pose.pose.position.z = op.z;
        relative_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,0,1));

        geometry_msgs::PoseStamped transformed_pose;
        tf2::doTransform(relative_pose, transformed_pose, transform_stamped);

        int nearest_id = -1;
        double nearest_distance = 1000000;
        for(const auto &last_op : last_ops_with_id_.object_positions_with_id)
        {
            double distance = sqrt(pow(last_op.x - transformed_pose.pose.position.x, 2) + pow(last_op.y - transformed_pose.pose.position.y, 2) + pow(last_op.z - transformed_pose.pose.position.z, 2));
            if(distance < OBJECT_DISTANCE_THRESHOLD_)
            {
                if(distance < nearest_distance)
                {
                    nearest_id = last_op.id;
                    nearest_distance = distance;
                }
            }
        }
        if(IS_ID_DEBUG_)
        {
            if(nearest_id == -1)
            {
                nearest_id = object_id_counter_;
                object_id_counter_++;
            }
        }
        object_identifier_msgs::ObjectPositionWithID op_with_id;
        op_with_id.x = transformed_pose.pose.position.x;
        op_with_id.y = transformed_pose.pose.position.y;
        op_with_id.z = transformed_pose.pose.position.z;
        op_with_id.id = nearest_id;
        ops_with_id.object_positions_with_id.push_back(op_with_id);

        object_identifier_msgs::ObjectPositionWithID rop_with_id;
        rop_with_id.x = op.x;
        rop_with_id.y = op.y;
        rop_with_id.z = op.z;
        rop_with_id.id = nearest_id;
        rops_with_id.object_positions_with_id.push_back(rop_with_id);

        ids.push_back(nearest_id);
    }

    ops_with_id_out_.publish(ops_with_id);
    rops_with_id_out_.publish(rops_with_id);

    last_ops_with_id_ = ops_with_id;

    if(IS_ID_DEBUG_)
    {
        std::cout << "ids: ";
        for(const auto &id : ids)
        {
            std::cout << id << ", ";
        }
        std::cout << std::endl;
    }
}

void ObjectIdentifier::process()
{
    ros::Rate loop_rate(HZ_);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}