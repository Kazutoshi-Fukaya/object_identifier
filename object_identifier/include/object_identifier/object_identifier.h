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
#include <visualization_msgs/MarkerArray.h>

// utils
#include "dbow3/vocabulary/vocabulary.h"
#include "dbow3/database/database.h"
#include "object_identifier/images.h"

// Custom msg
#include "object_detector_msgs/ObjectPositionsWithImage.h"
#include "object_identifier_msgs/ObjectPositionsWithID.h"

namespace object_identifier
{
class ObjectIdentifier {
public:
    ObjectIdentifier();
    ~ObjectIdentifier();
    void process();

private:
    void set_detector_mode(std::string detector_mode);
    void load_reference_images(std::string reference_images_path);
    void calc_features(Image& image,std::string name,cv::Mat img);
    void create_database(std::string reference_images_path,std::string database_name);
    void identify_object(object_detector_msgs::ObjectPositionWithImage input_msg,int& object_id);
    void add_new_image(int object_id,std::string name, sensor_msgs::Image image);
    std::vector<std::string> split(std::string& input,char delimiter);
    visualization_msgs::Marker create_init_marker();
    geometry_msgs::Pose get_pose(double x,double y,double z);
    std_msgs::ColorRGBA get_color(double r,double g,double b);

    // node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber ops_with_img_in_;
    ros::Subscriber odom_sub_;

    // publisher
    ros::Publisher ops_with_id_out_;
    ros::Publisher rops_with_id_out_;
    ros::Publisher markers_pub_;
    ros::Publisher id_markers_pub_;

    // callback
    void ops_with_img_callback(const object_detector_msgs::ObjectPositionsWithImageConstPtr& msg);
    // void odom_callback(const nav_msgs::OdometryConstPtr& msg);

    // tf
    boost::shared_ptr<tf2_ros::Buffer> buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> listener_;

    object_identifier_msgs::ObjectPositionsWithID last_ops_with_id_;
    std::vector<object_identifier_msgs::ObjectPositionsWithID> past_ops_with_id_list_;
    int object_id_counter_;
    int object_max_id_;

    // features
    std::vector<cv::Mat> features_;

    // vocabulary
    dbow3::Vocabulary* vocabulary_;

    // database
    dbow3::Database* database_;

    // detector
    cv::Ptr<cv::Feature2D> detector_;

    // Reference
    std::vector<Images> reference_images_;

    // parameter
    bool IS_ID_DEBUG_;
    bool USE_VISUALIZATION_;
    bool USE_DATABASE_;
    bool USE_EXISTING_VOCABULARY_;
    bool USE_EXISTING_DATABASE_;
    bool SAVE_VOCABULARY_;
    bool SAVE_DATABASE_;
    bool ADD_NEW_OBJECT_;
    int HZ_;
    int VOCABULARY_K_;
    int VOCABULARY_L_;
    int TRACKING_FRAME_NUM_;
    int TRACKING_THRESHOLD_NUM_;
    int DATABASE_MAX_RESULTS_;
    double OBJECT_DISTANCE_THRESHOLD_;
    double OBJECT_SEARCH_RADIUS_;
    std::string MAP_FRAME_ID_;
    std::string BASE_LINK_FRAME_ID_;
    std::string CAMERA_FRAME_ID_;
    std::string REFERENCE_IMAGES_PATH_;
    std::string VOCABULARY_NAME_;
    std::string DATABASE_NAME_;
};
}

#endif