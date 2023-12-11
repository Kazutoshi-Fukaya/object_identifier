#include "object_identifier/object_identifier.h"

using namespace dbow3;
using namespace object_identifier;

ObjectIdentifier::ObjectIdentifier()
    : private_nh_("~")
{
    // parameter
    private_nh_.param("IS_ID_DEBUG", IS_ID_DEBUG_, false);
    private_nh_.param("USE_VISUALIZATION", USE_VISUALIZATION_, false);
    private_nh_.param("USE_DATABASE", USE_DATABASE_, false);
    private_nh_.param("USE_EXISTING_VOCABULARY", USE_EXISTING_VOCABULARY_, false);
    private_nh_.param("USE_EXISTING_DATABASE", USE_EXISTING_DATABASE_, false);
    private_nh_.param("SAVE_VOCABULARY", SAVE_VOCABULARY_, false);
    private_nh_.param("SAVE_DATABASE", SAVE_DATABASE_, false);
    private_nh_.param("HZ", HZ_, 10);
    private_nh_.param("VOCABULARY_K", VOCABULARY_K_, 9);
    private_nh_.param("VOCABULARY_L", VOCABULARY_L_, 3);
    private_nh_.param("TRACKING_FRAME_NUM", TRACKING_FRAME_NUM_, 6);
    private_nh_.param("TRACKING_THRESHOLD_NUM", TRACKING_THRESHOLD_NUM_, 3);
    private_nh_.param("OBJECT_DISTANCE_THRESHOLD", OBJECT_DISTANCE_THRESHOLD_, 0.1);
    private_nh_.param("MAP_FRAME_ID", MAP_FRAME_ID_, std::string("map"));
    private_nh_.param("BASE_LINK_FRAME_ID", BASE_LINK_FRAME_ID_, std::string("base_link"));
    private_nh_.param("CAMERA_FRAME_ID", CAMERA_FRAME_ID_, std::string("base_link"));

    // detector_mode
    std::string detector_mode;
    private_nh_.param("DETECTOR_MODE", detector_mode, std::string("orb"));
    if(USE_DATABASE_) set_detector_mode(detector_mode);

    // reference images
    private_nh_.param("REFERENCE_IMAGES_PATH", REFERENCE_IMAGES_PATH_, std::string(""));
    private_nh_.param("IMAGE_MODE", IMAGE_MODE_, std::string("rgb"));
    private_nh_.param("VOCABULARY_NAME", VOCABULARY_NAME_, std::string("/dkan_voc.yml.gz"));
    private_nh_.param("DATABASE_NAME", DATABASE_NAME_, std::string("/dkan_db.yml.gz"));
    if(USE_DATABASE_)
    {
        load_reference_images(REFERENCE_IMAGES_PATH_, IMAGE_MODE_);
        create_database(REFERENCE_IMAGES_PATH_, IMAGE_MODE_, DATABASE_NAME_);
    }

    // subscriber
    ops_with_img_in_ = nh_.subscribe("ops_with_img_in", 1, &ObjectIdentifier::ops_with_img_callback, this);

    // publisher
    ops_with_id_out_ = nh_.advertise<object_identifier_msgs::ObjectPositionsWithID>("ops_with_id_out", 1);
    rops_with_id_out_ = nh_.advertise<object_identifier_msgs::ObjectPositionsWithID>("rops_with_id_out", 1);
    if(USE_VISUALIZATION_) markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("object_markers", 1);
    if(USE_VISUALIZATION_) id_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("object_id_markers", 1);

    // tf
    buffer_.reset(new tf2_ros::Buffer);
    listener_.reset(new tf2_ros::TransformListener(*buffer_));

    object_id_counter_ = 0;
}

ObjectIdentifier::~ObjectIdentifier() {}

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

        // int nearest_id = -1;
        // double nearest_distance = 1000000;
        // for(const auto &last_op : last_ops_with_id_.object_positions_with_id)
        // {
        //     double distance = sqrt(pow(last_op.x - transformed_pose.pose.position.x, 2) + pow(last_op.y - transformed_pose.pose.position.y, 2) + pow(last_op.z - transformed_pose.pose.position.z, 2));
        //     if(distance < OBJECT_DISTANCE_THRESHOLD_)
        //     {
        //         if(distance < nearest_distance)
        //         {
        //             nearest_id = last_op.id;
        //             nearest_distance = distance;
        //         }
        //     }
        // }
        int nearest_id = -1;
        int frame_count = 0;
        for(const auto &past_ops_with_id : past_ops_with_id_list_)
        {
            double nearest_distance = 1000000;
            bool is_found = false;
            for(const auto &past_op : past_ops_with_id.object_positions_with_id)
            {
                if((past_op.x - transformed_pose.pose.position.x) > OBJECT_DISTANCE_THRESHOLD_) continue;
                if((past_op.y - transformed_pose.pose.position.y) > OBJECT_DISTANCE_THRESHOLD_) continue;
                if((past_op.z - transformed_pose.pose.position.z) > OBJECT_DISTANCE_THRESHOLD_) continue;
                double distance = sqrt(pow(past_op.x - transformed_pose.pose.position.x, 2) + pow(past_op.y - transformed_pose.pose.position.y, 2) + pow(past_op.z - transformed_pose.pose.position.z, 2));
                if(distance < OBJECT_DISTANCE_THRESHOLD_)
                {
                    is_found = true;
                    if(distance < nearest_distance)
                    {
                        nearest_id = past_op.id;
                        nearest_distance = distance;
                    }
                }
            }
            if(is_found) frame_count++;
        }
        std::cout << "frame_count: " << frame_count << std::endl;
        if((nearest_id == -1) && (frame_count >= TRACKING_THRESHOLD_NUM_))
        {
            if (USE_DATABASE_) identify_object(op, nearest_id);
            else if (IS_ID_DEBUG_)
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

    // last_ops_with_id_ = ops_with_id;
    past_ops_with_id_list_.push_back(ops_with_id);
    if(past_ops_with_id_list_.size() > TRACKING_FRAME_NUM_) past_ops_with_id_list_.erase(past_ops_with_id_list_.begin());

    if(IS_ID_DEBUG_)
    {
        std::cout << "ids: ";
        for(const auto &id : ids)
        {
            std::cout << id << ", ";
        }
        std::cout << std::endl;
    }

    if(USE_VISUALIZATION_)
    {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::MarkerArray id_array;
        for(const auto &op : ops_with_id.object_positions_with_id)
        {
            visualization_msgs::Marker marker = create_init_marker();
            marker.id = op.id;
            marker.pose = get_pose(op.x, op.y, op.z);
            marker.color = get_color(1.0, 0.0, 0.0);
            marker_array.markers.emplace_back(marker);

            visualization_msgs::Marker id_marker = create_init_marker();
            id_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            id_marker.id = op.id;
            id_marker.pose = get_pose(op.x, op.y, op.z);
            id_marker.color = get_color(1.0, 1.0, 1.0);
            id_marker.text = std::to_string(op.id);
            id_array.markers.emplace_back(id_marker);
        }
        markers_pub_.publish(marker_array);
        id_markers_pub_.publish(id_array);
        std::cout << "marker_array: " << marker_array.markers.size() << std::endl;
    }
}

void ObjectIdentifier::set_detector_mode(std::string detector_mode)
{
    if(detector_mode == "orb") detector_ = cv::ORB::create();
    else if(detector_mode == "brisk") detector_ = cv::BRISK::create();
    else if(detector_mode == "akaze") detector_ = cv::AKAZE::create();
    else{
        ROS_WARN("No applicable 'detector_mode'. Please select 'orb', 'brisk' or 'akaze'");
        ROS_INFO("Set 'orb'");
        detector_ = cv::ORB::create();
    }
}

void ObjectIdentifier::load_reference_images(std::string reference_images_path,std::string image_mode)
{
    // load csv file
    std::cout << "=== Load Reference Images ===" << std::endl;
    std::string file_name = reference_images_path + "save.txt";
    std::cout << "load: " << file_name << std::endl;
    std::ifstream ifs(file_name);
    std::string line;
    while(std::getline(ifs,line)){
        std::vector<std::string> strvec = split(line,',');
        try{
            std::string equ_name = static_cast<std::string>(strvec[0]);
            std::string rgb_name = static_cast<std::string>(strvec[1]);
            int object_id = static_cast<int>(std::stoi(strvec[2]));

            Images images(object_id);
            cv::Mat rgb_image, equ_image;
            if(image_mode == "rgb"){
                rgb_image = cv::imread(reference_images_path + rgb_name,0);
                if(rgb_image.empty()) break;
                std::cout << "file_name: " << rgb_name << std::endl;
                Image rgb;
                calc_features(rgb,rgb_name,rgb_image);
                images.set_rgb_image(rgb);
            }
            else if(image_mode == "equ"){
                equ_image = cv::imread(reference_images_path + equ_name,0);
                if(equ_image.empty()) break;
                std::cout << "file name: " << equ_name << std::endl;
                Image equ;
                calc_features(equ,equ_name,equ_image);
                images.set_equ_image(equ);
            }
            else{
                ROS_ERROR("No applicable 'image_mode'. Please select 'rgb' or 'equ'");
                return;
            }
            reference_images_.emplace_back(images);
        }
        catch(const std::invalid_argument& e){
            ROS_ERROR("Invalid argument: %s", e.what());
            return;
        }
        catch(const std::out_of_range& e){
            ROS_ERROR("Out of Range: %s", e.what());
            return;
        }
    }
    ifs.close();
    std::cout << "=============================" << std::endl;
}

void ObjectIdentifier::calc_features(Image& image,std::string name,cv::Mat img)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detector_->detectAndCompute(img,cv::Mat(),keypoints,descriptors);
    image.set_params(name,img,keypoints,descriptors);
}

void ObjectIdentifier::create_database(std::string reference_images_path,std::string image_mode,std::string database_name)
{
    if(image_mode == "rgb"){
        for(const auto &ri : reference_images_) features_.emplace_back(ri.rgb.descriptor);
    }
    else if(image_mode == "equ"){
        for(const auto &ri : reference_images_) features_.emplace_back(ri.equ.descriptor);
    }
    else{
        ROS_ERROR("No applicable 'image_mode'. Please select 'rgb' or 'equ'");
        return;
    }

    if(USE_EXISTING_DATABASE_){
        std::cout << "=== Load Database ===" << std::endl;
        std::string database_file = reference_images_path + image_mode + DATABASE_NAME_;
        std::cout << "load: " << database_file << std::endl;
        database_ = new Database(database_file);
        std::cout << "=== Success to Load Database ===" << std::endl;
    }
    else if(USE_EXISTING_VOCABULARY_){
        std::cout << "=== Load Vocabulary ===" << std::endl;
        std::string vocabulary_file = reference_images_path + image_mode + VOCABULARY_NAME_;
        std::cout << "load: " << vocabulary_file << std::endl;
        vocabulary_ = new Vocabulary(vocabulary_file);
        std::cout << "=== Success to Load Vocabulary ===" << std::endl;
    }
    else{
        std::cout << "=== Create Vocabulary ===" << std::endl;
        std::cout << "k: " << VOCABULARY_K_ << std::endl;
        std::cout << "L: " << VOCABULARY_L_ << std::endl;
        // Vocabulary voc(VOCABULARY_K_,VOCABULARY_L_,TF_IDF,L1_NORM);
        vocabulary_ = new Vocabulary(VOCABULARY_K_,VOCABULARY_L_,TF_IDF,L1_NORM);
        vocabulary_->create(features_);
        std::cout << "=== Success to Create Vocabulary ===" << std::endl;
    }

    if(!USE_EXISTING_DATABASE_){
        database_ = new Database(*vocabulary_,false,0);
        std::cout << "=== Success to Create Database ===" << std::endl;
        std::cout << "=== Add Reference Images ===" << std::endl;
        for(const auto &f : features_) database_->add(f);
    }

    // info
    database_->get_info();

    if(SAVE_VOCABULARY_ && !USE_EXISTING_DATABASE_){
        std::cout << "=== Save Vocabulary ===" << std::endl;
        std::string vocabulary_file = reference_images_path + image_mode + VOCABULARY_NAME_;
        std::cout << "save: " << vocabulary_file << std::endl;
        vocabulary_->save(vocabulary_file);
        std::cout << "=== Success to Save Vocabulary ===" << std::endl;
    }
    if(SAVE_DATABASE_){
        std::cout << "=== Save Database ===" << std::endl;
        std::string database_file = reference_images_path + image_mode + DATABASE_NAME_;
        std::cout << "save: " << database_file << std::endl;
        database_->save(database_file);
        std::cout << "=== Success to Save Database ===" << std::endl;
    }
    std::cout << "=============================" << std::endl;
}

void ObjectIdentifier::identify_object(object_detector_msgs::ObjectPositionWithImage input_msg,int& object_id)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(input_msg.img, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& ex)
    {
        ROS_ERROR("Could not convert to color image");
        return;
    }

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detector_->detectAndCompute(cv_ptr->image, cv::Mat(), keypoints, descriptors);

    QueryResults ret;
    database_->query(descriptors, ret, 4);
    if(ret.empty()) return;
    object_id = ret.at(0).id;

}

std::vector<std::string> ObjectIdentifier::split(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while(std::getline(stream,field,delimiter)) result.emplace_back(field);
    return result;
}

visualization_msgs::Marker ObjectIdentifier::create_init_marker()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = MAP_FRAME_ID_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "object_identifier";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.5);
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.6;
    return marker;
}

geometry_msgs::Pose ObjectIdentifier::get_pose(double x,double y,double z)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
}

std_msgs::ColorRGBA ObjectIdentifier::get_color(double r,double g,double b)
{
    std_msgs::ColorRGBA color_msg;
    color_msg.r = r;
    color_msg.g = g;
    color_msg.b = b;
    color_msg.a = 0.7;
    return color_msg;
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