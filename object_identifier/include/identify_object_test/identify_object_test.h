#ifndef IDENTIFY_OBJECT_TEST_H_
#define IDENTIFY_OBJECT_TEST_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

// utils
#include "dbow3/vocabulary/vocabulary.h"
#include "dbow3/database/database.h"
#include "object_identifier/images.h"

namespace object_identifier
{
class IdentifyObjectTest {
public:
    IdentifyObjectTest();
    ~IdentifyObjectTest();
    void process();

private:
    void set_detector_mode(std::string detector_mode);
    void load_reference_images(std::string reference_images_path);
    void load_input_image(std::string input_image_path);
    void calc_features(Image& image,std::string name,cv::Mat img);
    void create_database(std::string reference_images_path);
    void identify_object(cv::Mat input_image,int& object_id);
    std::vector<std::string> split(std::string& input,char delimiter);

    // node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    std::vector<cv::Mat> features_;
    dbow3::Vocabulary* vocabulary_;
    dbow3::Database* database_;
    cv::Ptr<cv::Feature2D> detector_;
    std::vector<Images> reference_images_;
    cv::Mat input_image_;

    // parameters
    int HZ_;
    int VOCABULARY_K_;
    int VOCABULARY_L_;
    int DATABASE_MAX_RESULTS_;
    std::string DETECTOR_MODE_;
    std::string REFERENCE_IMAGES_PATH_;
    std::string INPUT_IMAGE_PATH_;
};
} // namespace object_identifier

#endif  // IDENTIFY_OBJECT_TEST_H_