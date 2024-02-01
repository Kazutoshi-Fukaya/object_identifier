#include "identify_object_test/identify_object_test.h"

using namespace dbow3;
using namespace object_identifier;

IdentifyObjectTest::IdentifyObjectTest() :
    private_nh_("~")
{
    // load parameters
    private_nh_.param("HZ",HZ_,10);
    private_nh_.param("VOCABULARY_K",VOCABULARY_K_,9);
    private_nh_.param("VOCABULARY_L",VOCABULARY_L_,3);
    private_nh_.param("DATABASE_MAX_RESULTS",DATABASE_MAX_RESULTS_,10);
    private_nh_.param("DETECTOR_MODE",DETECTOR_MODE_,std::string("orb"));
    private_nh_.param("REFERENCE_IMAGES_PATH",REFERENCE_IMAGES_PATH_,std::string(""));
    private_nh_.param("INPUT_IMAGE_PATH",INPUT_IMAGE_PATH_,std::string(""));

    // set detector mode
    set_detector_mode(DETECTOR_MODE_);

    // load reference images
    load_reference_images(REFERENCE_IMAGES_PATH_);

    // load input image
    load_input_image(INPUT_IMAGE_PATH_);

    // create database
    create_database(REFERENCE_IMAGES_PATH_);

    // identify object
    int object_id;
    identify_object(input_image_,object_id);
    std::cout << "object_id: " << object_id << std::endl;
}

IdentifyObjectTest::~IdentifyObjectTest(){}

void IdentifyObjectTest::set_detector_mode(std::string detector_mode)
{
    if(detector_mode == "orb"){
        detector_ = cv::ORB::create();
    }else if(detector_mode == "akaze"){
        detector_ = cv::AKAZE::create();
    }else if(detector_mode == "brisk"){
        detector_ = cv::BRISK::create();
    }else if(detector_mode == "kaze"){
        detector_ = cv::KAZE::create();
    }else{
        ROS_ERROR("Invalid detector mode");
        ros::shutdown();
    }
}

void IdentifyObjectTest::load_reference_images(std::string reference_images_path)
{
    // load reference images
    std::string file_name = reference_images_path + "save.txt";
    std::ifstream ifs(file_name);
    std::string line;
    while(std::getline(ifs,line)){
        std::vector<std::string> strvec = split(line,',');
        try{
            std::string rgb_name = static_cast<std::string>(strvec[0]);
            int object_id = static_cast<int>(std::stoi(strvec[1]));

            Images images(object_id);
            cv::Mat rgb_image;
            rgb_image = cv::imread(reference_images_path + rgb_name,1);
            if(rgb_image.empty()) break;
            Image rgb;
            calc_features(rgb,rgb_name,rgb_image);
            images.set_rgb_image(rgb);
            reference_images_.emplace_back(images);
        }
        catch(const std::invalid_argument& ex){
            ROS_ERROR("invalid: %s", ex.what());
        }
        catch(const std::out_of_range& ex){
            ROS_ERROR("out of range: %s", ex.what());
        }
    }
    ifs.close();
}

void IdentifyObjectTest::load_input_image(std::string input_image_path)
{
    input_image_ = cv::imread(input_image_path,1);
    if(input_image_.empty()){
        ROS_ERROR("input image is empty");
        ros::shutdown();
    }
}

void IdentifyObjectTest::calc_features(Image& image,std::string name,cv::Mat img)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detector_->detectAndCompute(img,cv::Mat(),keypoints,descriptors);
    image.set_params(name,img,keypoints,descriptors);
}

void IdentifyObjectTest::create_database(std::string reference_images_path)
{
    std::string image_mode = "rgb";

    for(const auto &ri : reference_images_) features_.emplace_back(ri.rgb.descriptor);

    vocabulary_ = new Vocabulary(VOCABULARY_K_,VOCABULARY_L_,TF_IDF,L1_NORM);
    vocabulary_->create(features_);
    database_ = new Database(*vocabulary_,false,0);
    for(const auto &f : features_) database_->add(f);
}

void IdentifyObjectTest::identify_object(cv::Mat input_image,int& object_id)
{
    std::cout << "identify_object start time: " << ros::Time::now() << std::endl;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detector_->detectAndCompute(input_image,cv::Mat(),keypoints,descriptors);

    QueryResults ret;
    database_->query(descriptors,ret,DATABASE_MAX_RESULTS_);
    std::cout << "identify_object end time: " << ros::Time::now() << std::endl;
    if(ret.empty()){
        object_id = -1;
        return;
    }
    // ret score
    for(const auto &r : ret)
    {
        std::cout << "id: " << r.id << ", score: " << r.score << std::endl;
    }
    object_id = reference_images_.at(ret[0].id).id;
}

std::vector<std::string> IdentifyObjectTest::split(std::string& input,char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while(std::getline(stream,field,delimiter)) result.emplace_back(field);
    return result;
}

void IdentifyObjectTest::process()
{
    ros::Rate loop_rate(HZ_);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "identify_object_test");
    IdentifyObjectTest identify_object_test;
    identify_object_test.process();
    return 0;
}