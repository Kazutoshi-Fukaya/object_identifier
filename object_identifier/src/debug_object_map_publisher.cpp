#include "debug_object_map_publisher/debug_object_map_publisher.h"

using namespace object_identifier;

DebugObjectMapPublisher::DebugObjectMapPublisher()
    : private_nh_("~")
{
    // parameters
    private_nh_.param("HZ",HZ_,{1});

    // load object map
    // load_object_map();

    // publisher
    object_map_pub_ = nh_.advertise<multi_localizer_msgs::ObjectMap>("debug_object_map_out",1);

    // start time
    start_time_ = ros::Time::now();
    is_object_map_loaded_ = false;
}

DebugObjectMapPublisher::~DebugObjectMapPublisher(){}

void DebugObjectMapPublisher::load_object_map()
{
    // load object map
    std::string yaml_file_name;
    private_nh_.param("YAML_FILE_NAME",yaml_file_name,{std::string("debug_object_map")});
    XmlRpc::XmlRpcValue object_positions;
    if(!private_nh_.getParam(yaml_file_name.c_str(),object_positions)){
        ROS_ERROR("Failed to load object map yaml file");
        return;
    }

    // set object map
    ROS_ASSERT(object_positions.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i=0;i<object_positions.size();i++){
        if(!object_positions[i]["start_time"].valid() ||
           !object_positions[i]["end_time"].valid() ||
           !object_positions[i]["objects"].valid()){
            ROS_ERROR("Invalid object map yaml file");
            return;
        }
        if(object_positions[i]["start_time"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
           object_positions[i]["end_time"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
           object_positions[i]["objects"].getType() == XmlRpc::XmlRpcValue::TypeArray){
            // conpare time
            if((ros::Time::now() - start_time_).toSec() < static_cast<int>(object_positions[i]["start_time"]) ||
               (ros::Time::now() - start_time_).toSec() > static_cast<int>(object_positions[i]["end_time"])){
                continue;
            }
            multi_localizer_msgs::ObjectMap object_map;
            object_map.header.frame_id = "map";
            object_map.header.stamp = ros::Time::now();
            for(int j=0;j<object_positions[i]["objects"].size();j++){
                if(!object_positions[i]["objects"][j]["id"].valid() ||
                   !object_positions[i]["objects"][j]["name"].valid() ||
                   !object_positions[i]["objects"][j]["x"].valid() ||
                   !object_positions[i]["objects"][j]["y"].valid()){
                    ROS_ERROR("Invalid object map yaml file");
                    return;
                }
                if(object_positions[i]["objects"][j]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
                   object_positions[i]["objects"][j]["name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
                   object_positions[i]["objects"][j]["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
                   object_positions[i]["objects"][j]["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble){
                    multi_localizer_msgs::ObjectData object_data;
                    object_data.id = static_cast<int>(object_positions[i]["objects"][j]["id"]);
                    object_data.name = static_cast<std::string>(object_positions[i]["objects"][j]["name"]);
                    object_data.x = static_cast<double>(object_positions[i]["objects"][j]["x"]);
                    object_data.y = static_cast<double>(object_positions[i]["objects"][j]["y"]);
                    object_map.data.emplace_back(object_data);
                }
            }
            object_map_ = object_map;
            is_object_map_loaded_ = true;
            std::cout << "debug object map loaded" << std::endl;
            return;
        }
    }
}

void DebugObjectMapPublisher::publish_object_map()
{
    if(!is_object_map_loaded_) return;
    object_map_pub_.publish(object_map_);
}

void DebugObjectMapPublisher::process()
{
    ros::Rate loop_rate(HZ_);
    while(ros::ok()){
        load_object_map();
        publish_object_map();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"debug_object_map_publisher");
    DebugObjectMapPublisher debug_object_map_publisher;
    debug_object_map_publisher.process();
    return 0;
}