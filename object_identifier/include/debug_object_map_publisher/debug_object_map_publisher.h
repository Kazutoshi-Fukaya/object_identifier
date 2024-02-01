#ifndef DEBUG_OBJECT_MAP_PUBLISHER_H_
#define DEBUG_OBJECT_MAP_PUBLISHER_H_

#include <ros/ros.h>

// custom msgs
#include "multi_localizer_msgs/ObjectMap.h"

namespace object_identifier
{
class DebugObjectMapPublisher
{
public:
    DebugObjectMapPublisher();
    ~DebugObjectMapPublisher();
    void process();

private:
    void load_object_map();    // yaml file
    void publish_object_map();

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // publisher
    ros::Publisher object_map_pub_;

    ros::Time start_time_;
    multi_localizer_msgs::ObjectMap object_map_;
    bool is_object_map_loaded_;

    // parameters
    int HZ_;
};
} // namespace object_identifier

#endif  // DEBUG_OBJECT_MAP_PUBLISHER_H_