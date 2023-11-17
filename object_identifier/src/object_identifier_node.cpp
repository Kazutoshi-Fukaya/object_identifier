#include "object_identifier/object_identifier.h"

using namespace object_identifier;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"object_identifier");
    ObjectIdentifier object_identifier;
    object_identifier.process();
    return 0;
}