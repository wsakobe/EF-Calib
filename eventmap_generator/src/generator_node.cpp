#include "generator.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "eventmap_generator");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    generator::EventMap em(nh, nh_private);

    ros::spin();

    return 0;
}
