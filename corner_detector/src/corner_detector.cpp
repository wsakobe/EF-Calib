#include "corner_detector.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    corner_detector::ImageProcessor* ip = new corner_detector::ImageProcessor(nh);
    
    ros::spin();

    return 0;
}
