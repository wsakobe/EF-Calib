#include "circle_detector.h"

using namespace circle_detector;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    
    ros::init(argc, argv, "circle_detector");
    ros::NodeHandle nh;
    
    circle_detector::CircleDetector cd(nh);

    ros::spin();

    ros::shutdown();

    return 0;
}
