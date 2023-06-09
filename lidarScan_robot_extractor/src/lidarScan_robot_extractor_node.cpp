#include "lidarScan_robot_extractor/lidarScan_robot_extractor.hpp"

int main(int argc, char* argv[]) {
    ros::init( argc, argv,  "lidarscan_robot_extractor" );
    ros::NodeHandle nh;
    std::cout<<"[lidarRobExtractor] Lidar scan robot location extractor started.."<<std::endl;
    ros::AsyncSpinner spinner(0); // Use all threads
    spinner.start();
    std::unique_ptr<LidarObstacleExtraction> lidarExtractor (new LidarObstacleExtraction(nh));
    ros::waitForShutdown();

    return 0;
};