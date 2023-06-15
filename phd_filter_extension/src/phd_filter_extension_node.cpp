#include "phd_filter_extension/filter.hpp"

int main(int argc, char* argv[]) {

    ros::init( argc, argv,  "phd_filter_extension_node" );
    ros::NodeHandle nh;
    // string filtertype = "Standard";//"adaptive";
    std::cout<<"phd_filter_extension_node started.."<<std::endl;
    ros::AsyncSpinner spinner(0); // Use all threads
    spinner.start();
    std::unique_ptr<Filter> catabottrack (new Filter(nh));
    ros::waitForShutdown();


    return 0;
};