#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include "mutex"
#include <random>
#include "utility_functions/probability_generator.hpp"
#include "utility_functions/utility_functions.hpp"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

#ifndef BLINDSPOTS_FOR_ROBOTS_IN_LIDAR_HPP
#define BLINDSPOTS_FOR_ROBOTS_IN_LIDAR_HPP

/**
 * @brief this class act as an profile for a robot, contain necessary
 * details for a blindspot created as an circle on the robot's position.
 * blinspot activate's based on the given probability. seperate probability
 * generator class is implemented.
 */
struct CircularBlindSpot {

    CircularBlindSpot (int Id, double pd, double radius) : 
                            robot_id_(Id), radius_(radius), pd_(pd) {
        pg_ = new ProbabilityGenerator(pd);
        std::printf("[CircularBlindSpot] save New Robot: %i, pd: %f, BlindSpotRad: %f \n", 
                                                        Id, pd, radius);
    };
    double pd_; //probability for each robot

    int robot_id_; //robot id

    double radius_; //the radius of blindspot circle. make sure it is sufficient to capture lidar measurement

    geometry_msgs::Point32 center_; //center of blindspot circle

    bool IsPointInside (geometry_msgs::Point32 pt) {
        auto output = utility_functions::isPointInsideCircle <geometry_msgs::Point32>
                                                                    (center_, radius_, pt);
        return output;
    }

    ProbabilityGenerator* pg_;

    ~CircularBlindSpot () {
        delete pg_;
    };
};

/**
 * @brief this class creates blindspots on given robot locations based on a probability.
 * Class will go through lidar measurements (Robot extracted locations) and iterate through
 * availbel robots to find which robot is represented by this measurement. Once it is located 
 * class will generate a probability outcome to delete this particular measurement or to keep it.
 * 
 */
struct BlindSpotForRobotsInLidar {

    BlindSpotForRobotsInLidar (ros::NodeHandle n);

    ros::NodeHandle n_;

    ros::Subscriber sub_gazebo_;

    ros::Publisher pub_rviz_;

    void CbForRobotsGroundTruth (const gazebo_msgs::ModelStates::ConstPtr& msg);

    std::map <int, std::unique_ptr <CircularBlindSpot> > robots_list_;

    static bool isFirstOdom;

    double radiusForBlindspotCircle;

    std::vector<double> pd_for_each_target;

    std::string frameID;

    /**
     * @brief this function will go through all measurements and compare with robots list.
     * when robot is match, function will obtain the pd for that robot.
     * if pd is false the measuremet will be erased
     * 
     * @param measurement_list 
     */
    void checkPdForEachMeasurementFromLidar (sensor_msgs::PointCloud& measurement_list, 
                                            Eigen::Matrix4d homogeneous_matrix);

    void publish_blindspots (const ros::TimerEvent& e);

    std::recursive_mutex lock;

    /**
     * @brief timer for publisher. 
     * 
     */
    ros::Timer T1;
};

#endif 