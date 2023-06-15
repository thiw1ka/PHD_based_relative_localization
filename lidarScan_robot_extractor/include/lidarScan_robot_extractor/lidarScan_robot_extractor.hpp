

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include <Eigen/Core>
#include <ros/console.h>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include <math.h>
#include <chrono>
#include <algorithm>
#include <numeric>
// #include "gaussian_comp.hpp"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point32.h"
#include "utility_functions/utility_functions.hpp"
#include "nav_msgs/Odometry.h"
#include "lidarScan_robot_extractor/blindspots_for_robots_in_lidar.hpp"

// #include "swarm_cherokeys/LidarMeas.h"

#ifndef _LIDARSCAN_ROBOT_EXTRACTOR_HPP
#define _LIDARSCAN_ROBOT_EXTRACTOR_HPP



#define RAD2DEG(x) ((x)*180./M_PI)
using namespace std;
using namespace Eigen;

class LidarObstacleExtraction{

	enum jumpSign {
		positive_jump,//0
		negative_jump,//1
		none//2
	};

	ros::NodeHandle _n;

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

	// void scanCallback_v2(const sensor_msgs::LaserScan::ConstPtr& scan);

	ros::Subscriber sub, subOdom;

	// std::vector<GaussianComponent> measurements,measurements_ready;
	// std::vector<geometry_msgs::Point32> measurements,measurements_ready;


	int counter;

	double RobValAngMin,RobValAngMax;
	double jumpLength;
	double Max_ScanRange;
	double Min_ScanRange;
	double Max_RobWidth,Max_ObstWidth;
	double Min__RobWidth;
	int Min_temp_vec_size;
	bool obj_at_back;

	geometry_msgs::Point32 check_data_valid_for_robot (const vector<pair<double,double >>& data_set,const double& ang_incr) ;

	// void its_a_obstacle (vector<pair<double,double >>& data_set, double ang_incr);

	sensor_msgs::LaserScan lastScan;

	jumpSign firstJumpSign; 

	// ros::Publisher  LMeas_talker;
	ros::Publisher  pub_locations, pub_after_blindspot;

	geometry_msgs::PoseArray Lm_publish;
	// swarm_cherokeys::LidarMeas msg;


	ros::Timer timer1;
	void ScI_callback(const ros::TimerEvent& event);//scan intensities callback
	bool newScan,IsNullMeasReady;
	vector<pair<double,double >> NullptsReady;

	void print_pair (vector<pair<double,double >>& points);

	std::unique_ptr <utility_functions::InsideBoxCal> computeInsideBoxPtr;

	utility_functions::OdomTranslaterToWorld odom_cb;

	//params from parameter server
	std::string lidarScanTopicName, robotOdomTopicName, extractionsWithBlindSPotPubTopicName, extractionsPubTopicName;
	bool isShowingAllPoints,IsSimulated; //to show all points in visualizer
	bool shouldWallMeasurmentsRemove; // false positives from wall be removed


	// void get_Nullpts(const sensor_msgs::LaserScan& scan);
	BlindSpotForRobotsInLidar* blindspot;

	Eigen::Matrix4d homogMatrix; //homogenious translator to map frame obtain from odom

	public:

		LidarObstacleExtraction (ros::NodeHandle n) : _n (n) {
			_n.param <bool> ("/simulation",IsSimulated,true); //is filter run in simulation?????
			_n.param <bool> ("/wallMeasurements",shouldWallMeasurmentsRemove,false); //local param. 
			_n.param <std::string> ("/lidarScanTopicName",lidarScanTopicName,"/Robot1/scan");
			_n.param <std::string> ("/robotOdomTopicName",robotOdomTopicName,"/Robot1/odom");
			_n.param <std::string> ("/extractionsPubTopicName",extractionsPubTopicName, "/lidar_extractions");
			_n.param <std::string> ("/extractionsWithBlindSPotPubTopicName",extractionsWithBlindSPotPubTopicName, "/lidar_extractions_blindspot");

			counter = 0;
			firstJumpSign = none;
			//RobValAngMin = 3.00;
			//RobValAngMax = 10.0;
			jumpLength = 0.1;
			Max_ScanRange = 5.0;
			Min_ScanRange = 0.1;
			Max_RobWidth = 0.08;
			Min__RobWidth = 0.001;
			Max_ObstWidth = 0.2;
			Min_temp_vec_size = 1; // points cluster size to check for robots
			obj_at_back = false;
			newScan = false;
			IsNullMeasReady = false;

			// odom_cb.reset(new utility_functions::OdomTranslaterToWorld ());
			// utility_functions::OdomTranslaterToWorld odom_cb;
			odom_cb;

			sub = _n.subscribe<sensor_msgs::LaserScan>(lidarScanTopicName, 1, &LidarObstacleExtraction::scanCallback, this);
			subOdom = _n.subscribe <nav_msgs::Odometry> (robotOdomTopicName, 1, &utility_functions::OdomTranslaterToWorld::odomCallback, &odom_cb);
			pub_locations = _n.advertise <sensor_msgs::PointCloud> (extractionsPubTopicName, 10);
			pub_after_blindspot = _n.advertise <sensor_msgs::PointCloud> (extractionsWithBlindSPotPubTopicName, 10);

			computeInsideBoxPtr.reset(new utility_functions::InsideBoxCal());
			double x =2.70, y=2.70;
			std::pair<double, double> points [] = {std::make_pair(x,y), 
                                                std::make_pair(x,-y), 
                                                std::make_pair(-x, -y), 
                                                std::make_pair(-x, y)
												};
												
			geometry_msgs::Point32 pt;
			int counter = 0;
			for (auto& p : points) {
				pt.x = p.first;
				pt.y = p.second;
				computeInsideBoxPtr->corners_of_rectangle[counter] = pt;
				counter++;
			}
			blindspot = new BlindSpotForRobotsInLidar(_n);
		}

		int getNullPts(std::vector<pair<double,double >> *NullPt){
			*NullPt=NullptsReady;
			return counter;
		}
};

void print (vector<pair<double,double >>& points);



#endif
