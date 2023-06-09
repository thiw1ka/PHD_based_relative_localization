

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

// #include "swarm_cherokeys/LidarMeas.h"

#ifndef _LIDAR_DATA_PROC_HPP
#define _LIDAR_DATA_PROC_HPP



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

	ros::Subscriber sub, subOdom;

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

	sensor_msgs::LaserScan lastScan;

	jumpSign firstJumpSign; 

	// ros::Publisher  LMeas_talker;
	ros::Publisher  pub_locations;

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
	std::string lidarScanTopicName, robotOdomTopicName;
	bool isShowingAllPoints,IsSimulated; //to show all points in visualizer
	bool shouldWallMeasurmentsRemove; // false positives from wall be removed

	public:

		LidarObstacleExtraction (ros::NodeHandle n) : _n (n) {
			n.param <bool> ("/simulation",IsSimulated,true); //is filter run in simulation?????
			n.param <bool> ("wallMeasurements",shouldWallMeasurmentsRemove,false); //local param. 
			n.param <std::string> ("lidarScanTopicName",lidarScanTopicName,"/Robot1/scan");
			n.param <std::string> ("robotOdomTopicName",robotOdomTopicName,"/Robot1/odom");
			counter = 0;
			firstJumpSign = none;
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

			odom_cb;

			sub = _n.subscribe<sensor_msgs::LaserScan>(lidarScanTopicName, 1, &LidarObstacleExtraction::scanCallback, this);
			subOdom = _n.subscribe <nav_msgs::Odometry> (robotOdomTopicName, 1, &utility_functions::OdomTranslaterToWorld::odomCallback, &odom_cb);
			pub_locations = _n.advertise <sensor_msgs::PointCloud> ("/lidar_extractions", 10); 

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
		}

		int getNullPts(std::vector<pair<double,double >> *NullPt){
			*NullPt=NullptsReady;
			return counter;
		}


};

void print (vector<pair<double,double >>& points);



#endif
