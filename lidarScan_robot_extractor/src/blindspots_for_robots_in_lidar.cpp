#include "lidarScan_robot_extractor/blindspots_for_robots_in_lidar.hpp"

BlindSpotForRobotsInLidar::BlindSpotForRobotsInLidar (ros::NodeHandle n) : n_(n) {
    std::printf("[BlindSpotForRobotsInLidar] Blindspot for lidar measurements node started..\n");
    n_.param <double> ("/radiusForBlindspotCircle", radiusForBlindspotCircle, 0.05);
    n_.param <std::string> ("/frameID", frameID, "map");
    n_.getParam("/pd_for_each_robot", pd_for_each_target);
    if (pd_for_each_target.empty()) pd_for_each_target = {1.0,1.0}; //to avoid other errors
    robots_list_.clear();
    sub_gazebo_ = n_.subscribe("/gazebo/model_states", 1, &BlindSpotForRobotsInLidar::CbForRobotsGroundTruth, this);
    pub_rviz_ = n_.advertise<visualization_msgs::MarkerArray>("Blindspots_Lidar_rviz_visulizer",2);
    T1 = n_.createTimer(ros::Duration(0.1), &BlindSpotForRobotsInLidar::publish_blindspots, this);
}

void BlindSpotForRobotsInLidar::CbForRobotsGroundTruth (const gazebo_msgs::ModelStates::ConstPtr& msg) {
//this callback read simulation model positions and keep it in a array
    std::lock_guard <std::recursive_mutex> lk(lock);
    // std::printf("[BlindSpotForRobotsInLidar] inside cb..\n");    
    double blindspot_radius = 0.05; // 5cm
    for (int i = 0; i < msg->name.size(); i++) {
        auto found = msg->name[i].find("Robot");    
        if(found != std::string::npos) {
            auto stringEnd = msg->name[i].size() - 1;//string starts from 0 not 1
            int robot_number = std::stoi(msg->name[i].substr(5, stringEnd));
            /*In the first msg create a list of robots with pd*/
            if (isFirstOdom) {
                //To ensure correct number of pd is provided in the vector
                auto pdFromList = robot_number < pd_for_each_target.size() ? pd_for_each_target[robot_number - 1] : 1.0; // robot number start from 1 where vector from 0
                auto output = robots_list_.insert_or_assign(robot_number,
                            std::unique_ptr<CircularBlindSpot>(new CircularBlindSpot(robot_number ,pdFromList, blindspot_radius)));
                std::printf("[BlindSpotForRobotsInLidar] new extracted robot number = %i, pd_assigned: %f, map_new_inserted: (%i) \n",
                                                                                robot_number, pdFromList, int(output.second));
            }
            //assign the robot position to the center
            geometry_msgs::Point32 pos;
            pos.x= msg ->pose[i].position.x;
            pos.y= msg ->pose[i].position.y;
            pos.z= msg ->pose[i].position.z;
            robots_list_[robot_number]->center_ = pos;
        }
    }
    isFirstOdom = false;
}

//static variables
bool BlindSpotForRobotsInLidar::isFirstOdom = true;

void BlindSpotForRobotsInLidar::checkPdForEachMeasurementFromLidar (sensor_msgs::PointCloud& measurement_list, 
                                                                        Eigen::Matrix4d homogeneous_matrix) {
    std::lock_guard <std::recursive_mutex> lk(lock);
    // auto newMeasurementList = measurement_list.points
    std::vector <int> list_of_robots_to_check;
    list_of_robots_to_check.reserve (robots_list_.size());
    for(auto i = 1; i < (robots_list_.size()+1); i++) list_of_robots_to_check.push_back(i); //robot list starts with 1

    std::vector<geometry_msgs::Point32> finalList;

    for(const auto& pt: measurement_list.points) {
        //translate each measurement into map frame
        Eigen::Vector4d pointInEigen (pt.x, pt.y, pt.z, 1);
        Eigen::Vector4d  translateToWorld = homogeneous_matrix * pointInEigen;
        auto convertedpt = pt;
        convertedpt.x = translateToWorld.x();
        convertedpt.y = translateToWorld.y();
        convertedpt.z = translateToWorld.z();
        int rob_indx = 0;
        for(rob_indx; rob_indx < list_of_robots_to_check.size(); rob_indx++) {
            auto isInside = robots_list_[list_of_robots_to_check[rob_indx]]->IsPointInside (convertedpt);
            if (isInside) {
                //if robot is found, check it's probability sample
                auto sample = robots_list_[rob_indx] ->pg_->get_sample();
                if(sample) finalList.push_back(pt); //push it inside the final list
                list_of_robots_to_check.erase(list_of_robots_to_check.begin()+rob_indx); //erase the current robot from list
                break;
            }
        }
    }
    if (!list_of_robots_to_check.empty()) {
        //obtain samples for other remaining robots to normalize with other robots
        for (auto robt : list_of_robots_to_check) {
            auto sample = robots_list_[robt] ->pg_->get_sample();
        }
    }
    measurement_list.points.clear();
    std::copy(finalList.begin(), finalList.end(), back_inserter(measurement_list.points)); 
}

void BlindSpotForRobotsInLidar::publish_blindspots (const ros::TimerEvent& e) {
    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker robot_blindspots, text;
    robot_blindspots.header.frame_id     = frameID;//"/surface_cam";
    // if (gazebo_sim_rosbag) robot_blindspots.header.frame_id = "Robot1/camera_link";
    robot_blindspots.header.stamp        = ros::Time::now();
    robot_blindspots.type                = visualization_msgs::Marker::CYLINDER;
    robot_blindspots.pose.orientation    = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    robot_blindspots.color.b             = 1;
    robot_blindspots.color.g             = 1;
    robot_blindspots.color.a             = 1;
    robot_blindspots.lifetime            = ros::Duration(0.5);
    robot_blindspots.ns                  = "blindspotCircle";
    robot_blindspots.scale.z             = 0.05;
    text = robot_blindspots;
    text.ns = "texts";
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    std::unique_lock <std::recursive_mutex> lk(lock);
    for(const auto& robot: robots_list_) {
        robot_blindspots.scale.x = robot.second->radius_;
        robot_blindspots.scale.y = robot_blindspots.scale.x;
        auto pos = robot.second ->center_;
        robot_blindspots.pose.position.x = pos.x;
        robot_blindspots.pose.position.y = pos.y;
        robot_blindspots.pose.position.z = 1.0;
        text.pose.position = robot_blindspots.pose.position;
        text.pose.position.z = 1.1;
        robot_blindspots.id = robot.first;
        text.id = robot_blindspots.id;
        text.text = std::string("ID: ") +std::to_string(robot.second->robot_id_) + 
                    std::string("PD: ") + std::to_string(robot.second->pd_);
        msg.markers.push_back(robot_blindspots);
        msg.markers.push_back(text);
    }
    lk.unlock();
    pub_rviz_.publish (msg);
}
