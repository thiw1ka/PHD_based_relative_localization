#include "phd_filter_extension/filter.hpp"

Filter::Filter(ros::NodeHandle n) : nh_(n) {
    ns = ros::this_node::getNamespace ();
    std::cout << ns <<" Catabot phd tracking is starting..." << std::endl;

    //local filter params
    nh_.param <std::string>  ("filter_setting", filter_setting, " ");
    nh_.param <double>      ("PD", PD, PD);
    nh_.param <double>      ("PS", PS, PD);
    nh_.param <int>         ("adaptivePSParam", APSParam, 5);

    //universal params here {"/" infont of name define global param}
    nh_.param <std::string> ("/measurement_topic_name", measurement_topic_name, "empty"); // "/"" infont of name define global param. 
    nh_.param <std::string> ("/odom_topic_name", odom_topic_name, "empty"); // "/"" infont of name define global param. 
    nh_.param <std::string> ("/camera_topic_name", camera_topic_name, "camera_invalid_topic_name"); // camera topic name
    nh_.param <int>         ("/MIN_MD_TO_MERGE", MIN_MD_TO_MERGE, 100); // md distance for merging
    nh_.param <int>         ("/filter_running_speed", filter_running_speed, 10); // filter speed
    nh_.param <double>      ("/minimum_weight", MINIMUM_WEIGHT, 0.001);
    nh_.param <int>         ("/pd_buffer_size", pd_buffer_length, 0);
    nh_.param <bool>        ("/camera_enabled", Filter::isCameraEnabled, false); //setting sensor pd correction
    nh_.param <double>      ("/camera_fov", camera_fov, 60.0); // camera fov
    nh_.param <double>      ("/cam_resol_x", cam_resol_x, 640.0); // camera fov
    nh_.param <double>      ("/cam_resol_y", cam_resol_y, 480.0); // camera fov
    nh_.param <double>      ("/camera_pd_inside_fov", EstimateContainer::cam_sensor_pd_, 0.99); 
    nh_.param <double>      ("/lidar_pd_inside_fov", EstimateContainer::lidar_sensor_pd_, 0.99);
    nh_.param <double>      ("/lidar_range", EstimateContainer::lidar_range_, 5.0);
    nh_.param <bool>        ("/gazebo_sim_rosbag", gazebo_sim_rosbag, false); //setting sensor pd correction
    nh_.param <double>      ("/birth_targets_cov", birth_target_cov, 5.0); // covariance for birth targets
    nh_.param <double>      ("/sensor_noise", sensor_noise, 0.001); //observation noise
    nh_.param <double>      ("/camera_noise_variance", camera_noise_variance, 3.0); //in degrees
    

    //overwrite blindspot pd from local variable inside filter group
    nh_.param <bool>        ("/sensor_pd", EstimateContainer::sensor_pd_correction, false); //setting sensor pd correction
    nh_.param <bool>        ("sensor_pd", EstimateContainer::sensor_pd_correction,EstimateContainer::sensor_pd_correction); //setting sensor pd correction
    std::printf(" %s - sensor_pd is enabled - (%i) \n",ns.c_str(), int(EstimateContainer::sensor_pd_correction));


    //Enable buffer for pd if launch file param is  > 0
    if (pd_buffer_length > 0) {
        EstimateContainer::is_buffer_true = true;
        EstimateContainer::buffer_size = pd_buffer_length;
        std::cout << "static variable size" << EstimateContainer::buffer_size<< std::endl;
    }
    
    std::cout << "measurment topic - " << measurement_topic_name << std::endl;

    std::cout << "filter setting read from launch file = " << filter_setting << std::endl;
    is_new_odom_available_ = false;
    is_measurement_available_ = false;
    is_new_camera_meas_avail_ =false;
    PROCESS_NOISE << 0.01, 0.0, 0.0, 0.01;
    previous_odom_copy_ = nav_msgs::Odometry ();
    previous_odom_copy_.pose.pose.orientation.w = 1.0;
    previous_odom_copy_.pose.pose.orientation.x = 0.0;
    previous_odom_copy_.pose.pose.orientation.y = 0.0;
    previous_odom_copy_.pose.pose.orientation.z = 0.0;
    /*function pointers */
    // FilterFunctionPtr timeUpdateFunction, measurementUpdateFunction;
    /****Filter types***/
    if (filter_setting == "Standard") {
        std::cout << "Filter settting is set to regular Pd and Ps" << std::endl;
        timeUpdateFunction          = &Filter::GetTimeUpdate;
        measurementUpdateFunction   = &Filter::GetMeasurementUpdate;
        MeasurementUpdateFunCamera  = &Filter::GetMeasurementUpdateCamera;
    }
    else if (filter_setting == "adaptive") {
        std::cout << "Filter setting is set to adaptive Pd and Ps" << std::endl;
        timeUpdateFunction          = &Filter::GetTimeUpdateUsingAdaptivePs;
        measurementUpdateFunction   = &Filter::GetMeasurementUpdateUsingAdaptivePd;
        MeasurementUpdateFunCamera  = &Filter::GetMeasurementUpdateUsingAdaptivePdCamera;
    }
    else if (filter_setting == "adaptive_regular_ps") {
        std::cout << "Filter setting is set to adaptive Pd and regular Ps" << std::endl;
        timeUpdateFunction          = &Filter::GetTimeUpdate;
        measurementUpdateFunction   = &Filter::GetMeasurementUpdateUsingAdaptivePd;
        MeasurementUpdateFunCamera  = &Filter::GetMeasurementUpdateUsingAdaptivePdCamera;
    }
    else if (filter_setting == "adaptive_low_pass") {
        std::cout << "Filter setting is set to adaptive_low_pass" << std::endl;
        timeUpdateFunction          = &Filter::GetTimeUpdateUsingAdaptivePs;
        measurementUpdateFunction   = &Filter::GetMeasurementUpdateLowPass;
    }
    else {
        std::cout << "[filter] filter type undetermined. check launch file param. exiting..." << std::endl;
        exit(0);
    }
    odom_ = nh_.subscribe(odom_topic_name, 1, &Filter::OdomCallBack, this);
    //gazebo_simulation_settings
    if (gazebo_sim_rosbag) {
        EstimateContainer::gazebo_sim_rosbag = true;
        homogeneous_tranform_matrix = Eigen::Matrix4d::Identity();
        subcriber_for_measurement_  = nh_.subscribe(measurement_topic_name, 10,
                                                    &Filter::CBForSyntheticMesurements , this);
        subcriber_for_measurement_ground_truth = nh_.subscribe("/gazebo/model_states", 10,
                                                    &Filter::CbForGazeboGroundTruth, this);
        if (isCameraEnabled) {
            ////isCameraNewMessageAvailable = false;
            EstimateContainer::isCameraEnabled = true;
            EstimateContainer::camera_fov_ = camera_fov;
            subcriber_for_camera_ = nh_.subscribe(camera_topic_name, 10, &Filter::CbForCameraMeasGazebo, this);
        }
    }
    //Mingi_catabot
    else if (measurement_topic_name == "/centroid_label_obstacle" || measurement_topic_name == "/centroid_label_obstacle_republished") {
        //These are for catabot rosbags
        EstimateContainer::isCatabotRosbagActive = true;
        subcriber_for_measurement_  = nh_.subscribe(measurement_topic_name, 1,
                                                    &Filter::CentroidLabelObstacleCB, this);
        if(EstimateContainer::sensor_pd_correction) {

        }
        if (isCameraEnabled) {
            ////isCameraNewMessageAvailable = false;
            EstimateContainer::isCameraEnabled = true;
            EstimateContainer::camera_fov_ = camera_fov;
            //subcriber_for_camera_ = nh_.subscribe(camera_topic_name, 1, &Filter::CbForCameraMeasurements, this);
        }
    }
    //synthetic_measurements
    else if (measurement_topic_name == "/synthetic_measurements") {
        //This are for synthetic rosbags
        EstimateContainer::isSyntheticMeasurementsProvided = true;
        subcriber_for_measurement_ = nh_.subscribe(measurement_topic_name, 1,
                                                &Filter::CBForSyntheticMesurements, this);
        subcriber_for_measurement_ground_truth = nh_.subscribe("/synthetic_measurements_vrpn", 1,
                                                                &Filter::CBForSyntheticMesurementsGroundTruth, this);// location without pd intererance                               
        if(EstimateContainer::sensor_pd_correction) {
            // bool waitforservice = ros::service::waitForService("/blindspots",ros::Duration(2.0));
            // clientForBlindspotRequest = nh_.serviceClient<catabot_phd_tracking::blindspotServiceMsg> ("/blindspots",false);
            sub_blindspot = nh_.subscribe("/blindspot_lines", 1, &Filter::CbForBlindSpots, this);
        }
    }
    else {
        std::cout << "Measurement topic is undetermined. exiting..." << std::endl;
        exit(0);
    }
    filter_results_     = nh_.advertise<sensor_msgs::PointCloud> (ns + "/filter_results", 100, true);
    f_results_for_plot_ = nh_.advertise<sensor_msgs::PointCloud> (ns + "/filter_results_for_plotter", 100, true);
    rviz_marker         = nh_.advertise<visualization_msgs::MarkerArray> (ns + "/weight_visulize", 100, true);
    filter_start = nh_.createTimer(ros::Duration(0.1), &Filter::executeFilter, this, true); //one shot timer
}

//init static variables here
bool EstimateContainer::is_buffer_true = false;
int EstimateContainer::buffer_size = 0;
bool EstimateContainer::sensor_pd_correction = false;
geometry_msgs::Point32 EstimateContainer::b_corners[3];
bool EstimateContainer::isCameraEnabled = false;
double EstimateContainer::camera_fov_ = 0.0;
double EstimateContainer::cam_sensor_pd_ = 0.0;
double EstimateContainer::lidar_sensor_pd_ = 0.0;
double EstimateContainer::lidar_range_ = 0.0;
bool EstimateContainer::isCameraMeasurementUpdateActive = false;
bool EstimateContainer::isLidarMeasurementUpdateActive = false;
bool EstimateContainer::isSyntheticMeasurementsProvided = false;
bool EstimateContainer::gazebo_sim_rosbag = false;
bool EstimateContainer::isTimeUpdateActive = false;
bool EstimateContainer::isCatabotRosbagActive = false;

int Filter::counter_for_measurements = 0;
int Filter::counter_for_odom = 0;
bool Filter::is_publishing_data = false;
Eigen::Matrix4d Filter::homogeneous_tranform_matrix = Eigen::Matrix4d::Identity();
bool Filter::is_odom_init = false;
// bool Filter::is_new_camera_meas_avail_ = false;

EstimateContainer::EstimateContainer() : 
                            estimate(uri_soft_base::Gaussian(2)),
                            weight(0.0),
                            undetected(0.8),
                            detected(0.2),
                            undetected_since_last_detected(0.0),
                            expected_undetections(0.0),
                            mslh_lp(0.0) {
    if(is_buffer_true) {
        int init_pd_list [] = {1, 1, 1, 0}; //initial pd for each estiamte. pd = 0.75
        for (auto i : init_pd_list) {
            /*each sensor has 2 buffers. one for estimate, one forsensor missed detection*/
            //Lidar
            buffer_for_pd_.push_back(i);
            buffer_for_lidar_missed_detection_.push_back(0.0);
            //CAMERA
            if (isCameraEnabled) {
                buffer_for_pd_camera_.push_back(i);
                buffer_for_camsensor_missed_detection_.push_back(0.0);
            }
        }
        // if (isCameraEnabled) {
        //     for (auto i : init_pd_list) {
        //     buffer_for_pd_camera_.push_back(i)
        //     buffer_for_camsensor_missed_detection_.clear(); //TODO check if buffer need inital 4 numbers like above

        //     }
        // }
    }
};

EstimateContainer::EstimateContainer(const EstimateContainer& original) : 
                            estimate(original.estimate),
                            weight(original.weight),
                            undetected(original.undetected),
                            detected(original.detected),
                            undetected_since_last_detected(original.undetected_since_last_detected),
                            mslh_lp(original.mslh_lp),
                            expected_undetections(original.expected_undetections)
                            // buffer_for_pd_ (original.buffer_for_pd_)
                            {
                                buffer_for_pd_ = original.buffer_for_pd_;
                                buffer_for_lidar_missed_detection_ = original.buffer_for_lidar_missed_detection_;
                                buffer_for_camsensor_missed_detection_ = original.buffer_for_camsensor_missed_detection_;
                                buffer_for_pd_camera_ = original.buffer_for_pd_camera_; //hits and misses for camera measurement update
                            };

void Filter::executeFilter (const ros::TimerEvent& e){
    ros::Rate r((double) filter_running_speed);
    filter_current_step_ = FilterSteps::Initializing;
    try {
        while ( ros::ok()) {
            if (filter_current_step_ == FilterSteps::Initializing) {
                // ros::Duration(0.01).sleep();//wait till other nodes are up
                //list_of_estimates__ptr_->clear();
                list_of_estimates_.clear();
                list_of_estimates_.reserve(20); // lets allocate for 20 estimates
                if (is_new_odom_available_ ==  false) continue; // wait till new
                if (!is_measurement_available_) continue;
                rot_prev_timestep_ = Eigen::Rotation2Dd::Identity();
                filter_current_step_ = FilterSteps::TimeUpdate;
                std::cout << "[Filter] filter initialization step completed." << std::endl;
            }
            else if (filter_current_step_ == FilterSteps::TimeUpdate) {
                auto timeNow = ros::Time::now();
                // std::cout << "[Filter] filter time update started.." << std::endl;
                if (!is_new_odom_available_) continue;
                EstimateContainer::isTimeUpdateActive = true;
                (this->*timeUpdateFunction) ();
                filter_current_step_ = FilterSteps::MeasurementUpdate;
                EstimateContainer::isTimeUpdateActive = false;

                auto duration = ros::Time::now() - timeNow;
                std::printf("[Execute]time taken for timeupdate %f seconds \n\n\n",duration.toSec());
            }
            else if (filter_current_step_ == FilterSteps::MeasurementUpdate) {
                //Lidar
                auto timeNow = ros::Time::now();
                EstimateContainer::isLidarMeasurementUpdateActive = true;
                /*get mutex to check the bool*/
                std::unique_lock <std::mutex> lk (cloud_lock_);
                auto isNewLidarMeasAvail = is_measurement_available_;
                lk.unlock();
                std::printf("[Execute] Lidar Measurements update check: is_measurement_available_(%i) \n", int(isNewLidarMeasAvail));
                if (isNewLidarMeasAvail) //continue; 
                    (this->*measurementUpdateFunction) ();
                EstimateContainer::isLidarMeasurementUpdateActive = false;
                auto duration = ros::Time::now() - timeNow;
                std::printf("[Execute] time taken for lidar mu %f seconds \n\n\n",duration.toSec());
                filter_current_step_ = FilterSteps::PublishData;

                //CAMERA UPDATE
                if (!isCameraEnabled) continue; 
                std::printf("[Execute] Camera update started \n");
                timeNow = ros::Time::now();
                EstimateContainer::isCameraMeasurementUpdateActive = true;
                
                /*get mutex to check the bool*/
                std::unique_lock <std::mutex> ulc (cam_angle_readerLock);
                auto isNewCamMeasAvail = Filter::is_new_camera_meas_avail_;
                ulc.unlock();

                std::printf("[Execute] camera Measurements update check: Filter::is_new_camera_meas_avail_(%i) \n", int(isNewCamMeasAvail));
                if(isNewCamMeasAvail)
                    (this->*MeasurementUpdateFunCamera)();
                EstimateContainer::isCameraMeasurementUpdateActive = false;
                std::printf("[Execute] Camera update ended.... \n");
                duration = ros::Time::now() - timeNow;
                std::printf("[Execute] time taken for camera mu %f seconds \n\n\n",duration.toSec());
            }
            else if (filter_current_step_ == FilterSteps::PublishData) {
                auto timeNow = ros::Time::now();

                is_publishing_data = true;
                publishData();
                is_publishing_data = false;

                filter_current_step_ = FilterSteps::TimeUpdate;
                auto duration = ros::Time::now() - timeNow;
                std::printf("[Execute]time taken for publish %f seconds \n\n\n",duration.toSec());
            }
            else{
                throw std::invalid_argument("Error in filter_current_step_ : ");
            }
            r.sleep();
        }
    }
    catch (std::exception& e) {
        std::cout << "[Filter] Error in  filter execution. step :"<< filter_current_step_<< std::endl;
        std::cout << "[Filter] standard exception: " << e.what() << std::endl;
        std::cout << "[Filter] exiting from filter....." << std::endl;
        exit(0);
    }
}

void EstimateContainer::resetSensorBuffer(std::string sensor_name) {
    std::printf("[resetSensorBuffer] sensor_name(%s), isCameraEnabled(%i) \n",sensor_name.c_str(), int(isCameraEnabled));
    int init_pd_list [] = {1, 1, 1, 0}; //initial pd for each estiamte. pd = 0.75
    if (sensor_name == "camera" && isCameraEnabled == true) {
        buffer_for_pd_camera_.clear();
        buffer_for_camsensor_missed_detection_.clear();
        for (auto i : init_pd_list) {
                buffer_for_pd_camera_.push_back(i);
                buffer_for_camsensor_missed_detection_.push_back(0.0);
        }
        utility_functions::printBuffer <int> (sensor_name +": h/m ", buffer_for_pd_camera_);
        utility_functions::printBuffer <double> (sensor_name + ": 1-pd ", buffer_for_camsensor_missed_detection_);
    }
    else if (sensor_name == "lidar"){
        buffer_for_pd_.clear();
        buffer_for_lidar_missed_detection_.clear();
        for (auto i : init_pd_list) {
            buffer_for_pd_.push_back(i);
            buffer_for_lidar_missed_detection_.push_back(0.0);
        }
        utility_functions::printBuffer <int> (sensor_name +": h/m ", buffer_for_pd_);
        utility_functions::printBuffer <double> (sensor_name + ": 1-pd ", buffer_for_lidar_missed_detection_);
    }
    else {
        throw std::invalid_argument("[resetSensorBuffer] Error Invalid sensor name...");
    }
}

double EstimateContainer::ComputePs(const int& APSParam) {
    std::printf("[computePS] ps calculation requested \n");
    double pd = ComputePd();//:ComputePdMSLH();
    //for ps - missed since last detected
    double expected_sensor_miss_detection = 0.0;
    // for(auto i: buffer_for_lidar_missed_detection_) expected_sensor_miss_detection += i;
    // std::printf(" sum of expected_sensor_miss_detection = %f \n", expected_sensor_miss_detection);
    expected_sensor_miss_detection = std::accumulate(buffer_for_lidar_missed_detection_.begin(), buffer_for_lidar_missed_detection_.end(), 0.0);
    // std::printf(" sum of expected_sensor_miss_detection = %f \n", expected_sensor_miss_detection);
    std::cout << "[ComputePs] expected sensor miss lidar " <<expected_sensor_miss_detection;
    if (isCameraEnabled) 
        expected_sensor_miss_detection += std::accumulate(buffer_for_camsensor_missed_detection_.begin(), buffer_for_camsensor_missed_detection_.end(), 0.0);
    std::cout << " , adding with_camera " <<expected_sensor_miss_detection <<std::endl;
    std::printf ("[ComputePs] undet_since_last_detected (%f) = usld(%f) - (int)std::round(expectedmiss)(%i)  \n",
        (undetected_since_last_detected - (int) std::round(expected_sensor_miss_detection)),undetected_since_last_detected, (int) std::round(expected_sensor_miss_detection) );
    undetected_since_last_detected = (undetected_since_last_detected - (int) std::round(expected_sensor_miss_detection));
    undetected_since_last_detected = (undetected_since_last_detected < 0.0 ? 0.0 : undetected_since_last_detected); //check negative
    std::printf("[ComputePs] undetected_since_last_detected (%f) = (undetected_since_last_detected < 0.0 ? 0.0  \n" , undetected_since_last_detected);
    double x = undetected_since_last_detected;
    double ps = (1 / (1.1 + x * pd / 2)) * (1 / (1 + exp(3 * (x - APSParam))));//5 for boat 20 synthetic
    std::printf("[ComputePs] ps(%f), undetected_since_last_detected(%f), pd(%f), expected_miss (%f) \n",ps,x, pd,expected_sensor_miss_detection);
    return ps;
}

double EstimateContainer::ComputePd() {
    double A = 1.0;//ComputeA();
    //use the counters if buffer is not used
    if(!is_buffer_true) {
        std::printf("[getPDFromBuffer] PD calculation From counters \n");
        return (this->detected * A / (this->detected * A + this->undetected));
    }
    /*Calculate pd from the buffer*/
    std::printf("[getPDFromBuffer] PD calculation From Buffer Requested \n");

    int detection_sum_lidar = 0, detection_sum_cam =0, buf_current_len_lidar = 0, buf_current_len_cam = 0; 
    int no_of_detection = buffer_for_pd_.size();
    double pd = 0.0;//(double) detection_sum_lidar / no_of_detection;
    double expected_sensor_miss_detection = 0.0;
    double missed_detections_corrected_lidar = 0.0, missed_detections_corrected_cam = 0.0, pd_lidar_buff =1.0, pd_cam_buff = 1.0;
    std::printf("[getPDFromBuffer] isSyntheticMeasurementsProvided(%i) or gazebo_sim_rosbag(%i), isCatabotRosbagActive(%i), isCameraEnabled(%i)\n", 
                                    int(isSyntheticMeasurementsProvided), int(gazebo_sim_rosbag), int(isCatabotRosbagActive), isCameraEnabled);
    //calculate lidar pd from buffers.
    //seperate variable is used for lidar pd
    if (isSyntheticMeasurementsProvided || gazebo_sim_rosbag || isCatabotRosbagActive) {
        buf_current_len_lidar = buffer_for_pd_.size();
        detection_sum_lidar = std::accumulate(buffer_for_pd_.begin(), buffer_for_pd_.end(), 0);
        int missed_detections = no_of_detection - detection_sum_lidar;
        if (sensor_pd_correction) 
            //do not calculate expected miss when sensor pd correction not enabled
            expected_sensor_miss_detection = std::accumulate(buffer_for_lidar_missed_detection_.begin(), buffer_for_lidar_missed_detection_.end(), 0.0);
        missed_detections_corrected_lidar = (double) missed_detections - expected_sensor_miss_detection; 
        missed_detections_corrected_lidar = missed_detections_corrected_lidar > 0 ? missed_detections_corrected_lidar : 0;//check negative values.
        pd_lidar_buff =  detection_sum_lidar / (detection_sum_lidar + missed_detections_corrected_lidar); //CAN GO 0/0 
        if (std::isnan(pd_lidar_buff)) {
            pd_lidar_buff = 0.0;
            std::printf("[getPDFromBuffer] Lidar PD is NAN!!! making pd_lidar = 0; \n");
        }
        utility_functions::printBuffer<int> ("L_h/m ", buffer_for_pd_);
        utility_functions::printBuffer<double> ("L_1-pd ", buffer_for_lidar_missed_detection_);
        std::cout << " lidr_hit/mis " << detection_sum_lidar << ", buff_sz "<< no_of_detection 
            << ", missed "<< missed_detections << ", Sensor_missed "<< expected_sensor_miss_detection 
            << ", mis_correc "<< missed_detections_corrected_lidar << ", pd_lidar "<< pd_lidar_buff <<std::endl;
    }
    else {
         throw std::invalid_argument("[getPDFromBuffer] Error No valid detection count state.");
    }
    //Calculate camera pd from buffers.
    if (isCameraEnabled) {
        expected_sensor_miss_detection = 0.0;
        no_of_detection = buffer_for_pd_camera_.size();
        detection_sum_cam = std::accumulate(buffer_for_pd_camera_.begin(),buffer_for_pd_camera_.end(), 0);
        missed_detections_corrected_cam = no_of_detection - detection_sum_cam;
        if (sensor_pd_correction) 
            //do not calculate expected miss when sensor pd correction not enabled
            expected_sensor_miss_detection = std::accumulate(buffer_for_camsensor_missed_detection_.begin(), buffer_for_camsensor_missed_detection_.end(), 0.0);
        missed_detections_corrected_cam = (double) missed_detections_corrected_cam - expected_sensor_miss_detection; 
        missed_detections_corrected_cam = missed_detections_corrected_cam > 0 ? missed_detections_corrected_cam : 0;//check negative values.
        pd_cam_buff =  detection_sum_cam / (detection_sum_cam + missed_detections_corrected_cam); //CAN GO 0/0
        if (std::isnan(pd_cam_buff)) {
            pd_cam_buff = 0.0;
            std::printf("[getPDFromBuffer] Camera PD is NAN!!! making pd_cam = 0; \n");
        }
        utility_functions::printBuffer<int> ("cam_h/m", buffer_for_pd_camera_);
        utility_functions::printBuffer<double> ("cam_1_pd", buffer_for_camsensor_missed_detection_);
        std::cout << " cam_hit/mis " << detection_sum_cam << ", buff_sz "<< no_of_detection 
            << ", missed "<< missed_detections_corrected_cam << ", Sensor_missed "<< expected_sensor_miss_detection 
            << ", mis_correc "<< missed_detections_corrected_cam << ", pd_cam "<< pd_cam_buff <<std::endl;
    }

    //return if sensor expected misses are not enabled.
    //sum both sensor values and calculate overall PD
    //Meaning only one buffer(aka estimate's) is used.
    if (!sensor_pd_correction) {
        auto total_detection = detection_sum_lidar + detection_sum_cam;
        auto total_misses = missed_detections_corrected_cam + missed_detections_corrected_lidar;
        pd = total_detection / (total_misses + total_detection); 
        // std::printf("[getPDFromBuffer]  sensor_pd_correction off(%i) \n", int(!sensor_pd_correction));
        // pd = pd_lidar_buff * pd_cam_buff;
        std::printf("[getPDFromBuffer] sensor_pd_correction off(%i), Pd(only using buff)(%f) = total_detect(%i) / total_miss(%f) +  total_detect(), t_detect (detection_sum_lidar(%i) + detection_sum_cam(%i)), t_misses( missed_detections_corrected_cam(%f) + missed_detections_corrected_lidar(%f)) \n", 
                                int(!sensor_pd_correction), pd, total_detection, total_misses, detection_sum_lidar, detection_sum_cam, missed_detections_corrected_cam, missed_detections_corrected_lidar);
        return pd; //blindspot is not active return pd value

    }

    /*
    Calculate sensor's probability of detection for an estimate. e.g, estimate inside fov hence sensor can detect it otherwise cant.
    if cant be detected then pd should not be reduced just keep the current pd.
    This is mainly used in 1-pd update to keep the component with original pd since it will not be detected from MU
    */
    Eigen::Vector2d m = estimate.getMean();
    geometry_msgs::Point32 pt;
    pt.x = m[0];
    pt.y = m[1];
    double one_minus_sensor_pd = 0.0; // if not in the blindspot pd = 1 hence 1- pd = 0.0
    double blindspot_pd = 0.001, pd_of_lidar = 0.099, pd_of_camera = 0.099;
    bool isPointInsideBlindspotTriangleSynthetic = false, isLidarPointInsideGazebo = false, isPointInsideCameraFov =false;
    if (EstimateContainer::isSyntheticMeasurementsProvided /*&& EstimateContainer::isLidarMeasurementUpdateActive*/) {
        isPointInsideBlindspotTriangleSynthetic = utility_functions::isPointInsideBlindSpot(pt, b_corners);
        pd_of_lidar = (isPointInsideBlindspotTriangleSynthetic == true ? blindspot_pd : lidar_sensor_pd_); // if inside pd should be zero
        std::printf("[getPDFromBuffer] isSyntheticMeasurementsProvided(%i), lidar blindspot pd :%f, x:%f, y:%f, pt_inside:%i \n",
                            int(EstimateContainer::isSyntheticMeasurementsProvided), pd_of_lidar, pt.x, pt.y, isPointInsideBlindspotTriangleSynthetic);
    }
    else if (EstimateContainer::gazebo_sim_rosbag || isCatabotRosbagActive ) {
        //only lidar range is being used. check if the point is inside the range.
        isLidarPointInsideGazebo = utility_functions::isPointInsideLidarRange(lidar_range_, pt); //is point inside the range
        //if point is inside then it is detectable hence pd of lidar = 1 otherwise 0
        pd_of_lidar = isLidarPointInsideGazebo == true ? lidar_sensor_pd_ : 0.001;
        std::printf("[getPDFromBuffer] gazebo_sim_rosbag(%i), lidar blindspot pd :%f, x:%f, y:%f, pt_inside:%i \n",
                            int(EstimateContainer::gazebo_sim_rosbag), pd_of_lidar, pt.x, pt.y, isLidarPointInsideGazebo);
    }
    if (EstimateContainer::isCameraEnabled && !EstimateContainer::isLidarMeasurementUpdateActive){
        isPointInsideCameraFov = utility_functions::isPointInsideCameraFov(camera_fov_, pt);
        pd_of_camera = (isPointInsideCameraFov == true ? cam_sensor_pd_ : blindspot_pd); //if inside pd should be camera pd.
        std::printf("[getPDFromBuffer] camera blindspot(%i), cam blindspot pd :%f, x:%f, y:%f, pt_inside:%i \n",
                            int(EstimateContainer::isCameraEnabled), pd_of_camera, pt.x, pt.y, int(isPointInsideCameraFov));
    }

    //return according to filter status
    if(EstimateContainer::isTimeUpdateActive || Filter::is_publishing_data) {
        auto hits_from_all_sums = detection_sum_cam + detection_sum_lidar;
        auto missed_correction_from_all_sum = missed_detections_corrected_cam + missed_detections_corrected_lidar;
        pd = hits_from_all_sums / (hits_from_all_sums + missed_correction_from_all_sum);
        std::printf("[getPDFromBuffer] isTimeUpdateActive(%i), is_publishing_data(%i): Pd(final)(%f) =  hits_from_all_sums(%i) / hits_from_all_sums +  missed_correction_from_all_sum(%f)\n",
                            int(isTimeUpdateActive), int(Filter::is_publishing_data), pd, hits_from_all_sums, missed_correction_from_all_sum);  
        return pd;
    }
    else if(EstimateContainer::isLidarMeasurementUpdateActive) {
        if(pd_lidar_buff == 0) resetSensorBuffer("lidar");
        pd = pd_lidar_buff * pd_of_lidar;
        std::printf("[getPDFromBuffer] Pd(final)(%f) = pd_lidar(%f) * blindspot (%f), isLidarMeasurementUpdateActive(%i) \n", 
                            pd, pd_lidar_buff, pd_of_lidar, int(EstimateContainer::isLidarMeasurementUpdateActive));   
        return pd;
    }
    else if (EstimateContainer::isCameraMeasurementUpdateActive) {
        if(pd_cam_buff == 0) resetSensorBuffer("camera");
        pd = pd_cam_buff * pd_of_camera;
        std::printf("[getPDFromBuffer] Pd(final)(%f) = pd_camera(%f) * blindspot (%f), isCameraMeasurementUpdateActive(%i) \n",
                             pd, pd_cam_buff, pd_of_camera, int(EstimateContainer::isCameraMeasurementUpdateActive));   
        return pd;
    }
    else {
        throw std::invalid_argument("[getPDFromBuffer] Error invalid pd request state.");
    }
}

//request sensor pd
double EstimateContainer::getCalculatedPDFromBuffers (std::string sensor_name) {

        std::deque<int>* sensor_hits_buf;
        std::deque<double>* sensor_expec_misses_buf;

        if(sensor_name == "lidar") {
            sensor_hits_buf = &buffer_for_pd_;
            sensor_expec_misses_buf = &buffer_for_lidar_missed_detection_;
        }
        else if (sensor_name == "camera") {
            sensor_hits_buf = &buffer_for_pd_camera_;
            sensor_expec_misses_buf = &buffer_for_camsensor_missed_detection_;
        }
        else if (sensor_name == "both") {

        }
        else {
            throw std::invalid_argument("[getCalculatedPDFromBuffers] Error Invalid sensor name...");
        }

        auto buffer_sz = sensor_hits_buf->size();
        auto detection_sum = std::accumulate(sensor_hits_buf->begin(),sensor_hits_buf->end(), 0);
        auto missed_detections = buffer_sz - detection_sum;
        auto expected_sensor_miss_detection = std::accumulate(sensor_expec_misses_buf->begin(), sensor_expec_misses_buf->end(), 0.0);
        auto missed_detections_corrected = (double) missed_detections - expected_sensor_miss_detection; 
        missed_detections_corrected = missed_detections_corrected > 0.0 ? missed_detections_corrected : 0.0;//check negative values.
        auto pd =  detection_sum / (detection_sum + missed_detections_corrected); //CAN GO 0/0
        if (std::isnan(pd)) {
            pd = 0.0;
            std::printf("[getPDFromBuffer] Camera PD is NAN!!! making pd_cam = 0; \n");
        }
        // utility_functions::printBuffer<int> ("cam_h/m", buffer_for_pd_camera_);
        // utility_functions::printBuffer<double> ("cam_1_pd", buffer_for_camsensor_missed_detection_);
        std::cout << "[getCalculatedPDFromBuffers] "+sensor_name <<" hits:" << detection_sum << ", buff_sz "<< buffer_sz 
            << ", missed "<< missed_detections << ", Sensor_missed "<< expected_sensor_miss_detection 
            << ", mis_correc "<< missed_detections_corrected << ", pd_cam "<< pd <<std::endl;
        return pd;
    
}

void EstimateContainer::IncreaseDetected() {
    this->detected += 1;
    this->undetected_since_last_detected = 0;
    if(is_buffer_true) {
        std::printf("[IncreaseDetected] AddDetectionBuffer(1) \n");
        AddDetectionBuffer(true);
    }
}

void EstimateContainer::IncreaseUndetected() {
    this->undetected += 1;
    this->undetected_since_last_detected += 1;
    if(is_buffer_true) {
        std::printf("[IncreaseUndetected] AddDetectionBuffer(0) \n");
        AddDetectionBuffer(false);
    }
}

void EstimateContainer::AddDetectionBuffer(int i) {
    if (EstimateContainer::isLidarMeasurementUpdateActive) {
        std::printf("[AddDetectionBuffer] isLidarMeasurementUpdateActive (%i) then buffer_for_pd_.push_back(%i) \n",
                                            int(EstimateContainer::isLidarMeasurementUpdateActive), i);
        buffer_for_pd_.push_back(i);
        if (buffer_for_pd_.size() > buffer_size)  buffer_for_pd_.pop_front();
    }
    else if (EstimateContainer::isCameraMeasurementUpdateActive) {
        // utility_functions::printBuffer<int> ("cam_h/m", buffer_for_pd_camera_);
        std::printf("[AddDetectionBuffer] isCameraMeasurementUpdateActive (%i) then buffer_for_pd_camera_.push_back(%i) \n",
                                            int(EstimateContainer::isCameraMeasurementUpdateActive), i);
        buffer_for_pd_camera_.push_back(i);
        if (buffer_for_pd_camera_.size() > buffer_size) buffer_for_pd_camera_.pop_front();
        // utility_functions::printBuffer<int> ("cam_h/m", buffer_for_pd_camera_);
    }
    else {
        throw std::invalid_argument("[AddDetectionBuffer] invalid measurement state.\n");
    }
    // utility_functions::printBuffer<int> (buffer_for_pd_);
    std::printf("if (sensor_pd_correction(%i)) UpdateSensorPDBuffer() \n", int(sensor_pd_correction));
    if (sensor_pd_correction) UpdateSensorPDBuffer();
}

void EstimateContainer::UpdateSensorPDBuffer() {
    std::printf("[UpdateSensorPDBuffer] EstimateContainer::isLidarMeasurementUpdateActive(%i), isCameraEnabled(%i) && EstimateContainer::isCameraMeasurementUpdateActive (%i)\n",
            int(EstimateContainer::isLidarMeasurementUpdateActive), int(isCameraEnabled), int(EstimateContainer::isCameraMeasurementUpdateActive));

    Eigen::Vector2d m = estimate.getMean();
    geometry_msgs::Point32 pt;
    pt.x = m[0];
    pt.y = m[1];
    double one_minus_sensor_pd = 0.0; // if not in the blindspot pd = 1 hence 1- pd = 0.0
    if (EstimateContainer::isLidarMeasurementUpdateActive) {
        /*BLIND SPOT TRIANGLE*/
        if(isSyntheticMeasurementsProvided) {
            bool isPointInside = utility_functions::isPointInsideBlindSpot(pt, b_corners);
            if (isPointInside) {
                std::printf("[UpdateSensorPDBuffer] The point is NOT_DETECTED------ \n");
                one_minus_sensor_pd = 1.0; //if inside the blingspot sensor pd = 1,  1 - sensorpd = 1
            }
            buffer_for_lidar_missed_detection_.push_back(one_minus_sensor_pd);// just add zero for now
            if (buffer_for_lidar_missed_detection_.size() > buffer_size) buffer_for_lidar_missed_detection_.pop_front();
            std::printf("[UpdateSensorPDBuffer] isSyntheticMeasurementsProvided(%i), isPointInside(%i), buffer_for_lidar_missed_detection_.push_back(one_minus_sensor_pd(%f))\n", 
                            int(isSyntheticMeasurementsProvided), int(isPointInside), one_minus_sensor_pd);
        }

        /*GAZEBO SIMULATION*/
        else if (gazebo_sim_rosbag || isCatabotRosbagActive) {
            auto lidar_field_of_view = 360.0; // degrees
            //Lidar FOV not used. Only the range is used
            auto isinside = utility_functions::isPointInsideLidarRange(lidar_range_, pt);
            if (!isinside) one_minus_sensor_pd = 0.99; // cannot detect the estimate pd = 0.0 
            buffer_for_lidar_missed_detection_.push_back(one_minus_sensor_pd);// just add zero for now
            if (buffer_for_lidar_missed_detection_.size() > buffer_size) buffer_for_lidar_missed_detection_.pop_front();
            std::printf("[UpdateSensorPDBuffer] gazebo_sim_rosbag(%i), isCatabotRosbagActive(%i), isinside(%i), buffer_for_lidar_missed_detection_.push_back(one_minus_sensor_pd(%f))\n", 
                int(gazebo_sim_rosbag), int(isCatabotRosbagActive), int(isinside), one_minus_sensor_pd);
        }

    }
    /*CAMERA PD*/
    else if (isCameraEnabled && EstimateContainer::isCameraMeasurementUpdateActive) {
        one_minus_sensor_pd = 1.0; //(1-pd)
        bool isPointDetectable = utility_functions::isPointInsideCameraFov(camera_fov_, pt);
        if (isPointDetectable) {
            one_minus_sensor_pd = 1.0 - cam_sensor_pd_;
        }
        buffer_for_camsensor_missed_detection_.push_back(one_minus_sensor_pd);// just add zero for now
        if (buffer_for_camsensor_missed_detection_.size() > buffer_size) buffer_for_camsensor_missed_detection_.pop_front();
        std::printf("[UpdateSensorPDBuffer] isCameraEnabled(%i), isPointDetectable(%i), buffer_for_camsensor_missed_detection_.push_back(one_minus_sensor_pd(%f)) \n", 
        int(isCameraEnabled), int(isPointDetectable), one_minus_sensor_pd);
    }

    else{
        throw std::invalid_argument("[UpdateSensorPDBuffer] invalid update state.\n");
    }
}

void EstimateContainer::IncreaseUndetectedLP() {
    this->undetected = 1; 
    this->detected = 1;
    this->undetected_since_last_detected += 1;
}

double EstimateContainer::ComputeA() {
    double pd = 0.9;
    double B = 1.0;
    double Mk = this ->undetected;
    double Hk = this ->detected;
    double Mk_sq = std::pow(Mk, 2);
    double Hk_sq = std::pow(Hk, 2);
    double Wk = this ->weight;
    double A = pd * (Mk_sq * (1 + Wk) + Mk * (Wk + 2 * pd) + Hk * Mk * (1 + Wk) * (pd - 1)) /
                (Hk_sq - pd * Hk * (Mk + 3 * pd));
    std::cout << " A = " << A << std::endl; 
    return A;
}

double EstimateContainer::ComputePdMSLH() {
    double alpha = 0.9;
    mslh_lp = alpha * mslh_lp + (1 - alpha) * undetected_since_last_detected;
    if (mslh_lp == 0) return 0.5;
    double pd = 1 / mslh_lp;
    if (pd > 0.95) pd = 0.95;
    return pd;
}

void EstimateContainer::print () {
    auto location = estimate.getMean();
    std::printf ("[EstCont] x: %f, y: %f, w:%f \n", location[0], location[1], weight);
    std::cout <<"Cov = \n" << estimate.getCovariance() << std::endl;
}

//Callbacks
void Filter::CbForGazeboGroundTruth (const gazebo_msgs::ModelStates::ConstPtr& msg) {
    //this callback read simulation model positions and keep it in a array
    sensor_msgs::PointCloud converted_msg;
    
    for (int i =0; i < msg->name.size(); i++) {
        // auto ii = i.position;
        auto found = msg->name[i].find("Robot");
        // std::cout << "[cbGazeboGTruth] name: "<<msg->name[i];
        if(found != std::string::npos) {
            if (msg->name[i] == "Robot1") continue; //CHECK HERE.REMOVE ROBOT 1*******
            // std::cout << "\n found "<< std::endl;
            geometry_msgs::Point32 pos;
            pos.x= msg ->pose[i].position.x;
            pos.y= msg ->pose[i].position.y;
            pos.z= msg ->pose[i].position.z;
            converted_msg.points.push_back(pos);
            // std::cout << "[cbGazeboGTruth] name: "<<msg->name[i] << std::endl;
        } 
    }
    std::lock_guard <std::mutex> lg(cloud_syntheticGT_lock);
    cloud_syntheticGT_ = converted_msg;
}

void Filter::CbForCameraMeasGazebo (const geometry_msgs::PoseArray::ConstPtr& msg) {
    std::lock_guard <std::mutex> ul (cam_angle_readerLock);
    camera_measurements_.points.clear();
    if (msg->poses.empty()) return;
    Filter::is_new_camera_meas_avail_ = true;
    /*calculating camera matrix and rotation to robot frame*/
    // geometry_msgs::Point p;
    auto fov_half = camera_fov / 2;
    static double focal_length_px = (cam_resol_x / 2) / std::tan(fov_half * M_PI / 180); //Assuming fov is 60. 60/2 = 30
    // p.z = focal_length_px;//557.0; focal lenth is in Z direction.
    // std::printf("focal length in pixel %f should be 557 \n", p.z);
    Eigen::Matrix3d rot_CamtoRob;
    //anti clockwise is positive in rotation angle
	rot_CamtoRob = Eigen::AngleAxisd(-90 * M_PI / 180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(90 * M_PI / 180, Eigen::Vector3d::UnitY());
    Eigen::Vector3d point;
    point[2] = focal_length_px; //Z direction
    // std::cout << "[CbForCameraMeasGazebo] focal len " << focal_length_px << ", Rot = X(90)*Y(90) \n" << rot_CamtoRob <<std::endl;

    // cam_angle_readerLock.lock();
    // camera_angles_deg.clear();
    // camera_measurements_.points.clear();

    camera_measurements_.header.stamp = ros::Time::now();

    for (auto i : msg->poses) {
        // std::printf("bounding box x %f y %f confident %f \n", i.bbox.center.x, i.bbox.center.y, i.results[0].score);
        // double camx = i.position.x - (cam_resol_x / 2);//320.0;
        // double camy = (cam_resol_y / 2) - i.position.y; //image px start from left upper corner
        // p.y = 240.0 - i.position.y;
        // p.x = i.position.x - 320.0;

        // double FocalPix = 557;
        point[0] = i.position.x - (cam_resol_x / 2);//320.0;
        point[1] = (cam_resol_y / 2) - i.position.y; //image px start from left upper corner
        Eigen::Vector3d rotatedPoint = rot_CamtoRob * point;
        geometry_msgs::Point32 pt;
        pt.x = rotatedPoint[0]; pt.y = rotatedPoint[1]; pt.z = rotatedPoint[2];
        camera_measurements_.points.push_back(pt);
        // std::printf("[CbForCameraMeasGazebo] pix_x,y(%f, %f),transfromedCenter{x(%f),y(%f)} tranformed_pt{x(%f),y(%f),z(%f)}  \n",
        // i.position.x, i.position.y, i.position.x - (cam_resol_x / 2), (cam_resol_y / 2) - i.position.y, pt.x, pt.y, pt.z);
        // double bearing = utility_functions::getBearingForCameraTargets <double> (camx, FocalPix);
        // double bearing = getBearingForCameraTargets(camx, camy) * 180 / M_PI;
        // camera_angles_deg.push_back(bearing);
        // std::printf("bearing angle %f , angle from rotation %f\n", bearing, rotatedAngleCheck);
    }
}

void Filter::CbForBlindSpots (const visualization_msgs::Marker::ConstPtr& msg){
    for (int i = 0; i < 3; i++) {
        EstimateContainer::b_corners[i].x = msg->points[i].x;
        EstimateContainer::b_corners[i].y = msg->points[i].y;
    }
}

void Filter::OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard <std::mutex> lk (odom_now_lock_);
    odom_now_ = *msg;
    is_new_odom_available_ = true;
    ++counter_for_odom;
}

void Filter::InitStep() {}

void Filter::CentroidLabelObstacleCB (const sensor_msgs::PointCloud2::ConstPtr& msg) {
    /*
        obtaining locations of objects detectedby mingi's node.
        Converting from cloud2 to cloud format.
    */
    if (msg->data.empty()) return;
    std::lock_guard <std::mutex> lk (cloud_lock_);
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud_);
    if (list_of_estimates_.empty()){
        //if no estimates present then add raw measurements to the container
        std::cout<<"[centroidCB] estimates are empty. adding meas into estimates.1 ";
        for (auto& pt : cloud_.points) {
            //printf( "x : %f, y : %f, z : %f \n ", pt.x, pt.y, pt.z );
            EstimateContainer temp_estimate;
            temp_estimate.estimate.setMean(Eigen::Vector2d(pt.x, pt.y));
            temp_estimate.estimate.setCovariance(Eigen::Matrix2d::Identity());
            temp_estimate.weight = 1.0;
            list_of_estimates_.push_back(temp_estimate);
        }
        std::cout<< "2. Measurement added" << std::endl;
    }
    ++counter_for_measurements;
    is_measurement_available_ = true;
}

void Filter::CBForSyntheticMesurements (const sensor_msgs::PointCloud::ConstPtr& msg){
    /*
        obtaining locations of objects detectedby mingi's node.
        Converting from cloud2 to cloud format.
    */
   
    std::lock_guard <std::mutex> lk (cloud_lock_);
    // cloud_lock_.lock();
    cloud_ = *msg;
    if (list_of_estimates_.empty()) {
        //if no estimates present then add raw measurements to the container
        std::cout<<"[synthetic] estimates are empty. adding meas into estimates"<<std::endl;
        for (auto& pt : msg->points) {
            //printf( "x : %f, y : %f, z : %f \n ", pt.x, pt.y, pt.z );
            EstimateContainer temp_estimate;
            temp_estimate.estimate.setMean(Eigen::Vector2d(pt.x, pt.y)); // z is x axis in optitrac & x is y, rotx(90)
            temp_estimate.estimate.setCovariance(Eigen::Matrix2d::Identity() * 0.2);
            temp_estimate.weight = 1.0;
            list_of_estimates_.push_back(temp_estimate);
        }
    }
    ++counter_for_measurements;
    // cloud_lock_.unlock();
    // requestBlindspotDetails(); // service to get blindspots
    is_measurement_available_ = true;
}

void Filter::CBForSyntheticMesurementsGroundTruth (const sensor_msgs::PointCloud::ConstPtr& msg){
    /*
        obtatining ground truth msg of each synthetic msg
    */
    std::lock_guard <std::mutex> lg(cloud_syntheticGT_lock);
    cloud_syntheticGT_ = *msg;
}

double Filter::getPDensity(Eigen::Vector2d z, Eigen::Vector2d m, Eigen::Matrix2d& cov) const {
	Eigen::Vector2d error;
	//cout<<"z & nk-1 ="<<z<<endl<<nKminus1toK<<endl;
	error = z - m;
	//cout<<"error ="<<error<<endl;
	double sum = -0.5 * error.transpose() * cov.inverse() * error;
	double sum2 = sqrt(4 * M_PI * M_PI * abs(cov.determinant())); //change the 2pi values
	double temp = exp(sum) / sum2;

	//cout<<"pdensity = "<<temp<<", sum1 , sum2 "<<sum<<" , "<<sum2<<endl;
    // if (temp > 1.0) {
    //     std::cout << "[getPDensity] value is greater than 1!!!!.. I am making it equals to one \n";
    //     std::cout << "[getPDensity] location \n" << m << std::endl;
    //     std::cout << "[getPDensity] measurement \n" << z << std::endl;    
    //     std::cout << "[getPDensity] error\n" <<  error <<std::endl;
    //     std::cout << "[getPDensity] cov \n" << cov << std::endl;    
    //     std::cout << "[getPDensity] cov.inverse \n" <<cov.inverse() <<std::endl;
    //     std::cout << "[getPDensity] abs(cov.determinant()) "<<abs(cov.determinant()) << std::endl;
    //     std::cout << "[getPDensity] -0.5 * error.transpose() * cov.inverse() * error = "<< sum << std::endl;        
    //     std::printf("[getPDensity] pdensity(%f) = exp(-0.5 * error.transpose() * cov.inverse() * error)(%f) / sqrt(4 * M_PI * M_PI * abs(cov.determinant()))(%f) \n",
    //     temp, exp(sum), sum2);
    // }
    // if (temp > 1.0) temp = 1.0;
	return temp;
}

double Filter::GetMahalanobisDistance(uri_soft_base::Gaussian& A, uri_soft_base::Gaussian& B) const {
    /*  The Mahalanobis distance is a measure of the distance between a point P and a distribution D, 
        introduced by P. C. Mahalanobis in 1936.
        [1] It is a multi-dimensional generalization of the idea of measuring 
        how many standard deviations away P is from the mean of D
    */
	// Eigen::Vector2d mean_error;
	double MD;
	Eigen::Vector2d  mean_error (A.getMean() - B.getMean());
    //cout<< "mean A,B"<<A.mean<<" , "<< B.mean<<", error = "<<mean_error<<endl;
    //cout<< "covariance"<<A.covariance<<" , "<< B.covariance<<", Inv_VarCov = "<<Inv_VarCov<<endl;
	MD = mean_error.transpose() * A.getCovariance().inverse() * mean_error;//refer table 2 page 4097
	// std::printf("MD distance %f", sqrt(MD));
    return sqrt(MD);
}

void Filter::mergedNeareastComponents (std::vector <EstimateContainer>& list_of_estimates) {
    std::printf("[mergedNeareastComponents] Start Merging. MIN_MD_TO_MERGE(%i) \n", MIN_MD_TO_MERGE);
	if (list_of_estimates.size() > 0){
        //std::unique_ptr<std::vector <EstimateContainer> > merged_estimates(
                    //new std::vector<EstimateContainer>(estimates_count*2));
		double Comp_Size = list_of_estimates_.size();
		std::vector<double> newClusterIndexes;
        newClusterIndexes.reserve(Comp_Size);
		std::vector<bool> belAvailable_flags(Comp_Size,true);
		std::vector<EstimateContainer> merged_estimates_list;
        merged_estimates_list.reserve(Comp_Size);
		double highest_points_VecIndex, MaxW=0.0;
		int MaxWInd =0; //higest weight index
		double notUsedbel = Comp_Size;
		double belUsedcounter=0;
		while (notUsedbel > 0) {
			newClusterIndexes.clear();
			MaxW=0.0;
			for (double i = 0; i < Comp_Size; i++) {//finding highest weight in the vector
				if (belAvailable_flags[i] == true && list_of_estimates[i].weight > MaxW) {
					MaxW = list_of_estimates[i].weight;
					MaxWInd = i;
				}
			}
			for (int j = 0; j < Comp_Size;  j++) {
				//cout<<" i = "<<i<<" j = "<<j<<endl;
				if (belAvailable_flags[j] == true) {
					double Mahalanobis_Dis = GetMahalanobisDistance(list_of_estimates[MaxWInd].estimate, list_of_estimates[j].estimate);
					//save the points to a group which are within closest distance given. if not disregard
					if (Mahalanobis_Dis <= MIN_MD_TO_MERGE) {
						newClusterIndexes.push_back(j);//save vector index???
						belAvailable_flags[j] = false;
						notUsedbel--;
						belUsedcounter++;
					}
				}
			}
			if (newClusterIndexes.size() > 0) {
				//merged_estimates_list.push_back( get_cluster_mean_Cov(newClusterIndexes,list));
                //GaussianComponent PHDfilter::get_cluster_mean_Cov(vector<double>& indexVector,vector<GaussianComponent>& original){
                /* computing weighted mean and covariance*/
                double weight_cumulated = 0;
                EstimateContainer merged_estimate;
                merged_estimate.detected = 0;
                merged_estimate.undetected = 0;
                merged_estimate.undetected_since_last_detected = 0;
                merged_estimate.mslh_lp = 0;
                Eigen::Vector2d mean = Eigen::Vector2d::Zero();
                Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
                int max_weight_index = 0;
                double max_weight = 0.0;
                for (int i = 0; i < newClusterIndexes.size(); i++) {
                    double index_for_OriginalVec = newClusterIndexes[i];
                    double w_original = list_of_estimates[index_for_OriginalVec].weight;
                    weight_cumulated += w_original;
                    if (max_weight <= w_original) {
                        max_weight = w_original;
                        max_weight_index = index_for_OriginalVec;
                    }
                    mean += list_of_estimates[index_for_OriginalVec].estimate.getMean() * 
                            list_of_estimates[index_for_OriginalVec].weight;
                    Eigen::Vector2d diff = ((mean / weight_cumulated) - 
                                    list_of_estimates[index_for_OriginalVec].estimate.getMean());
                    covariance += list_of_estimates[index_for_OriginalVec].weight * 
                                    (list_of_estimates[index_for_OriginalVec].estimate.getCovariance() +
                                    (diff*diff.transpose()));
                    merged_estimate.detected += list_of_estimates[index_for_OriginalVec].detected * w_original;
                    merged_estimate.undetected += list_of_estimates[index_for_OriginalVec].undetected * w_original;
                    merged_estimate.undetected_since_last_detected += list_of_estimates[index_for_OriginalVec].undetected_since_last_detected
                                                                    * w_original;
                    merged_estimate.mslh_lp += list_of_estimates[index_for_OriginalVec].mslh_lp * w_original;
                    auto meanToPrint = list_of_estimates[index_for_OriginalVec].estimate.getMean();
                    std::printf("[mergedNeareastComponents] x:(%f), y:(%f), w:(%f) ,", meanToPrint[0], meanToPrint[1], list_of_estimates[index_for_OriginalVec].weight);
                }
                // mean=mean/weight_cumulated;
                // covariance=covariance/weight_cumulated;
                if (filter_setting == "adaptive" || filter_setting == "adaptive_regular_ps" ) {
                    merged_estimate = list_of_estimates[max_weight_index];
                    //mean and cov will be copies in the end
                    // merged_estimate.detected = list_of_estimates[max_weight_index].detected;
                    // merged_estimate.undetected = list_of_estimates[max_weight_index].undetected;
                    // merged_estimate.undetected_since_last_detected = list_of_estimates[max_weight_index].undetected_since_last_detected;
                    // merged_estimate.detected /= weight_cumulated;
                    // merged_estimate.undetected /= weight_cumulated;
                    // merged_estimate.undetected_since_last_detected /= weight_cumulated;
                }
                else if (filter_setting == "adaptive_low_pass") {
                    merged_estimate.detected /= weight_cumulated;
                    merged_estimate.undetected /= weight_cumulated;
                    merged_estimate.undetected_since_last_detected /= weight_cumulated;
                    merged_estimate.mslh_lp /= weight_cumulated;
                }
                else if (filter_setting == "Standard") {
                    //nothing to do here for stander phd filter
                }
                else {
                    throw std::invalid_argument("[mergedNeareastComponents] invalid filter setting.. huh! you forgot update here right!!!\n");
                }

                //copy mean and covaricance here
                merged_estimate.estimate.setMean(mean/weight_cumulated);
                merged_estimate.estimate.setCovariance(covariance/weight_cumulated);
                merged_estimate.weight = weight_cumulated;
                merged_estimates_list.push_back(merged_estimate);
                auto mergMeanToPrint = merged_estimate.estimate.getMean();
                std::printf("\n[mergedNeareastComponents] merged: x(%f), y(%f), w(%f)\n", mergMeanToPrint[0], mergMeanToPrint[1], merged_estimate.weight);
			}
		}
		list_of_estimates.clear();
        list_of_estimates.reserve(merged_estimates_list.size());
		list_of_estimates = merged_estimates_list;
        // std::cout<<" buffer sizes - ";
        // for (auto e : list_of_estimates){
        //     std::cout<<","<<e.buffer_for_pd_.size();
        // }
        // std::cout<<endl;
	}
}

void Filter::getCopyofGroundTruth() {
    if(EstimateContainer::isSyntheticMeasurementsProvided || gazebo_sim_rosbag) {
        std::lock_guard <std::mutex> lg(cloud_syntheticGT_lock);
        measurement_copy_to_compute_error = cloud_syntheticGT_;
    }

}

void Filter::CommonTimeUpdateSteps() {
// if ( !list_of_estimates_.empty())  {/*wait until new odom available*/
    //     std::cout << "[GetTimeUpdate] waiting till new odom available" << std::endl;
    //     return;
    // }

    // odom_now_lock_.lock();
    std::printf("[TUCommonStep] time update common step started------------------ \n");
    std::unique_lock <std::mutex> ul (odom_now_lock_);
    nav_msgs::Odometry odom_copy = odom_now_;
    is_new_odom_available_ = false;
    ul.unlock();
    if(!is_odom_init) {
        previous_odom_copy_ = odom_copy;
        if(gazebo_sim_rosbag) {
            Eigen::Quaterniond prior_quat;
            tf2::fromMsg(previous_odom_copy_.pose.pose.orientation, prior_quat);
            prior_quat.normalize();
            double yaw_angle  = uri_soft_base::quaternion_to_yaw(prior_quat);
            Eigen::Rotation2Dd rot_matrix_current_step(yaw_angle);
            // Eigen::Matrix2d rotationmat = rot_matrix_current_step.toRotationMatrix() * homogeneous_tranform_matrix.topLeftCorner (2,2);
            homogeneous_tranform_matrix.topLeftCorner (2,2) =  rot_matrix_current_step.toRotationMatrix();
        }
        is_odom_init = true;
    }

    Eigen::Vector3d prev_timestep_pos, current_timestep_pos;
    tf2::fromMsg(odom_copy.pose.pose.position, current_timestep_pos);
    tf2::fromMsg(previous_odom_copy_.pose.pose.position, prev_timestep_pos);
    Eigen::Vector3d displacement_between_timesteps = current_timestep_pos - prev_timestep_pos;
    // translation_to_map = current_timestep_pos;

    /*  QUATERNION --->
        ros uses two Quaternion types. tf::Quaternion and geometry::quaternion
        convert from geometry to tf --> tf2::fromMsg, tf::convert
        convert from geometry to eigen --> same as above
        convert from tf to geometry --> tf2::toMsg
        http://wiki.ros.org/tf2/Tutorials/Quaternions
        only tf2 has a library with Quaternion multiplications not geometry::quaternons. 
        Eigen library can be used but need to convert from geometry to eigen
    */

    Eigen::Quaterniond prior_quat, current_quat, relative_rot;
    tf2::fromMsg(previous_odom_copy_.pose.pose.orientation, prior_quat);
    tf2::fromMsg(odom_copy.pose.pose.orientation, current_quat);
    relative_rot = current_quat * prior_quat.inverse(); //quat2 = q_rot * quat1 --> q_rot = quat2*quat1^-1
    relative_rot.normalize(); //magnitude of a quaternion should be one. otherwise ros will print error msgs
    double yaw_angle  = uri_soft_base::quaternion_to_yaw(relative_rot);
    Eigen::Rotation2Dd rot_matrix_current_step(yaw_angle);

    Eigen::Vector2d displacement_rotated_to_prev_timestep = rot_prev_timestep_.toRotationMatrix().transpose() * 
                                                            Eigen::Vector2d(displacement_between_timesteps.x(), displacement_between_timesteps.y());
    if(gazebo_sim_rosbag) {
        //for gazebo simulation, create Homog matrix for estimates to translate back to map

        Eigen::Matrix2d rotationmat = rot_matrix_current_step.toRotationMatrix() * homogeneous_tranform_matrix.topLeftCorner (2,2);
        homogeneous_tranform_matrix.topLeftCorner (2,2) =  rotationmat;
        homogeneous_tranform_matrix.topRightCorner (3,1) = current_timestep_pos;
        std::cout << "Homogeneious matrix = \n" << homogeneous_tranform_matrix << std::endl;

    }

    for ( auto& estimate : list_of_estimates_ ) {
        EstimateContainer time_updated_estimate(estimate);
        Eigen::Vector2d timeUpdateMean = rot_matrix_current_step.toRotationMatrix().transpose()*(estimate.estimate.getMean() - displacement_rotated_to_prev_timestep);
        Eigen::Matrix2d timeUpdateCov = rot_matrix_current_step * estimate.estimate.getCovariance() * rot_matrix_current_step.toRotationMatrix().transpose()
                                        + rot_matrix_current_step * PROCESS_NOISE * rot_matrix_current_step.toRotationMatrix().transpose();
        time_updated_estimate.estimate.setMean(timeUpdateMean);
        time_updated_estimate.estimate.setCovariance(timeUpdateCov);
        // time_updated_estimate.weight = time_updated_estimate.ComputePs() * time_updated_estimate.weight;
        
        std::cout << "Before TU \n";
        estimate.print();
        estimate = time_updated_estimate;
        std::cout << "After TU \n";
        estimate.print();
    }

    EstimateContainer BIRTH_COMPONENT;
    BIRTH_COMPONENT.estimate = uri_soft_base::Gaussian(Eigen::Matrix2d::Identity() * birth_target_cov);
    BIRTH_COMPONENT.weight = MINIMUM_WEIGHT;
    // BIRTH_COMPONENT.detected = 0.1;
    // BIRTH_COMPONENT.undetected = 0.9;

    list_of_estimates_.push_back(BIRTH_COMPONENT);
    previous_odom_copy_ = odom_copy;
    rot_prev_timestep_ = rot_matrix_current_step;
    std::printf("[TUCommonStep] time update common step finished -------------------\n");
    // std::cout << "[GetTimeUpdate] time update complete" + ns << std::endl;
}

void Filter::GetTimeUpdateUsingAdaptivePs() {
    std::cout << "[GetTimeUpdate] time update starting" + ns << std::endl;
    CommonTimeUpdateSteps();
    for ( auto& estimate : list_of_estimates_ ) {
        auto ps = estimate.ComputePs(APSParam);
        std::printf("[GetTUAdap] Final_weight(%f) = ps (%f) * prior_weight(%f), APSParam: %i \n", 
                        ps * estimate.weight, ps, estimate.weight, APSParam);
        estimate.weight = ps * estimate.weight;
    }
    list_of_estimates_.back().weight = MINIMUM_WEIGHT; //for birth target weight
    std::cout << "[GetTimeUpdate] time update complete" + ns << std::endl;
}

void Filter::GetTimeUpdate() {

    CommonTimeUpdateSteps();
    for ( auto& estimate : list_of_estimates_ ) {
        // std::printf("[TUAdaptive] Final_weight(%f) = ps (%f) * prior_weight(%f)\n", 
        //                 PS * estimate.weight, PS, estimate.weight);
        estimate.weight = PS * estimate.weight;
    }
    list_of_estimates_.back().weight = MINIMUM_WEIGHT;
    std::cout << "[GetTimeUpdate] time update complete" + ns << std::endl;
}

void Filter::GetMeasurementUpdateUsingAdaptivePd() {
    std::cout << "[GetMeasUpdate] Meas update LIDAR started" + ns << std::endl;
    sensor_msgs::PointCloud measurement_copy;
    cloud_lock_.lock();
    measurement_copy = cloud_;
    cloud_lock_.unlock();
    getCopyofGroundTruth();
    // if(EstimateContainer::sensor_pd_correction) requestBlindspotDetails();

    is_measurement_available_ = false;
    if (list_of_estimates_.empty()) return;
    int estimates_count = list_of_estimates_.size();
    int meas_count = measurement_copy.points.size();
    std::unique_ptr<std::vector<EstimateContainer> > measurement_updated_estimates(
                    new std::vector<EstimateContainer>());
    measurement_updated_estimates->reserve(estimates_count * meas_count * 2);
    //following data containers are for visualization purpose
    // std::vector<uri_soft_base::Gaussian> list_of_estimates_to_plot;
    std::vector<Eigen::Vector2d> list_of_meas_to_plot;
    // std::vector<double> list_of_weights_to_plot;
    // list_of_estimates_to_plot.reserve(estimates_count * meas_count);
    list_of_meas_to_plot.reserve(meas_count);
    // sensor_msgs::PointCloud msg, msg_to_plotter;
    //----------------------------------------------------------
    std::printf("[GetMeasUpdate] --1 - pd update started--------------------- \n");
    for (auto prior_estimate : list_of_estimates_) {
        auto one_minus_pd = (1 - prior_estimate.ComputePd());
        double new_weight = prior_estimate.weight * one_minus_pd;
        std::printf("[MU] new weight (%f) = prior_weight(%f) * (1-pd) (%f) > Minimum_weight (%f) \n", 
                                            new_weight, prior_estimate.weight, one_minus_pd, MINIMUM_WEIGHT);
        // std::printf("(1-pd) new weight = %f, undetected = %f, detected = %f",new_weight,prior_estimate.undetected, prior_estimate.detected);
        if (new_weight > MINIMUM_WEIGHT) {
            // std::printf("(1-pd) undetected = %f detected = %f",prior_estimate.undetected, prior_estimate.detected);
            prior_estimate.IncreaseUndetected(); ///TODO check here
            // std::printf("(1-pd) undetected = %f detected = %f /n",prior_estimate.undetected, prior_estimate.detected);
            auto m = prior_estimate.estimate.getMean();
            measurement_updated_estimates->push_back(prior_estimate);
            measurement_updated_estimates->back().weight = new_weight;
            std::printf("[LidarMU] (1- pd) Pruned comp- weight (%f), mean (%f, %f)\n", prior_estimate.weight, m[0], m[1]);
            // measurement_updated_estimates->back().IncreaseUndetected();///TODO check why this is pissu kellinne
        }
    }
    double w_for_normalization = 0;
    std::vector <EstimateContainer> meas_updated_before_nomalized;
    meas_updated_before_nomalized.reserve(estimates_count);
    std::printf("\n[GetMeasUpdate]  --pd update (measurement) started------------------------------------------ \n");
    for (const auto& measurement : measurement_copy.points) {
        std::printf("[MUAdap] Measurement x: %f, y: %f \n",measurement.x, measurement.y);
        Eigen::Vector2d meas_eigen_converted(measurement.x, measurement.y);
        list_of_meas_to_plot.push_back(meas_eigen_converted);
        for (const auto& prior_estimate :  list_of_estimates_) {

            //copy data from prior estimate
            EstimateContainer estimate_meas_updated(prior_estimate);

            std::printf("[MU]prior estimate = \n");
            estimate_meas_updated.print();

            Eigen::Vector2d e_mean = estimate_meas_updated.estimate.getMean();
            Eigen::Matrix2d e_cov = estimate_meas_updated.estimate.getCovariance();
            double pd = estimate_meas_updated.ComputePd();
                        
            //construction of phd update components
            Eigen::Matrix2d Rk = Eigen::Matrix2d::Identity() * sensor_noise; //is the observation noise covariance default = 0.001
            Eigen::Matrix2d Hk = Eigen::Matrix2d::Identity();//is the observation matrix
            Eigen::Matrix2d Sk = Rk + (Hk * e_cov * Hk.transpose());
            Eigen::Matrix2d Kk = e_cov * Hk.transpose() * Sk.inverse();
            Eigen::Vector2d nKminus1toK = Hk * e_mean;
            Eigen::Matrix2d Pktok = (Eigen::Matrix2d::Identity() - Kk * Hk) * e_cov;

            //calculating new mean
            Eigen::Vector2d mean_updated = e_mean + Kk * (meas_eigen_converted - nKminus1toK);
            auto error = meas_eigen_converted - nKminus1toK;
            std::printf("[MU] updated_mean x(%f), y(%f) = prior_estimate * kk* error(%f, %f)\n", 
                        mean_updated[0], mean_updated[1], error[0], error[1]);
            
            estimate_meas_updated.estimate.setMean(mean_updated);
            estimate_meas_updated.estimate.setCovariance(Pktok);
            //calcualting the weight. not normalized yet
            estimate_meas_updated.weight = pd * prior_estimate.weight * 
                                           getPDensity(meas_eigen_converted, nKminus1toK, Sk);
            auto Pdensity = getPDensity(meas_eigen_converted, nKminus1toK, Sk); //just to print
            w_for_normalization += estimate_meas_updated.weight;
            estimate_meas_updated.IncreaseDetected();
            meas_updated_before_nomalized.push_back(estimate_meas_updated);

            std::printf("[MU] Update_weight(%f) = pd(%f) * prior_estimate.weight(%f) * ProbDensity(%f), SumOfWeight =(%f) \n",
                        estimate_meas_updated.weight, pd, prior_estimate.weight, Pdensity, w_for_normalization);

        }
        //normalizing weights of calculated new estimates
        //pruning the lower weights
        std::printf("[MU] pruning started... \n");
        for (auto& estimate : meas_updated_before_nomalized) {
            estimate.weight = w_for_normalization > 0.0 ? estimate.weight /= w_for_normalization : 0.0; //preventing division by 1/0
            auto mean = estimate.estimate.getMean();
            std::printf("[LidarMU] beforePruned normalize weight (%f), mean (%f, %f)\n", estimate.weight, mean[0], mean[1]);
            if (estimate.weight > MINIMUM_WEIGHT) {
                measurement_updated_estimates->push_back(estimate);
            }
        }
        w_for_normalization = 0;
        meas_updated_before_nomalized.clear();
        meas_updated_before_nomalized.reserve(estimates_count);
    }
    list_of_estimates_.clear();
    list_of_estimates_.reserve(measurement_updated_estimates->size());
    list_of_estimates_ = *measurement_updated_estimates;
    measurement_updated_estimates.reset();
    mergedNeareastComponents(list_of_estimates_);
    std::cout << "[GetMeasUpdate] Measurement update complete----------------" + ns << std::endl;
    // publishData();
}

void Filter::GetMeasurementUpdate () {
    std::cout << "[GetMeasUpdate] Meas update started - " + ns << std::endl;
    sensor_msgs::PointCloud measurement_copy;
    cloud_lock_.lock();
    measurement_copy = cloud_;
    cloud_lock_.unlock();
    getCopyofGroundTruth();

    is_measurement_available_ = false;
    if (list_of_estimates_.empty()) return;
    int estimates_count = list_of_estimates_.size();
    int meas_count = measurement_copy.points.size();
    std::unique_ptr<std::vector<EstimateContainer> > measurement_updated_estimates(
                    new std::vector<EstimateContainer>());
    measurement_updated_estimates->reserve(estimates_count*2);
    //following data containers are for visualization purpose
    // std::vector<uri_soft_base::Gaussian> list_of_estimates_to_plot;
    std::vector<Eigen::Vector2d> list_of_meas_to_plot;
    // std::vector<double> list_of_weights_to_plot;
    // list_of_estimates_to_plot.reserve(estimates_count * meas_count);
    list_of_meas_to_plot.reserve(meas_count);
    // sensor_msgs::PointCloud msg, msg_to_plotter;
    //----------------------------------------------------------
    for (auto prior_estimate : list_of_estimates_) {
        // prior_estimate.IncreaseUndetected();
        double new_weight = prior_estimate.weight * (1 - PD);
        if (new_weight > MINIMUM_WEIGHT) {
            prior_estimate.weight = new_weight;
            measurement_updated_estimates->push_back(prior_estimate);
            // measurement_updated_estimates->back().weight = new_weight;
            // measurement_updated_estimates->back().IncreaseUndetected();
        }
    }
    double w_for_normalization = 0;
    std::vector <EstimateContainer> meas_updated_before_nomalized;
    meas_updated_before_nomalized.reserve(estimates_count);
    for (auto& measurement : measurement_copy.points) {
        Eigen::Vector2d meas_eigen_converted(measurement.x, measurement.y);
        list_of_meas_to_plot.push_back(meas_eigen_converted);
        for (auto& prior_estimate :  list_of_estimates_) {
            Eigen::Vector2d e_mean = prior_estimate.estimate.getMean();
            Eigen::Matrix2d e_cov = prior_estimate.estimate.getCovariance();
            EstimateContainer estimate_meas_updated(prior_estimate);

            //construction of phd update components
            Eigen::Matrix2d Rk = Eigen::Matrix2d::Identity() * sensor_noise; //sensor noise
            Eigen::Matrix2d Hk = Eigen::Matrix2d::Identity();
            Eigen::Matrix2d Sk = Rk + (Hk * e_cov * Hk.transpose());
            Eigen::Matrix2d Kk = e_cov * Hk.transpose() * Sk.inverse();
            Eigen::Vector2d nKminus1toK = Hk * e_mean;
            Eigen::Matrix2d Pktok = (Eigen::Matrix2d::Identity() - Kk * Hk) * e_cov;

            //calculating new mean
            Eigen::Vector2d mean_updated = e_mean + Kk * (meas_eigen_converted - nKminus1toK);
            
            estimate_meas_updated.estimate.setMean(mean_updated);
            estimate_meas_updated.estimate.setCovariance(Pktok);
            // estimate_meas_updated.IncreaseDetected();
            //calcualting the weight. not normalized yet
            estimate_meas_updated.weight = PD * prior_estimate.weight * 
                                           getPDensity(meas_eigen_converted, nKminus1toK, Sk);
            w_for_normalization += estimate_meas_updated.weight;
            meas_updated_before_nomalized.push_back(estimate_meas_updated);

        }
        //normalizing weights of calculated new estimates
        //pruning the lower weights
        for (auto& estimate : meas_updated_before_nomalized) {
            estimate.weight = w_for_normalization > 0.0 ? estimate.weight /= w_for_normalization : 0.0; //preventing division by 1/0;
            if (estimate.weight > MINIMUM_WEIGHT) {
                measurement_updated_estimates->push_back(estimate);

            }
        }
        w_for_normalization = 0;
        meas_updated_before_nomalized.clear();
        meas_updated_before_nomalized.reserve(estimates_count);
    }
    list_of_estimates_.clear();
    list_of_estimates_.reserve(measurement_updated_estimates->size());
    list_of_estimates_ = *measurement_updated_estimates;
    measurement_updated_estimates.reset();
    mergedNeareastComponents(list_of_estimates_);

    std::cout << "[GetMeasUpdate] Measurement update complete" + ns << std::endl;
    // publishData();
}

void Filter::GetMeasurementUpdateLowPass () {
    std::cout << "[GetMeasUpdate] Meas update started - " + ns << std::endl;
    sensor_msgs::PointCloud measurement_copy;
    cloud_lock_.lock();
    measurement_copy = cloud_;
    cloud_lock_.unlock();
    getCopyofGroundTruth();

    is_measurement_available_ = false;
    if (list_of_estimates_.empty()) return;
    int estimates_count = list_of_estimates_.size();
    int meas_count = cloud_.points.size();
    std::unique_ptr<std::vector<EstimateContainer> > measurement_updated_estimates(
                    new std::vector<EstimateContainer>());
    measurement_updated_estimates->reserve(estimates_count * 2);
    //following data containers are for visualization purpose
    // std::vector<uri_soft_base::Gaussian> list_of_estimates_to_plot;
    std::vector<Eigen::Vector2d> list_of_meas_to_plot;
    // std::vector<double> list_of_weights_to_plot;
    // list_of_estimates_to_plot.reserve(estimates_count * meas_count);
    list_of_meas_to_plot.reserve(meas_count);
    // sensor_msgs::PointCloud msg, msg_to_plotter;
    //----------------------------------------------------------
    for (auto prior_estimate : list_of_estimates_) {
        double pd_lp = prior_estimate.ComputePdMSLH();
        double new_weight = prior_estimate.weight * (1 - pd_lp);
        if (new_weight > MINIMUM_WEIGHT) {
            prior_estimate.IncreaseDetected();
            prior_estimate.weight = new_weight;
            measurement_updated_estimates->push_back(prior_estimate);
            // measurement_updated_estimates->back().weight = new_weight;
            // measurement_updated_estimates->back().IncreaseUndetected();
        }
    }
    double w_for_normalization = 0;
    std::vector <EstimateContainer> meas_updated_before_nomalized;
    meas_updated_before_nomalized.reserve(estimates_count);
    for (auto& measurement : measurement_copy.points) {
        Eigen::Vector2d meas_eigen_converted(measurement.x, measurement.y);
        list_of_meas_to_plot.push_back(meas_eigen_converted);
        for (auto& prior_estimate :  list_of_estimates_) {
            Eigen::Vector2d e_mean = prior_estimate.estimate.getMean();
            Eigen::Matrix2d e_cov = prior_estimate.estimate.getCovariance();
            EstimateContainer estimate_meas_updated(prior_estimate);

            //construction of phd update components
            Eigen::Matrix2d Rk = Eigen::Matrix2d::Identity() * sensor_noise; //sensor noise
            Eigen::Matrix2d Hk = Eigen::Matrix2d::Identity();
            Eigen::Matrix2d Sk = Rk + (Hk * e_cov * Hk.transpose());
            Eigen::Matrix2d Kk = e_cov * Hk.transpose() * Sk.inverse();
            Eigen::Vector2d nKminus1toK = Hk * e_mean;
            Eigen::Matrix2d Pktok = (Eigen::Matrix2d::Identity() - Kk * Hk) * e_cov;

            //calculating new mean
            Eigen::Vector2d mean_updated = e_mean + Kk * (meas_eigen_converted - nKminus1toK);

            estimate_meas_updated.estimate.setMean(mean_updated);
            estimate_meas_updated.estimate.setCovariance(Pktok);
            //calcualting the weight. not normalized yet
            double pd_lp = prior_estimate.ComputePdMSLH();
            estimate_meas_updated.weight = pd_lp * prior_estimate.weight * 
                                           getPDensity(meas_eigen_converted, nKminus1toK, Sk);
            w_for_normalization += estimate_meas_updated.weight;
            estimate_meas_updated.IncreaseDetected();
            meas_updated_before_nomalized.push_back(estimate_meas_updated);
        }
        //normalizing weights of calculated new estimates
        //pruning the lower weights
        for (auto& estimate : meas_updated_before_nomalized) {
            estimate.weight = w_for_normalization > 0.0 ? estimate.weight /= w_for_normalization : 0.0; //preventing division by 1/0
            if (estimate.weight > MINIMUM_WEIGHT) {
                measurement_updated_estimates->push_back(estimate);
            }
        }
        w_for_normalization = 0;
        meas_updated_before_nomalized.clear();
        meas_updated_before_nomalized.reserve(estimates_count);
    }
    list_of_estimates_.clear();
    list_of_estimates_.reserve(measurement_updated_estimates->size());
    list_of_estimates_ = *measurement_updated_estimates;
    measurement_updated_estimates.reset();
    mergedNeareastComponents(list_of_estimates_);

    std::cout << "[GetMeasUpdate] Measurement update complete" + ns << std::endl;
    // publishData();
}

void Filter::GetMeasurementUpdateCamera () {

    std::cout << "[GetMeasUpdate] CAMERA Meas update started" + ns << std::endl;
    
    std::unique_lock <std::mutex> ul (cam_angle_readerLock);
    auto measurement_copy = camera_measurements_; //camera points rotated to robot frame
    Filter::is_new_camera_meas_avail_ = false;
    if (camera_measurements_.points.empty()) {
        std::printf("[GetMeasUpCam] if (camera_measurements_.points.empty()(%i)) -> return \n", int(camera_measurements_.points.empty()) );
        return;
    }
    ul.unlock();
    //isCameraNewMessageAvailable = false;
    // getCopyofGroundTruth();
    
    if (list_of_estimates_.empty()) return;
    int estimates_count = list_of_estimates_.size();
    int meas_count = measurement_copy.points.size();
    std::unique_ptr<std::vector<EstimateContainer> > measurement_updated_estimates(
                    new std::vector<EstimateContainer>());
    measurement_updated_estimates->reserve(estimates_count * meas_count * 2);

    // std::vector<Eigen::Vector2d> list_of_meas_to_plot;
    // list_of_meas_to_plot.reserve(meas_count);

    //----------------------------------------------------------
    std::printf("[GetMeasUpCam] (1-Pd) update started---------------------------- \n");
    for (auto prior_estimate : list_of_estimates_) {
        auto pd = PD;//prior_estimate.ComputePd();
        double new_weight = prior_estimate.weight * (1 - pd);
        std::printf("[GetMeasUpCam] (1-pd) new weight(%f) = prior_w(%f) * (1 - prior_pd(%f)), pruneW (%f) \n",new_weight, prior_estimate.weight, pd, MINIMUM_WEIGHT);
        if (new_weight > MINIMUM_WEIGHT) {
            // std::printf("(1-pd) undetected = %f detected = %f",prior_estimate.undetected, prior_estimate.detected);
            // prior_estimate.IncreaseUndetected(); ///TODO check here
            // std::printf("(1-pd) undetected = %f detected = %f /n",prior_estimate.undetected, prior_estimate.detected);
            measurement_updated_estimates->push_back(prior_estimate);
            measurement_updated_estimates->back().weight = new_weight;
            // measurement_updated_estimates->back().IncreaseUndetected();///TODO check why this is pissu kellinne
        }
    }
    double w_for_normalization = 0;
    std::vector <EstimateContainer> meas_updated_before_nomalized;
    meas_updated_before_nomalized.reserve(estimates_count);
    std::printf("[GetMeasUpCam] Pd update started--------------------------------- \n");
    for (const auto& measurement : measurement_copy.points) {
        Eigen::Vector2d meas_eigen_converted(measurement.x, measurement.y);
        double bearing_measurement = std::atan2(measurement.y, measurement.x); //radian
        std::printf("[GetMeasUpCam] cam Measuremnt x: %f, y: %f, angle: %f \n",measurement.x, measurement.y, bearing_measurement);
        // list_of_meas_to_plot.push_back(meas_eigen_converted);
        for (const auto& prior_estimate :  list_of_estimates_) {

            //copy data from prior estimate
            EstimateContainer estimate_meas_updated(prior_estimate);
            Eigen::Vector2d e_mean = estimate_meas_updated.estimate.getMean();
            Eigen::Matrix2d e_cov = estimate_meas_updated.estimate.getCovariance();
            double pd = PD;//estimate_meas_updated.ComputePd();
            auto sq_sum = e_mean[0]*e_mean[0] + e_mean[1]*e_mean[1];
            std::printf("[GetMeasUpCam] estimate x: %f, y: %f, pd: %f, cov:\n",e_mean[0], e_mean[1], pd);
            std::cout << e_cov << std::endl;

            // double camera_noise_variance = std::pow((3.0 * M_PI / 180), 2.0);
            double camera_noise_variance_Rk = std::pow((camera_noise_variance * M_PI / 180), 2.0);
            auto Rk = camera_noise_variance_Rk; //is the observation noise covariance default = 0.001
            Eigen::RowVector2d Hk(-1*e_mean[1]/sq_sum, e_mean[0]/sq_sum);
            std::cout << "[GetMeasUpCam] hk "<< Hk[0] <<", " << Hk[1] <<std::endl;
            auto Sk = Rk + (Hk * e_cov * Hk.transpose());
            std::cout << "[GetMeasUpCam] camera_noise_variance / Rk: "<< camera_noise_variance <<", Sk: " << Sk <<std::endl;

            auto Kk = e_cov * Hk.transpose() / Sk;
            std::cout << "[GetMeasUpCam] kk: \n" << Kk[0]<<", "<<Kk[1]<< std::endl;
            auto nKminus1toK =  (std::atan2(e_mean[1], e_mean[0])); //radian
            auto Pktok = (Eigen::Matrix2d::Identity() - Kk * Hk) * e_cov;
            std::cout << "[GetMeasUpCam] Pktok: \n" << Pktok << std::endl;
            double error = bearing_measurement - nKminus1toK;
            std::printf("[GetMeasUpCam] error(deg)(%f) = bearing_measurement(%f) - nKminus1toK(%f) \n",
                                rad2deg(error), rad2deg(bearing_measurement), rad2deg(nKminus1toK));
            //calculating new mean
            Eigen::Vector2d mean_updated = e_mean + Kk * (bearing_measurement - nKminus1toK);
            std::printf("[GetMeasUpCam] update_mean (%f, %f) = prior_mean(%f, %f) + kk * (bearing (%f) - prior(%f)) \n",
                                mean_updated[0],mean_updated[1],e_mean[0], e_mean[1],bearing_measurement,nKminus1toK);
            
            estimate_meas_updated.estimate.setMean(mean_updated);
            estimate_meas_updated.estimate.setCovariance(Pktok);
            //calcualting the weight. not normalized yet

            double WeightGD = exp( -0.5 * (error / Sk) * (error / Sk)) / ( Sk * sqrt( 2 * M_PI));
            std::printf("[GetMeasUpCam] WeightGD (%f) = exp( -0.5 * (error(%f) / Sk (%f)) * (error / Sk)) / (%f)( Sk * sqrt( 2 * M_PI)); \n",
                                            WeightGD, error, Sk, (Sk * sqrt( 2 * M_PI)));
            estimate_meas_updated.weight = pd * prior_estimate.weight * WeightGD;
            std::printf("[GetMeasUpCam] update weight(%f) = pd(%f) * priorW(%f) * gaussianDensity (%f), sum_weight(%f)\n",
                                            estimate_meas_updated.weight, pd, prior_estimate.weight, WeightGD, w_for_normalization);
            w_for_normalization += estimate_meas_updated.weight;
            // estimate_meas_updated.IncreaseDetected();
            meas_updated_before_nomalized.push_back(estimate_meas_updated);
        }
        //normalizing weights of calculated new estimates
        //pruning the lower weights
        std::printf("[GetMeasUpCam] pruning ... \n");
        for (auto& estimate : meas_updated_before_nomalized) {
            estimate.weight = w_for_normalization > 0 ? estimate.weight/= w_for_normalization : 0;
            auto mean = estimate.estimate.getMean();
            std::printf("[GetMeasUpCam] normalize weight (%f), mean (%f, %f)\n", estimate.weight, mean[0], mean[1]);
            if (estimate.weight > MINIMUM_WEIGHT) {
                measurement_updated_estimates->push_back(estimate);
            }
        }
        w_for_normalization = 0;
        meas_updated_before_nomalized.clear();
        meas_updated_before_nomalized.reserve(estimates_count);
    }
    list_of_estimates_.clear();
    list_of_estimates_.reserve(measurement_updated_estimates->size());
    list_of_estimates_ = *measurement_updated_estimates;
    measurement_updated_estimates.reset();
    mergedNeareastComponents(list_of_estimates_);
    std::cout << "[GetMeasUpdate] Measurement update complete" + ns << std::endl;
}

void Filter::GetMeasurementUpdateUsingAdaptivePdCamera () {

    std::cout << "[GetMeasUpdate] CAMERA Meas update started" + ns << std::endl;
    
    std::unique_lock <std::mutex> ul (cam_angle_readerLock);
    auto measurement_copy = camera_measurements_; //camera points rotated to robot frame
    Filter::is_new_camera_meas_avail_ = false;
        if (camera_measurements_.points.empty()) {
        std::printf("[GetMeasUpCam] if (camera_measurements_.points.empty()(%i)) -> return \n", int(camera_measurements_.points.empty()) );
        return;
    }
    ul.unlock();
    //isCameraNewMessageAvailable = false;
    // getCopyofGroundTruth();
    
    if (list_of_estimates_.empty()) return;
    int estimates_count = list_of_estimates_.size();
    int meas_count = measurement_copy.points.size();
    std::unique_ptr<std::vector<EstimateContainer> > measurement_updated_estimates(
                    new std::vector<EstimateContainer>());
    measurement_updated_estimates->reserve(estimates_count * meas_count * 2);

    // std::vector<Eigen::Vector2d> list_of_meas_to_plot;
    // list_of_meas_to_plot.reserve(meas_count);

    //----------------------------------------------------------
    std::printf("[GetMeasUpCam] (1-Pd) update started---------------------------- \n");
    for (auto prior_estimate : list_of_estimates_) {
        auto pd = prior_estimate.ComputePd();
        double new_weight = prior_estimate.weight * (1 - pd);
        std::printf("[GetMeasUpCam] (1-pd) new weight(%f) = prior_w(%f) * (1 - prior_pd(%f)), pruneW (%f) \n",new_weight, prior_estimate.weight, pd, MINIMUM_WEIGHT);
        if (new_weight > MINIMUM_WEIGHT) {
            // std::printf("(1-pd) undetected = %f detected = %f",prior_estimate.undetected, prior_estimate.detected);
            prior_estimate.IncreaseUndetected(); ///TODO check here
            // std::printf("(1-pd) undetected = %f detected = %f /n",prior_estimate.undetected, prior_estimate.detected);
            measurement_updated_estimates->push_back(prior_estimate);
            measurement_updated_estimates->back().weight = new_weight;
            // measurement_updated_estimates->back().IncreaseUndetected();///TODO check why this is pissu kellinne
        }
    }
    double w_for_normalization = 0;
    std::vector <EstimateContainer> meas_updated_before_nomalized;
    meas_updated_before_nomalized.reserve(estimates_count);
    std::printf("[GetMeasUpCam] Pd update started--------------------------------- \n");
    std::printf("[GetMeasUpCam] measurement size = (%i)\n", int(measurement_copy.points.size()));
    for (const auto& measurement : measurement_copy.points) {
        Eigen::Vector2d meas_eigen_converted(measurement.x, measurement.y);
        double bearing_measurement = std::atan2(measurement.y, measurement.x); //radian
        std::printf("[GetMeasUpCam] cam Measuremnt x: %f, y: %f, angle: %f \n",measurement.x, measurement.y, bearing_measurement);
        // list_of_meas_to_plot.push_back(meas_eigen_converted);
        for (const auto& prior_estimate :  list_of_estimates_) {

            //copy data from prior estimate
            EstimateContainer estimate_meas_updated(prior_estimate);
            Eigen::Vector2d e_mean = estimate_meas_updated.estimate.getMean();
            Eigen::Matrix2d e_cov = estimate_meas_updated.estimate.getCovariance();
            double pd = estimate_meas_updated.ComputePd();
            auto sq_sum = e_mean[0]*e_mean[0] + e_mean[1]*e_mean[1];
            std::printf("[GetMeasUpCam] estimate x: %f, y: %f, pd: %f, cov:\n",e_mean[0], e_mean[1], pd);
            std::cout << e_cov << std::endl;

            // RowVector2d Hkc;//observation matrix
			// 		Hkc[0] = -1*e_mean[1]/sq_sum;
			// 		Hkc[1] = e_mean[0]/sq_sum;

			// 		//checking for NAN values(one NaN is genrated viabirth targets)
			// 		if(isnan(Hkc[0])==true||isnan(Hkc[1])==true){
			// 			Hkc<<0.0,0.0;
			// 		}
			// 		double Rkk=CamCov;//observation noise covariance
			// 		double Skk =Rkk+(Hkc*PriorEstimates[j].covariance*Hkc.transpose());
			// 		Vector2d Kkk = PriorEstimates[j].covariance*Hkc.transpose()*(1/Skk);
			// 		double nKminus1toKk =PriorEstimates[j].get_angle();//Hkc*PriorEstimates[j].mean;//PriorEstimates[j].get_angle(); 
			// 					//RAD2DEG(atan2(PriorEstimates[j].mean[1],PriorEstimates[j].mean[0]));
			// 		//cout<<"nKminus1toKk = "<<nKminus1toKk<<endl;
			// 		Matrix2d Pktok=(I-Kkk*Hkc)*PriorEstimates[j].covariance;

            //construction of phd update components

            double camera_noise_variance = std::pow((3.0 * M_PI / 180), 2.0);
            auto Rk = camera_noise_variance; //is the observation noise covariance default = 0.001
            Eigen::RowVector2d Hk(-1*e_mean[1]/sq_sum, e_mean[0]/sq_sum);
            std::cout << "[GetMeasUpCam] hk "<< Hk[0] <<", " << Hk[1] <<std::endl;
            auto Sk = Rk + (Hk * e_cov * Hk.transpose());
            std::cout << "[GetMeasUpCam] camera_noise_variance / Rk: "<< camera_noise_variance <<", Sk: " << Sk <<std::endl;

            auto Kk = e_cov * Hk.transpose() / Sk;
            std::cout << "[GetMeasUpCam] kk: \n" << Kk[0]<<", "<<Kk[1]<< std::endl;
            auto nKminus1toK =  (std::atan2(e_mean[1], e_mean[0])); //radian
            auto Pktok = (Eigen::Matrix2d::Identity() - Kk * Hk) * e_cov;
            std::cout << "[GetMeasUpCam] Pktok: \n" << Pktok << std::endl;
            double error = bearing_measurement - nKminus1toK;
            std::printf("[GetMeasUpCam] error(deg)(%f) = bearing_measurement(%f) - nKminus1toK(%f) \n",
                                rad2deg(error), rad2deg(bearing_measurement), rad2deg(nKminus1toK));
            //calculating new mean
            Eigen::Vector2d mean_updated = e_mean + Kk * (bearing_measurement - nKminus1toK);
            std::printf("[GetMeasUpCam] update_mean (%f, %f) = prior_mean(%f, %f) + kk * (bearing (%f) - prior(%f)) \n",
                                mean_updated[0],mean_updated[1],e_mean[0], e_mean[1],bearing_measurement,nKminus1toK);
            
            estimate_meas_updated.estimate.setMean(mean_updated);
            estimate_meas_updated.estimate.setCovariance(Pktok);
            //calcualting the weight. not normalized yet

            double WeightGD = exp( -0.5 * (error / Sk) * (error / Sk)) / ( Sk * sqrt( 2 * M_PI));
            std::printf("[GetMeasUpCam] WeightGD (%f) = exp( -0.5 * (error(%f) / Sk (%f)) * (error / Sk)) / (%f)( Sk * sqrt( 2 * M_PI)); \n",
                                            WeightGD, error, Sk, (Sk * sqrt( 2 * M_PI)));
            estimate_meas_updated.weight = pd * prior_estimate.weight * WeightGD;
            std::printf("[GetMeasUpCam] update weight(%f) = pd(%f) * priorW(%f) * gaussianDensity (%f), sum_weight(%f)\n",
                                            estimate_meas_updated.weight, pd, prior_estimate.weight, WeightGD, w_for_normalization);
            w_for_normalization += estimate_meas_updated.weight;
            estimate_meas_updated.IncreaseDetected();
            meas_updated_before_nomalized.push_back(estimate_meas_updated);
        }
        //normalizing weights of calculated new estimates
        //pruning the lower weights
        std::printf("[GetMeasUpCam] pruning ... \n");
        for (auto& estimate : meas_updated_before_nomalized) {
            estimate.weight = w_for_normalization > 0.0 ? estimate.weight /= w_for_normalization : 0.0; //preventing division by 1/0;
            auto mean = estimate.estimate.getMean();
            std::printf("[GetMeasUpCam] normalize weight (%f), mean (%f, %f)\n", estimate.weight, mean[0], mean[1]);
            if (estimate.weight > MINIMUM_WEIGHT) {
                measurement_updated_estimates->push_back(estimate);
            }
        }
        w_for_normalization = 0;
        meas_updated_before_nomalized.clear();
        meas_updated_before_nomalized.reserve(estimates_count);
    }
    list_of_estimates_.clear();
    list_of_estimates_.reserve(measurement_updated_estimates->size());
    list_of_estimates_ = *measurement_updated_estimates;
    measurement_updated_estimates.reset();
    mergedNeareastComponents(list_of_estimates_);
    std::cout << "[GetMeasUpdate] Measurement update complete" + ns << std::endl;
}

double Filter::getMinErrorForMeasurementFromEstimateList(geometry_msgs::Point32 m, std::vector<EstimateContainer>& list_of_estimates) {
    std::pair <int, double> minError {-1, 1.0};
    int counter = 0;
    if (list_of_estimates.empty()) return 1.0; //handling the empty container
    for (auto& e : list_of_estimates) {
        Eigen::Vector2d estimate (e.estimate.getMean());
        Eigen::Vector2d measurement (m.x, m.y); 
        Eigen::Vector2d diff (estimate - measurement);
        double error = std::hypot(diff[0], diff[1]);
        // std::printf("\n error cal = %f\n", distance);
        if (error < minError.second) {
            // error = distance;
            minError.first = counter;
            minError.second = error;
        }
        counter++;
    }
    if(minError.first > -1) {
        list_of_estimates.erase(list_of_estimates.begin() + minError.first);
    }
    return minError.second;
}

void Filter::calculateError(sensor_msgs::ChannelFloat32& channel) {
    // measurement_copy_to_compute_error;
    // list_of_estimates_
    //Calculate error between estimate and ground truth
    double error = 0.0;
    std::vector <EstimateContainer> copyofestimate;
    copyofestimate.reserve(list_of_estimates_.size());
    if(gazebo_sim_rosbag) {
        //estiamtes tranform to map frame to compute the error
        for (auto& i :  list_of_estimates_) {
            copyofestimate.push_back(i);
            auto mean = i.estimate.getMean();
            Eigen::Vector4d point_in_homogForm (mean.x(),mean.y(),0.45,1);
            Eigen::Vector4d tranformed_to_mapframe = homogeneous_tranform_matrix * point_in_homogForm;
            mean[0] = tranformed_to_mapframe[0];
            mean[1] = tranformed_to_mapframe[1];
            copyofestimate.back().estimate.setMean(mean);
        }
    }
    else {
        copyofestimate = list_of_estimates_;
    }
    // std::printf("\n[ErrorCal] error calculation before for loop (%f)\n",measurement_copy_to_compute_error.points.size() );
    for (auto m : measurement_copy_to_compute_error.points){
        error = getMinErrorForMeasurementFromEstimateList(m, copyofestimate);
        std::printf("\n[ErrorCal] error cal = %f\n", error);
        channel.values.push_back(error);
    }
    //channel should be equal length as the estimate list.
    //fill remaining with zeros. Max error should be 4
    if (!copyofestimate.empty()){
        for(int i = 0; i < copyofestimate.size(); i++){
            channel.values.push_back(1.0); 
        }
    }
}

void Filter::publishData () {
    std::printf("[Publish] Publish data started *************************************************\n");
    sensor_msgs::PointCloud msg/*results*/, msg_to_plotter;
    sensor_msgs::ChannelFloat32 pd_values, ps_values, error_values, pd_Lidar_value, pd_camera_value;
    visualization_msgs::MarkerArray filter_results_in_markers; //for RVIZ
    visualization_msgs::Marker estimate_marker;
    
    pd_Lidar_value.name = "lidar_pd";
    pd_camera_value.name = "camera_pd";
    bool is_synthetic_meas = (measurement_topic_name == "/synthetic_measurements" ? true : false);
    if (is_synthetic_meas) error_values.name = "error_estimateVSMeasurements";
    pd_values.name = "pd_for_each_point-" + std::to_string(PD);
    ps_values.name = "ps_for_each_point-"+ std::to_string(PS);

    std::vector<uri_soft_base::Gaussian> list_of_estimates_to_plot;
    std::vector<double> list_of_weights_to_plot;

    double vector_length = list_of_estimates_.size();
    list_of_estimates_to_plot.reserve(vector_length);
    list_of_weights_to_plot.reserve(vector_length);
    //final publish for plotting
    std::string frameid             = (gazebo_sim_rosbag == true ? "Robot1/odom" : "base_link");
    msg.header.frame_id             = (is_synthetic_meas == true ? "experiment" : frameid);
    msg.header.stamp                = ros::Time::now();
    msg_to_plotter.header           = msg.header;
    estimate_marker.header          = msg.header;
    estimate_marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    estimate_marker.color.b         = (filter_setting == "adaptive" ? 1.0 : 0.0);
    estimate_marker.color.g         = (filter_setting == "adaptive" ? 0.0 : 1.0);
    estimate_marker.color.a         = 1;
    estimate_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
    estimate_marker.lifetime        = ros::Duration(0.8);
    int list_counter                = list_of_estimates_.size();
    for (auto& e : list_of_estimates_) {
        list_counter--;
        //for ploting purpose
        Eigen::Vector2d mean(e.estimate.getMean());
        geometry_msgs::Point32 p;
        p.x = mean.x();
        p.y = mean.y();
        p.z = e.weight;
        double z_for_H = 0.0;
        std::printf("[Publish] estimate X(%f), Y(%f), W(%f)-------------------------\n", p.x, p.y, p.z);
        if(gazebo_sim_rosbag) {
            Eigen::Vector4d point_in_homogForm (mean.x(),mean.y(),0.45,1);
            Eigen::Vector4d tranformed_to_mapframe = homogeneous_tranform_matrix * point_in_homogForm;            
            p.x = tranformed_to_mapframe.x();
            p.y = tranformed_to_mapframe.y();
            z_for_H = tranformed_to_mapframe.z();
        }
        msg_to_plotter.points.push_back(p);
        p.z = (gazebo_sim_rosbag == true) ? z_for_H : 0.0;
        msg.points.push_back(p);

        estimate_marker.text = "w" + std::to_string(e.weight).substr(0, 5);
        float pd = (filter_setting == "Standard" ? this->PD : e.ComputePd());
        pd = (filter_setting == "adaptive_low_pass" ? e.ComputePdMSLH() : pd);
        float ps = (filter_setting == "adaptive" ? e.ComputePs(APSParam) : this->PS);
        pd_values.values.push_back(pd);
        ps_values.values.push_back(ps);

        /*individual pd*/
        auto lidarpd = filter_setting == "Standard" ? PD : e.getCalculatedPDFromBuffers("lidar");
        pd_Lidar_value.values.push_back(lidarpd);
        auto camerapd = filter_setting == "Standard" ? PD : e.getCalculatedPDFromBuffers("camera");
        pd_camera_value.values.push_back(camerapd);
        std::printf("[Publish] filter_setting(%s), lidarpd(%f), camerapd(%f)\n", filter_setting.c_str(), lidarpd, camerapd);
        
        if(EstimateContainer::is_buffer_true) {
            double expected_sensor_miss_detection = std::accumulate(e.buffer_for_lidar_missed_detection_.begin(), e.buffer_for_lidar_missed_detection_.end(), 0.0);
            int pdsum = std::accumulate(e.buffer_for_pd_.begin(), e.buffer_for_pd_.end(), 0);
            int M = e.buffer_for_pd_.size() - pdsum;
           // double pds = e.getSensorPD();
            std::string sensor_pd_details = ", Ps"+ std::to_string(ps).substr(0,4) +
                                       ", TPd"+ std::to_string(pd).substr(0,4) +
                                       ", D"+ std::to_string(pdsum).substr(0,3) +
                                       ", M"+ std::to_string(M).substr(0,2) +
                                       ", C"+std::to_string(camerapd).substr(0,3) + 
                                       ", L" + std::to_string(lidarpd).substr(0,3) ;
            estimate_marker.text = estimate_marker.text + sensor_pd_details;
        }

        estimate_marker.scale.z = (EstimateContainer::isCatabotRosbagActive == true ? 2.0 : 0.09);//2.0;//e.weight/5;
        estimate_marker.pose.position.x = p.x;
        estimate_marker.pose.position.y = p.y;
        estimate_marker.pose.position.z = (filter_setting == "adaptive" ? p.z + 0.2 : p.z + 0.4);
        estimate_marker.ns = std::to_string(list_counter);
        filter_results_in_markers.markers.push_back(estimate_marker);

        /*visulizer variables*/
        list_of_estimates_to_plot.push_back(e.estimate);
        list_of_weights_to_plot.push_back(e.weight);
    }
    msg_to_plotter.channels.push_back(pd_values);
    msg_to_plotter.channels.push_back(ps_values);
    msg_to_plotter.channels.push_back(pd_camera_value);
    msg_to_plotter.channels.push_back(pd_Lidar_value);

    /*camera angle visualization*/
    if (isCameraEnabled) {
        visualization_msgs::Marker cam_angle_line;
        cam_angle_line.header.frame_id     = frameid;//"/surface_cam";
        // if (gazebo_sim_rosbag) cam_angle_line.header.frame_id = "Robot1/camera_link";
        cam_angle_line.header.stamp        = ros::Time::now();
        cam_angle_line.type                = visualization_msgs::Marker::LINE_LIST;
        cam_angle_line.pose.orientation    = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        cam_angle_line.color.b             = 1;
        cam_angle_line.color.g             = 1;
        cam_angle_line.color.a             = 1;
        cam_angle_line.lifetime            = estimate_marker.lifetime;
        cam_angle_line.ns                  = "line";
        cam_angle_line.scale.x             = 0.05;
        geometry_msgs::Point origin,endPoint;
        origin.z = 0.1; origin.x = 0.0; origin.y = 0.0;
        endPoint = origin;
        double length_of_line = 100.0;
        endPoint.x = length_of_line;
        cam_angle_readerLock.lock();
        auto copy_of_camerapoints = camera_measurements_.points; 
        if (!Filter::is_new_camera_meas_avail_) camera_measurements_.points.clear();
        std::printf("[Publish] !Filter::is_new_camera_meas_avail_ (%i) == true -> clear cam meas \n", !Filter::is_new_camera_meas_avail_ );
        cam_angle_readerLock.unlock();
        // cam_angle_line.points.push_back(origin); //line to be drawn from the center
        // auto copy = origin;
        // copy.x = 50;
        // cam_angle_line.points.push_back(copy);
        if (gazebo_sim_rosbag) {
            length_of_line = 10.0;
            Eigen::Vector4d point_in_homogForm (origin.x,origin.y,origin.z,1);
            Eigen::Vector4d tranformed_to_mapframe = homogeneous_tranform_matrix * point_in_homogForm; //translating origin to map 
            origin.x = tranformed_to_mapframe.x();
            origin.y = tranformed_to_mapframe.y();
            origin.z = tranformed_to_mapframe.z();
            for (const auto& pt : copy_of_camerapoints){
                double theta = atan2(pt.y, pt.x);
                point_in_homogForm(0) = length_of_line * std::cos(theta);
                point_in_homogForm(1) = length_of_line * std::sin(theta);
                tranformed_to_mapframe = homogeneous_tranform_matrix * point_in_homogForm;  
                endPoint.x = tranformed_to_mapframe.x();
                endPoint.y = tranformed_to_mapframe.y();
                cam_angle_line.points.push_back(endPoint);
                cam_angle_line.points.push_back(origin); //line to be drawn from the center
            }
        }
        else {
            for (const auto& pt : copy_of_camerapoints){
                double theta = atan2(pt.y, pt.x);
                endPoint.x = length_of_line * std::cos(theta);
                endPoint.y = length_of_line * std::sin(-1*theta);
                cam_angle_line.points.push_back(endPoint);
                cam_angle_line.points.push_back(origin); //line to be drawn from the center
            }
        }
        filter_results_in_markers.markers.push_back(cam_angle_line);
    }

    if (is_synthetic_meas || gazebo_sim_rosbag ) {
        // std::printf("\n[ErrorCal] error calculation\n");
        //calculate error for each robot.max error is 4 since there are 4our robots
        calculateError(error_values);
        msg_to_plotter.channels.push_back(error_values);
    }


    filter_results_.publish(msg);
    f_results_for_plot_.publish(msg_to_plotter);
    rviz_marker.publish(filter_results_in_markers);

    std::printf("[Publish] Publish data End *************************************************\n");
}

EstimateContainer::~EstimateContainer () {
    // delete buffer_for_pd_;
    // buffer_for_pd_.~deque();
}

Filter::~Filter () {
    std::printf("\n[FilterDestructor] =================The filter has terminated================== \n");
}