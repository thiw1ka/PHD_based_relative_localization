#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <mutex>
#include <numeric>
#include <ros/console.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

//Data & message types
#include <deque>
#include <vector>
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "phd_filter_extension/blindspotServiceMsg.h"
#include "utility_functions/utility_functions.hpp"

#include "uri_soft_base/angle_conversion.hpp"
#include "uri_soft_base/gaussian.hpp"


#define rad2deg(rad)( (rad) * 180 / M_PI)
#define deg2rad(deg)( (deg) * M_PI / 180)

#ifndef FILTER_HPP
#define FILTER_HPP

/*
@author thivanka_perera <thiva@uri.edu>
*/

/**
 * @brief Container for a estimate
 * @param estimate mean , covariance
 * @param weight weight of an estimate
 * @param undetected gets incresed by 1 every time that a component derives from the 1-pd term.
 * @param detected gets increased by 1 every time that a component derived frsom the pd * measurement term
 * @param undetected_since_detected reset to 0 when a component derives from the pd*measurment and 1 when derived from 1-pd
 */
struct EstimateContainer {

    uri_soft_base::Gaussian estimate;
    double weight;
    double undetected;
    double detected;
    double undetected_since_last_detected;
    double expected_undetections;
    double mslh_lp;

    /**
     * @brief let filter class access estimate container
     * 
     */
    friend class Filter;

    /**
     * @brief Construct a new Estimate Container object
     * 
     */
    EstimateContainer();

    /**
     * @brief copy Construct a new Estimate Container object
     * 
     * @param original EstimateContainer object to be copied 
     */
    EstimateContainer(const EstimateContainer& original);

    /**
     * @brief increase undetected and undetected_since_last_detected components
     * 
     */
    void IncreaseUndetected();
    
    /**
     * @brief increase deteceted component and reset undetected_since_last_detected = 0 
     * 
     */
    void IncreaseDetected();

    /**
     * @brief calculate Pd based on undetected and detected components
     * 
     * @return double Pd
     */
    double ComputePd();

    /**
     * @brief calculate Ps based on undetected and detected component
     * 
     * @return double Ps
     */
    double ComputePs (const int& APSParam);

    double ComputeA ();

    double ComputePdMSLH ();

    void IncreaseUndetectedLP();

    /**
     * buffer implementation for the PD
    */
    void AddDetectionBuffer(int i);

    /**
     * buffer to accumilate
    */
    // std::shared_ptr <std::deque<int> > buffer_for_pd_;
    std::deque<int> buffer_for_pd_;

    /**
     * @brief buffer for sensor PD
     * 
     */
    std::deque<double> buffer_for_lidar_missed_detection_;

    /**
     * @brief add value to sensor pd buffer
     * 
     */
    void UpdateSensorPDBuffer();

    /**
     * @brief sensor pd;
     * 
     */
    static bool sensor_pd_correction;

    /**
     * @brief buffer size for pd. inside the estimation class
    */
    static int buffer_size;

    /**
     * @brief use buffer values for calculationg PD.
     * if pd_buffer_length > 0 then is_buffer_true = true
     * 
     */
    static bool is_buffer_true;

    /**
     * calcualte pd from thebuffer
    */
    double getPDFromBuffer();

    /**
     * @brief blindspot triangle corner coordinates
     * 
     */
    static geometry_msgs::Point32 b_corners[3];

    /**
     * @brief Get the Sensor pd for a estimate
     * 
     * @return double 
     */
    double getSensorPD (void);

    /**state of measurement update*/

    /**
     * @brief camera measurements are enabled
     * 
     */
    static bool isCameraEnabled;

    /**
     * @brief record camera missed detection for each update
     * based on blind spot or out of FOV
     */
    std::deque<double>  buffer_for_camsensor_missed_detection_;

    /**
     * @brief hits and misses for camera measurement update
     * 
     */
    std::deque <int> buffer_for_pd_camera_;


    /**
     * @brief field of view of camera
     * 
     */
    static double camera_fov_;

    /**
     * @brief camera sensor pd isnid fov
     * 
     */
    static double cam_sensor_pd_;

    /**
     * @brief lidar sensor pd inside fov
     * 
     */
    static double lidar_sensor_pd_;

    /**
     * @brief flag for timeupdate
     * 
     */
    static bool isTimeUpdateActive;

    /**
     * @brief is measurement update is inside the camera 
     * 
     */
    static bool isCameraMeasurementUpdateActive;

    /**
     * @brief to inform if the measurement update is inside the lidar
     * 
     */
    static bool isLidarMeasurementUpdateActive;

    /**
     * @brief is synthetic measurements active 
     * 
     *
    */
    static bool isSyntheticMeasurementsProvided;

    /**
     * @brief flag for gazebo sim enable filter
     * 
     */
    static bool gazebo_sim_rosbag;

    /**
     * @brief range for lidar to be used in sim blindspot.
     * To check a target is outside of the detection range
     */
    static double lidar_range_;

    /**
     * @brief reset sensor buffer with pd=0.75
     * 
     * @param sensor_name camera, lidar
     */
    void resetSensorBuffer(std::string sensor_name);


    double getCalculatedPDFromBuffers (std::string sensor_name);

    /**
     * @brief print estimate mean, weight and cov
     * 
     */
    void print ();

    /**
     * @brief indicate that catabot rosbag is running
     * 
     */
    static bool isCatabotRosbagActive;

    /**
     * @brief Destroy the Estimate Container object
     * 
     */
    ~EstimateContainer();

};

class Filter{ 

    public:

        /**
         * @brief Filter constructor for PHD filter
         * 
         * @param n rosenode handle
         */
        Filter(ros::NodeHandle n);

        /**
         * @brief Destroy the Filter object
         * 
         */
        ~Filter();

        /**
         * @brief flag to inform filter is in the publishing state
         * 
         */
        static bool is_publishing_data;

    private:
        ros::Subscriber odom_, subcriber_for_measurement_, 
                        subcriber_for_measurement_ground_truth, sub_blindspot,
                        subcriber_for_camera_;
        ros::Publisher filter_results_, time_updated_estimates_, f_results_for_plot_, rviz_marker;
        ros::NodeHandle nh_;

        /**
         * @brief freq that is used to create ros rate for the filter
         * 
         */
        int filter_running_speed;

        /**
         * @brief group name space from launch file
         * 
         */
        std::string ns;

        /**
         * @brief filter mode.
         * regular - regular pd and ps
         * adaptive - adaptive pd and ps
         * 
         */
        std::string filter_setting;

        /**
         * @brief topic that contain measurements for the filter
         * 
         */
        std::string measurement_topic_name;

        /**
         * @brief this contains which odom to subscribe
         * 
         */
        std::string odom_topic_name;

        /**
         * @brief counter for measurement callback and odometry callback
         * 
         */
        static int counter_for_measurements, counter_for_odom;

        /**
         * @brief locks avoid simultanious calls for data containers.
         * lock will only allow access to one function at a time
         * used for measurement callback and odometry callback
         */
        std::mutex cloud_lock_, odom_now_lock_, cloud_syntheticGT_lock;//avoid deadlock when same thread locks
        
        //constants
        Eigen::Matrix2d PROCESS_NOISE;
        double sensor_noise; //observation noise covariance
        double PS = 0.95; //probability of survival
        double PD = 0.9;
        double MINIMUM_WEIGHT = 0.001;
        double camera_noise_variance;

        // /**
        //  * @brief visualization object for visualizer class
        //  * 
        //  */
        // std::unique_ptr<OpencvVisualization> visualizePtr_;

        /**
         * @brief states of the filter state machine
         * 
         */
        enum FilterSteps{
            Initializing = 0,
            TimeUpdate = 1,
            MeasurementUpdate = 2,
            PublishData = 3
        };
        int filter_current_step_; // contains the status of filter state machine

        /**
             * buffer size for pd. inside the estimation class
        */
        int pd_buffer_length;

        /**
         * @brief vector to keep all estimates.
         * 
         */
        std::vector<EstimateContainer> list_of_estimates_; //contains the estimates.

        /**
         * @brief initialize the variables for filter ot start
         * 
         */
        void InitStep();

        /**
         * @brief previous timestep rotation matrix calculated in timeupdate function
         * 
         */
        Eigen::Rotation2Dd rot_prev_timestep_;

        /**
         * @brief retain the previous odometry msg.
         * it is used to calculate the previos time step rotation matrix
         * 
         */
        nav_msgs::Odometry previous_odom_copy_;

        /**
         * @brief Time update steps common to all filter types
         * 
         */
        void CommonTimeUpdateSteps();

        /**
         * @brief Get the Time Update object
         * update the estimates with new odometry
         * 
         */
        void GetTimeUpdate();

        /**
         * @brief Time update using adaptive Pd Ps
         * update the estimates with new odometry
         * 
         */
        void GetTimeUpdateUsingAdaptivePs();

        // /**
        //  * @brief Time update use adaptive Pd
        //  * but use regular PS value 
        //  *
        // */
        // void GetTimeUpdateUsingAdaptivePdRegularPs();
        
        /**
         * @brief Get the Measurement Update Lowpass object
         * 
         */
        void GetMeasurementUpdateLowPass();

        /**
         * @brief perfome measurement update
         * use adaptive Pd and Ps
         * update the estimates with lidar measurements
         * 
         */
        void GetMeasurementUpdateUsingAdaptivePd();


        /**
         * @brief perform measurement update using regular ps and pd
         * update the estimates with lidar measurements
         * 
         */
        void GetMeasurementUpdate();

        /**
         * @brief use measurements provided by the camera to update
         * the estimates. Consider FOV of the camera.
         * 
         */
        void GetMeasurementUpdateCamera();

        /**
         * @brief Get the Adaptive Measurement Update for Camera
         * 
        */
        void GetMeasurementUpdateUsingAdaptivePdCamera ();

        /**
         * @brief call back for odometry ros topic
         * 
         * @param msg = nav_msgs::Odometry
         */
        void OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg);

        /**
         * @brief retain a copy of latest odometry msg
         * 
         */
        nav_msgs::Odometry odom_now_;

        /**
         * @brief check if new odometry is avaialble from the callback function
         * 
         */
        bool is_new_odom_available_;

        /**
         * @brief callback from mingi's python script that extract centroid of potential boats.
         * subscribed to "/centroid_label_obstacle"
         * 
         * @param msg sensor_msgs::PointCloud2
         */
        void CentroidLabelObstacleCB(const sensor_msgs::PointCloud2::ConstPtr& msg);

        /**
         * @brief call back function for synthetic measurements topic
         * 
         * @param msg msg type is sensor_msgs::pointcloud
         */
        void CBForSyntheticMesurements (const sensor_msgs::PointCloud::ConstPtr& msg);

        /**
         * @brief 1.retain a copy of measurements received from mingi's script
         *        2. for ground truth
         */
        sensor_msgs::PointCloud cloud_, cloud_syntheticGT_;

        /**
         * @brief check if new measurements are available from the callback CentroidLabelObstacleCB
         * 
         */
        bool is_measurement_available_;

        /**
         * @brief flag for new camera measurement
         * 
         */
        bool is_new_camera_meas_avail_;

        /**
         * @brief calculates the probability density of a point from a given  PD distribution.
         * 
         * @param z measurement
         * @param m mean of the distribution
         * @param cov covariance of the distribution
         * @return double = probability of the given measurement in the given probability density plot
         */
        double getPDensity(Eigen::Vector2d z, Eigen::Vector2d m, Eigen::Matrix2d& cov) const;

        /**
         * @brief calculates the Mahalanobis Distance of two distributions
         * 
         * @param A distribution one
         * @param B distribution two
         * @return double = how many standard deviations away B is from the mean of A
         */
        double GetMahalanobisDistance( uri_soft_base::Gaussian& A, uri_soft_base::Gaussian& B) const;

        /**
         * @brief this function merge estimates that suppost to be nearby and represent same target.
         * 
         */
        void mergedNeareastComponents(std::vector <EstimateContainer>& list_of_estimates);

        /**
         * @brief within this number of standard deviations, estimates are merged
         * 
         */
        int MIN_MD_TO_MERGE;

        /**
         * @brief function pointers to time update and measurment update
         * 
         */
        typedef void(Filter::* FilterFunctionPtr)();

        //timeupdate and measurement update function pointers
        FilterFunctionPtr timeUpdateFunction, measurementUpdateFunction, MeasurementUpdateFunCamera;

        /**
         * @brief publish the relavent msgs
         * 
         */
        void publishData();

        /**
         * @brief calculate error between ground truth and estimates.
        */
        void calculateError(sensor_msgs::ChannelFloat32& channel);

        /**
         * copy of measurements that used for MU function.
         * This is to use in calculateError function to compute error
        */
        sensor_msgs::PointCloud measurement_copy_to_compute_error;

        /**
         * this subcriber is for ground truth of synthetic measurements 
        */
        void CBForSyntheticMesurementsGroundTruth (const sensor_msgs::PointCloud::ConstPtr& msg);

        /**
         * keeping a copy of ground truth IF available for later calculation of error
        */
        void getCopyofGroundTruth (void);
        
        /**
         * @brief adaptive ps calculation parameter
         * 
         */
        int APSParam;

        /**
         * @brief Get the Min Error For Measurement From Estimate List
         * delete the estimate from the list once error calculated
         * 
         * @param m measurement to calculate error
         * @param list_of_estimates vector containing all estimates
         * @return double 
         */
        double getMinErrorForMeasurementFromEstimateList(geometry_msgs::Point32 m, std::vector<EstimateContainer>& list_of_estimates);

        /**
         * @brief client is used to request blindspot triangle points
         * 
         */
        ros::ServiceClient clientForBlindspotRequest;

        /**
         * @brief the corners of the blind spot triangle
         * 
         */
        geometry_msgs::Point32 blindspot_corners[3];

        /**
         * @brief callback function for blind spot service client
         * 
         */
        void requestBlindspotDetails();

        /**
         * @brief callback function for topic that publishers blindspot details
         * 
         * @param msg 
         */
        void CbForBlindSpots (const visualization_msgs::Marker::ConstPtr& msg);

        /**
         * @brief timer will execute the filter start function in a different thread
         * this is a oneshot timer.
         * 
         */
        ros::Timer filter_start;

        /**
         * @brief filter will be execute in this function.
         * 
         * @param e 
         */
        void executeFilter (const ros::TimerEvent& e);

        /**
         * @brief parameter to inform the filter that camera is included in the measurements
         * 
         */
        bool isCameraEnabled;

        /**
         * @brief obtain camera topic name from launch file
         * 
         */
        std::string camera_topic_name;

        /**
         * @brief keep camera measurements when received from topic
         * 
         */
        sensor_msgs::PointCloud camera_measurements_;

        /**
         * @brief mutex lock for camera measurement lock
         * 
         */
        std::mutex cam_angle_readerLock;

        // /**
        //  * @brief flag for new message
        //  * 
        //  */
        // bool isCameraNewMessageAvailable;

        /**
         * @brief camera params
         * 
         */
        double camera_fov;
        double cam_resol_x; //image resolution x width
        double cam_resol_y; //image resolution x width

        /**
         * @brief callback for camera measurements receive from gazebo
         * 
         * @param msg 
         */
        void CbForCameraMeasGazebo (const geometry_msgs::PoseArray::ConstPtr& msg);

        /**
         * @brief flag for gazebo simulation
         * 
         */
        bool gazebo_sim_rosbag;

        /**
         * @brief for gazebo sim. transform estimates back to map frame.
         * it will easy calculate the error
         */
        static Eigen::Matrix4d homogeneous_tranform_matrix;

        /**
         * @brief one time odom initialization
         * 
         */
        static bool is_odom_init;

        void CbForGazeboGroundTruth(const gazebo_msgs::ModelStates::ConstPtr& msg);
        
        /**
         * @brief covariance for birth targets that introduced in time update.
         * Matrix2d::identity() * Birth_covariance
         * variance = SD^2
         * 
         */
        double birth_target_cov;
};

#endif