#include <iostream>
#include <deque>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#ifndef UTILITY_FUNCTIONS_HPP
#define UTILITY_FUNCTIONS_HPP

namespace utility_functions {
    /**
     * @brief calculate area of a triangle
     * 
     * @param corner1 
     * @param corner2 
     * @param pt 
     * @return double 
     */
    double calculateTriangleArea (const geometry_msgs::Point32& corner1, 
                                            const geometry_msgs::Point32& corner2, 
                                            const geometry_msgs::Point32& pt );

    /**
     * @brief calculate if a point is inside a box
     * 
     */
    struct InsideBoxCal {
        InsideBoxCal () {};

        static geometry_msgs::Point32 corners_of_rectangle [4];

        bool isPointInsideBox (const geometry_msgs::Point32& pt);

        bool isPointInsideBox (const Eigen::Matrix4d& homogeneous_matrix, const geometry_msgs::Point32& pt);
    };

    /**
     * @brief is point inside triangle blindspot
     * 
     * @param pt 
     * @param blindspot_corners 
     * @return true 
     * @return false 
     */
    bool isPointInsideBlindSpot (const geometry_msgs::Point32& pt, 
                                geometry_msgs::Point32 blindspot_corners[3]);

    /**
     * @brief Print buffers that containe pd
     * 
     * @tparam t 
     * @param buff_name 
     * @param buff 
     */
    template <typename t>
    void printBuffer (std::string buff_name, const std::deque<t>& buff) {
        std::cout << buff_name << " :";
        for (auto i : buff) {
            std::cout << i << ",";
        }
        std::cout<<std::endl;
    };

    /**
     * @brief Get the Bearing For targets from a camera image.
     * https://stackoverflow.com/questions/18092855/finding-a-pixel-location-in-a-field-of-view
     * 
     * @tparam t 
     * @param x x coordniate x = point - imageresolution_Horizontal / 2
     * @param focallengthInPixel = image_resolution_horizontal / tan(fov_of_camera)
     * @return t 
     */
    template <typename t>
    inline t getBearingForCameraTargets(const t& x, const t& focallengthInPixel) {
        // double focalLengthPx = 557.0;
        t bearing = std::atan2(x, focallengthInPixel);
        return bearing * 180 / M_PI; // degrees
    }

    template <typename t1, typename t2>
    double getProbDensityVal (const t1& z, const t1& m, const t2& cov) {
        
    };

    /**
     * @brief if a given point is inside the camera field of view
     * 
     * @param fov 
     * @return true 
     * @return false 
     */
    bool isPointInsideCameraFov (const double& fov, const geometry_msgs::Point32&);

    /**
     * 
     * @brief check if the point is inside the lidar range
     * 
     * @param range 
     * @param pt 
     * @return true 
     * @return false 
     */
    bool isPointInsideLidarRange (const double& range, const geometry_msgs::Point32& pt);

    /**
     * @brief check if a point is inside a given circle
     * 
     * @param center center point
     * @param rad radius of the circle
     * @param pt point to be check
     * @return true - point is inside
     * @return false  - point is outside
     */
    template <typename t>
    bool isPointInsideCircle (const t& center, double rad, const t& pt) {
        double distance_point_to_center = std::hypot ((pt.x - center.x), (pt.y - center.y));
        std::printf("[UtilFun] isPtInsideCircle :%i, cirX:%f, cirY:%f, PtX:%f, PtY:%f, dist(hypot(cirX-PtX, cirY-ptY)): %f, radius: %f \n",
                                    int((distance_point_to_center < rad)), center.x, center.y, pt.x, pt.y, distance_point_to_center, rad);
        if (distance_point_to_center < rad) return true; //point is inside
        else return false; //point is outside
    }


    /**
     * @brief This struct continously calculate the homogeneous tranform matrix from odom
     * odom has to provide pos in world frame
     * 
     */
    struct OdomTranslaterToWorld {

        OdomTranslaterToWorld () {
            Eigen::Matrix4d homogeneous_tranform_matrix = Eigen::Matrix4d::Identity();
            };

        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

        Eigen::Matrix4d homogeneous_tranform_matrix;

        static bool isFirstOdomArrived;

        nav_msgs::Odometry previous_odom_copy_;

        std::recursive_mutex odom_lk_;

        Eigen::Matrix4d get_homog () {
            std::lock_guard <std::recursive_mutex> lg (odom_lk_);
            return homogeneous_tranform_matrix;
        }
    };
};


#endif