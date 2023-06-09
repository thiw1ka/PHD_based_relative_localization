#include "utility_functions/utility_functions.hpp"




double utility_functions::calculateTriangleArea (const geometry_msgs::Point32& corner1, 
                                                    const geometry_msgs::Point32& corner2, 
                                                            const geometry_msgs::Point32& pt ){
    //area of an triangle using 3 coordinate points
    return 0.5*(abs(corner1.x*(corner2.y-pt.y)+corner2.x*(pt.y-corner1.y)+pt.x*(corner1.y-corner2.y)));

}

bool utility_functions::isPointInsideBlindSpot (const geometry_msgs::Point32& pt, 
                                                geometry_msgs::Point32 blindspot_corners[3]) {

    double blind_spot_area = calculateTriangleArea (blindspot_corners[0], blindspot_corners[1], blindspot_corners[2]);
    double area_to_match = calculateTriangleArea(blindspot_corners[0], blindspot_corners[1], pt);
    area_to_match +=  calculateTriangleArea(blindspot_corners[0], blindspot_corners[2], pt);
    area_to_match +=  calculateTriangleArea(blindspot_corners[1], blindspot_corners[2], pt);
    // std::printf("[utility] area of triangle %f, are of the point %f \n", trunc(blind_spot_area * 100), trunc(area_to_match * 100));
    return ( trunc(blind_spot_area * 100) == trunc(area_to_match * 100)); // accurate for 1 cm

}

bool utility_functions::isPointInsideCameraFov (const double& fov/*deg*/, const geometry_msgs::Point32& point) {
        double bearing_of_estimate = std::atan2(point.y, point.x);
        static double fov_rad = fov * M_PI / 180;
        static auto min_fov = -1 * fov_rad / 2;
        static auto max_fov = fov_rad / 2;
        // std::printf("[isPointInsideCameraFov] bearing_of_estimate(%f) = std::atan(%f, %f), fov_rad(%f)=fov(%f)*M_PI/180,  min_fov(%f), max_fov(%f) \n",
        // bearing_of_estimate, point.y, point.x, fov_rad, fov, min_fov, max_fov);
        std::printf("[isPointInsideCameraFov] bearing_of_estimate(%f) >= min_fov(%f) &&  bearing_of_estimate(%f) <= max_fov(%f) : return: (%i) \n",
        bearing_of_estimate, min_fov,bearing_of_estimate, max_fov,(bearing_of_estimate >= min_fov &&  bearing_of_estimate <= max_fov));
        if (bearing_of_estimate >= min_fov &&  bearing_of_estimate <= max_fov) return true;
        else return false;
}

bool utility_functions::isPointInsideLidarRange (const double& range, const geometry_msgs::Point32& pt){
        auto hypotenus = std::hypot(pt.x, pt.y);
        // std::printf("[pd sim] hypot : %f \n", hypotenus);
        if (hypotenus > range) return false; //point lies outside of the range
        else return true;
}

bool utility_functions::InsideBoxCal::isPointInsideBox (const geometry_msgs::Point32& pt){

        static double boxlength = std::hypot((corners_of_rectangle[0].x - corners_of_rectangle[1].x), 
                                                (corners_of_rectangle[0].y - corners_of_rectangle[1].y));
        static double boxwidth = std::hypot((corners_of_rectangle[1].x - corners_of_rectangle[2].x), 
                                                (corners_of_rectangle[1].y - corners_of_rectangle[2].y));
        static double area = boxlength * boxwidth;

        double areacalculated = 0.0;
        areacalculated += calculateTriangleArea(corners_of_rectangle[0], corners_of_rectangle[1], pt);
        areacalculated += calculateTriangleArea(corners_of_rectangle[1], corners_of_rectangle[2], pt);
        areacalculated += calculateTriangleArea(corners_of_rectangle[2], corners_of_rectangle[3], pt);
        areacalculated += calculateTriangleArea(corners_of_rectangle[0], corners_of_rectangle[3], pt);

        if (trunc(area) == trunc(areacalculated)) return true;
        else return false;
}

bool utility_functions::InsideBoxCal::isPointInsideBox (const Eigen::Matrix4d& homogeneous_matrix, 
                                                        const geometry_msgs::Point32& pt) {
        Eigen::Vector4d pointInEigen (pt.x, pt.y, pt.z, 1);
        // pointInEigen[0] = pt.x;
        // pointInEigen[1] = pt.y;
        // pointInEigen[2] = pt.z;
        // tf2::fromMsg(pt, pointInEigen);
        Eigen::Vector4d  translateToWorld = homogeneous_matrix * pointInEigen;
        geometry_msgs::Point32 translated_point;
        translated_point.x = translateToWorld.x();
        translated_point.y = translateToWorld.y();
        translated_point.z = translateToWorld.z();
        return isPointInsideBox (translated_point);
}

//static point init
geometry_msgs::Point32 utility_functions::InsideBoxCal::corners_of_rectangle [4] = {};
bool utility_functions::OdomTranslaterToWorld::isFirstOdomArrived = false;
// utility_functions::homogeneous_tranform_matrix = Eigen::Matrix4d::Identity();


void utility_functions::OdomTranslaterToWorld::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard <std::recursive_mutex> lg (odom_lk_);
        nav_msgs::Odometry odom_copy = *msg;
        if (!isFirstOdomArrived) {
                previous_odom_copy_ = *msg;
                Eigen::Quaterniond prior_quat;
                tf2::fromMsg(previous_odom_copy_.pose.pose.orientation, prior_quat);
                prior_quat.normalize();
                homogeneous_tranform_matrix.topLeftCorner (3,3) =  prior_quat.toRotationMatrix();
                // std::cout << "H tranform \n" <<homogeneous_tranform_matrix <<std::endl;
        }

        Eigen::Vector3d prev_timestep_pos, current_timestep_pos;
        tf2::fromMsg(odom_copy.pose.pose.position, current_timestep_pos); //current pos is coming in map frame
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
        homogeneous_tranform_matrix.topLeftCorner (3,3) =  relative_rot.toRotationMatrix() * homogeneous_tranform_matrix.topLeftCorner (3,3);
        homogeneous_tranform_matrix.topRightCorner (3,1) = current_timestep_pos;

}

// void utility_functions::printBuffer (const std::deque<>)