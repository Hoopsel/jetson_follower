/**
 * @file    follower.h
 * @author  Daniel Koguciuk (daniel.koguciuk@gmail.com)
 * @date    June, 2016
 * @brief   This is main calibration class. One can set all HSV ranges with online
 *          preview. The result is saved to file in config directory.
 */

#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>


using namespace ros;
using namespace std;
using namespace cv;


class Follower
{

public:

    typedef float Error;                        // Normalized -1 (left) : 1 (right)
    typedef float Distance;                     // Above 0
    typedef pair<Error, Distance> Position;     // For futher convenience

    /**
     * @brief Follower          Default constructor of the class requires ROS node handle.
     * @param n                 ROS node handle.
     */
    Follower(NodeHandle& n);

    /**
     * @brief findMarkerCPU     The core of our class - method finding markers position.
     * @param[in] using gpu     Param specyfying if gpu is used (true) or not (false).
     * @return                  Position of the marker.
     */
    Position findMarker(bool using_gpu);

private:

    // =============================================================================
    // ============================= ROS SUBSCRIBERS ===============================
    // =============================================================================

    /**
     * @brief sub_depth         Depth image subscriber.
     */
    Subscriber sub_depth;

    /**
     * @brief sub_color         Color image subscriber.
     */
    Subscriber sub_color;

    // =============================================================================
    // ============================== ROS CALLBACKS ================================
    // =============================================================================

    /**
     * @brief callbackDepth     Depth image callback.
     * @param image             Pointer to the depth image.
     */
    void callbackDepth(const sensor_msgs::ImageConstPtr &image);

    /**
     * @brief callbackColor     Color image callback.
     * @param image             Pointer to the color image.
     */
    void callbackColor(const sensor_msgs::ImageConstPtr &image);

    // =============================================================================
    // =================================== IMAGES ==================================
    // =============================================================================

    /**
     * @brief image_depth       Depth image pointer.
     */
    sensor_msgs::ImageConstPtr image_depth;

    /**
     * @brief image_color       Color image pointer.
     */
    sensor_msgs::ImageConstPtr image_color;

    // =============================================================================
    // ================================== HSV PARAMS ===============================
    // =============================================================================

    /**
     * @brief h_swap            H range swap (h is cyclic)
     */
    bool h_swap;

    /**
     * @brief h_min             Value of h_min.
     */
    int h_min;

    /**
     * @brief h_max             Value of h_max.
     */
    int h_max;

    /**
     * @brief s_min             Value of s_min.
     */
    int s_min;

    /**
     * @brief s_max             Value of s_max.
     */
    int s_max;

    /**
     * @brief v_min             Value of v_min.
     */
    int v_min;

    /**
     * @brief v_max             Value of v_max.
     */
    int v_max;
};

#endif // FOLLOWER_H
