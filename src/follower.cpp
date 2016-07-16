#include "follower.h"
#include "gpu_utils.cuh"

#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/chrono.hpp>

#define ITERATIONS  100
#define DEBUG       1

Follower::Follower(NodeHandle &n)
{
    //==================================================================
    //============================ HSV params ==========================
    //==================================================================

    YAML::Node params = YAML::LoadFile((string(PROJECT_SOURCE_DIR) + string("/config/hsv.yaml")).c_str());

    bool valid = true;
    if (params["h_swap"])
    {
        h_swap = params["h_swap"].as<bool>();
    } else valid = false;

    if (params["h_min"])
    {
        h_min = params["h_min"].as<int>();
    } else valid = false;

    if (params["h_max"])
    {
        h_max = params["h_max"].as<int>();
    } else valid = false;

    if (params["s_min"])
    {
        s_min = params["s_min"].as<int>();
    } else valid = false;

    if (params["s_max"])
    {
        s_max = params["s_max"].as<int>();
    } else valid = false;

    if (params["v_min"])
    {
        v_min = params["v_min"].as<int>();
    } else valid = false;

    if (params["v_max"])
    {
        v_max = params["v_max"].as<int>();
    } else valid = false;

    if (!valid)
    {
        cout << "Invalid yamlfile. Please use forbot_hsv node one more time to generate valid params." << endl;
        exit(-1);
    }

    //==================================================================
    //============================== Images ============================
    //==================================================================

    image_color = nullptr;
    image_depth = nullptr;

    //==================================================================
    //============================ Robot name ==========================
    //==================================================================

    std::string ns = n.getNamespace();
    std::string parent_ns = ros::names::parentNamespace(ns);

    //==================================================================
    //========================= Sub & Publishers =======================
    //==================================================================

    sub_depth =  n.subscribe<sensor_msgs::Image>("/camera/depth/image_rect", 1, &Follower::callbackDepth, this);
    sub_color =  n.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1, &Follower::callbackColor, this);
}

void Follower::callbackDepth(const sensor_msgs::ImageConstPtr &image)
{
    image_depth = image;
}
void Follower::callbackColor(const sensor_msgs::ImageConstPtr &image)
{
    image_color = image;
}

Follower::Position Follower::findMarker(bool using_gpu)
{
    //==================================================================
    //=========================== No messages ==========================
    //==================================================================

    static Time start = Time::now();
    if (image_color == nullptr)
    {
        if ((Time::now() - start).sec > 5)
        {
            cout << "No messages received from color image." << endl;
            exit(-1);
        }
        return make_pair(0, -1);
    }

    if (image_depth == nullptr)
    {
        if ((Time::now() - start).sec > 5)
        {
            cout << "No messages received from depth image." << endl;
            exit(-1);
        }
        return make_pair(0, -1);
    }

    //==================================================================
    //=================== Threshold HSV (CPU or GPU?) ==================
    //==================================================================

    boost::chrono::high_resolution_clock::time_point time_start = boost::chrono::high_resolution_clock::now();

    Mat image_hsv_thr;
    if (using_gpu)
    {
        convertAndThreshold(image_hsv_thr, image_color, h_swap, h_min, h_max, s_min, s_max, v_min, v_max);
        if (DEBUG) imshow("ColorThresholded", image_hsv_thr);
    } else
    {
        cv_bridge::CvImagePtr mat_color = cv_bridge::toCvCopy(*image_color, sensor_msgs::image_encodings::BGR8);
        if (DEBUG) imshow("Color", mat_color->image);

        cv::Mat image_hsv;
        image_hsv_thr = Mat(mat_color->image.rows, mat_color->image.cols, CV_8UC1);
        cv::cvtColor(mat_color->image, image_hsv, CV_BGR2HSV);
        image_hsv_thr.setTo(0);
        for (int y=0; y<image_hsv.rows; ++y)
            for (int x=0; x<image_hsv.cols; ++x)
            {
                cv::Vec3b pixel = image_hsv.at<cv::Vec3b>(y, x);
                if (pixel[1] >= s_min && pixel[1] <= s_max &&
                    pixel[2] >= v_min && pixel[2] <= v_max)
                {
                    if (!h_swap && pixel[0] >= h_min && pixel[0] <= h_max)
                        image_hsv_thr.at<uchar>(y,x) = 255;

                    if (h_swap && (pixel[0] < h_min || pixel[0] > h_max))
                        image_hsv_thr.at<uchar>(y,x) = 255;
                }
            }
        if (DEBUG) imshow("ColorThresholded", image_hsv_thr);
    }

    boost::chrono::high_resolution_clock::time_point time_stop = boost::chrono::high_resolution_clock::now();
    if(ITERATIONS > 0)
    {
        static int i=0;
        static float sum=0;
        if(i<ITERATIONS)
        {
            sum+=(float)(time_stop - time_start).count()/1000000;
            ++i;
        }
        if(i==ITERATIONS)
        {
            if (using_gpu) cout << "Parallel time = ";
            else cout << "Serial time = ";
            cout << sum/ITERATIONS << endl;
            ++i;
        }
    }

    //==================================================================
    //=========================== Filter THSV ==========================
    //==================================================================

    // Median filter 5x5
    cv::Mat image_hsv_th_fil;
    medianBlur(image_hsv_thr, image_hsv_th_fil, 5);

    // Closing 5x5
    Mat element = getStructuringElement(MORPH_RECT, Size(5,5), Point(3,3));
    morphologyEx(image_hsv_th_fil, image_hsv_th_fil, MORPH_CLOSE, element);
    if (DEBUG) imshow("ColorMorph", image_hsv_th_fil);

    //==================================================================
    //========================= Biggest contour ========================
    //==================================================================

    // Find biggest contour
    int max_index = 0;
    vector<vector<Point> > contours;
    findContours(image_hsv_th_fil, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    if (!contours.size()) return make_pair(0, -1);
    for( int i = 1; i< contours.size(); i++ )
        if(contourArea(contours.at(max_index)) < contourArea(contours.at(i))) max_index = i;

    // Find a center of the biggest contour
    if (contours.at(max_index).size() < 25) return make_pair(0, -1);
    Moments mom = moments(contours.at(max_index));
    int x = round(mom.m10/mom.m00);
    int y = round(mom.m01/mom.m00);

    // Draw contours
    Mat drawing = Mat::zeros( image_hsv_th_fil.size(), CV_8UC1 );
    for( int i = 0; i< contours.size(); i++ )
        if (i != max_index) drawContours(drawing, contours, i, 128, 2, 8);
        else drawContours(drawing, contours, i, 255, 2, 8);
    if (DEBUG)
    {
        circle(drawing, Point(x,y), 50, Scalar(255));
        imshow("Contur", drawing);
    }

    //==================================================================
    //==================== Get distance to the marker ==================
    //==================================================================

    // Find distance to the marker
    cv_bridge::CvImagePtr mat_depth = cv_bridge::toCvCopy(*image_depth, sensor_msgs::image_encodings::TYPE_32FC1);
    int rect_size = 5;                                                      // size of rect
    Rect rect(x - rect_size/2, y-rect_size/2, rect_size, rect_size);        // Rect
    Mat rected(mat_depth->image,rect);                                      // Rected
    cv::Mat mask = cv::Mat(rected == rected);                               // NaN mask
    Scalar mean_v = mean(rected, mask);                                     // Mean
    if (sum(mask) == Scalar(0))
    {
        cout << "All NaNs!" << endl;
        return make_pair(0, -1);
    }

    // Display
    if (DEBUG)
    {
        normalize(mat_depth->image, mat_depth->image, 0, 1, cv::NORM_MINMAX);
        imshow("Depth?", mat_depth->image);
        waitKey(30);
    }

    //==================================================================
    //====================== Return the Error struct ===================
    //==================================================================

    // Calculate final error
    Error error = ((Error)(x - image_hsv_th_fil.cols/2))/(image_hsv_th_fil.cols/2);
    return  make_pair(error, mean_v[0]);
}
