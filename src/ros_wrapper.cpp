#include "ros_wrapper.h"
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

ROSWrapper::ROSWrapper(ros::NodeHandle& n, QObject *parent) : QThread(parent)
{
    image_sub = n.subscribe("/camera/rgb/image_rect_color", 1, &ROSWrapper::image_callback, this);
}

void ROSWrapper::run()
{
    ros::Rate rate(25);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    emit gotQuit();
}

void ROSWrapper::image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr mat_cv_rgb = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
    QImage image = QImage((uchar*) mat_cv_rgb->image.data, mat_cv_rgb->image.cols, mat_cv_rgb->image.rows, mat_cv_rgb->image.step, QImage::Format_RGB888).copy();
    emit gotFrame(image);
}
