/**
 * @file    ros_wrapper.h
 * @author  Daniel Koguciuk (daniel.koguciuk@gmail.com)
 * @date    June, 2016
 * @brief   This is ROS wrapper class extending QThread class. It is used only to
 *          catch openni camera color frame. One can catch gotFrame signal with
 *          received image.
 */

#ifndef ROS_WRAPPER_H
#define ROS_WRAPPER_H

#include <QImage>
#include <QThread>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class ROSWrapper : public QThread
{
    Q_OBJECT

public:

    /**
     * @brief ROSWrapper        Default constructor of the class requires ROS node handle.
     * @param n                 ROS node handle.
     * @param parent            Parent of the class.
     */
    explicit ROSWrapper(ros::NodeHandle& n, QObject *parent = 0);

protected:

    /**
     * @brief run               Override of the virtual method of the QThread class (infinite
     *                          ROS loop here).
     */
    void run();

private:

    /**
     * @brief image_sub         ASUS Xtion image subscriber.
     */
    ros::Subscriber image_sub;

    /**
     * @brief image_callback    ROS callback with the image from the sensor.
     * @param msg               Image received from the sensor.
     */
    void image_callback(const sensor_msgs::ImageConstPtr &msg);

signals:

    /**
     * @brief gotFrame          Qt signal with received image.
     * @param image             QImage (640x480xRGB).
     */
    void gotFrame(const QImage &image);

    /**
     * @brief gotQuit           Qt signal with quit information.
     */
    void gotQuit();

};

#endif // ROS_WRAPPER_H
