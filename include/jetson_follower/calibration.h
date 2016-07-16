/**
 * @file    calibration.h
 * @author  Daniel Koguciuk (daniel.koguciuk@gmail.com)
 * @date    June, 2016
 * @brief   This is main calibration class. One can set all HSV ranges with online
 *          preview. The result is saved to file in config directory.
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <QMainWindow>
#include <ros/ros.h>
#include <ros_wrapper.h>

namespace Ui {
class Calibration;
}

class Calibration : public QMainWindow
{
    Q_OBJECT

public:

    /**
     * @brief Calibration           Default constructor of the class requires ROS node handle.
     * @param n                     ROS node handle.
     * @param parent                Parent of the class.
     */
    explicit Calibration(ros::NodeHandle &n, QWidget *parent = 0);

    /**
     * @brief ~Calibration          Default destructor of the class.
     */
    ~Calibration();

private slots:

    /**
     * @brief onFrame               Qt slot catching an image from the sensor.
     * @param qimage                QImage (640x480xRGB).
     */
    void onFrame(const QImage &qimage);

    /**
     * @brief onQuit                Qt slot catching quit signal.
     */
    void onQuit();

    /**
     * @brief on_horizontalSlider_valueChanged      Qt slot catching h_min value change.
     * @param value                                 New value.
     */
    void on_horizontalSlider_valueChanged(int value);

    /**
     * @brief on_horizontalSlider_2_valueChanged    Qt slot catching h_max value change.
     * @param value                                 New value.
     */
    void on_horizontalSlider_2_valueChanged(int value);

    /**
     * @brief on_horizontalSlider_3_valueChanged    Qt slot catching s_min value change.
     * @param value                                 New value.
     */
    void on_horizontalSlider_3_valueChanged(int value);

    /**
     * @brief on_horizontalSlider_4_valueChanged    Qt slot catching s_max value change.
     * @param value                                 New value.
     */
    void on_horizontalSlider_4_valueChanged(int value);

    /**
     * @brief on_horizontalSlider_5_valueChanged    Qt slot catching v_min value change.
     * @param value                                 New value.
     */
    void on_horizontalSlider_5_valueChanged(int value);

    /**
     * @brief on_horizontalSlider_6_valueChanged    Qt slot catching v_max value change.
     * @param value                                 New value.
     */
    void on_horizontalSlider_6_valueChanged(int value);

    /**
     * @brief on_radioButton_clicked                Qt slot catching h_swap option.
     * @param checked                               h_swap option checked or not?
     */
    void on_radioButton_clicked(bool checked);

    /**
     * @brief on_pushButton_clicked                 Qt slot catching save demand.
     */
    void on_pushButton_clicked();

private:

    /**
     * @brief ui                User interface class.
     */
    Ui::Calibration *ui;

    /**
     * @brief wrapper           ROS wrapper class object.
     */
    ROSWrapper *wrapper;

    /**
     * @brief h_swap            Should we swap h?
     */
    bool h_swap;

    /**
     * @brief h_min             h_min value.
     */
    int h_min;

    /**
     * @brief h_max             h_max value.
     */
    int h_max;

    /**
     * @brief s_min             s_min value.
     */
    int s_min;

    /**
     * @brief s_max             s_max value.
     */
    int s_max;

    /**
     * @brief v_min             v_min value.
     */
    int v_min;

    /**
     * @brief v_max             v_max value.
     */
    int v_max;
};

#endif // CALIBRATION_H
