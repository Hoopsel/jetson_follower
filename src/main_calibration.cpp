#include <qt4/Qt/qapplication.h>
#include <ros/ros.h>
#include "calibration.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "forbot_calibration");
    ros::NodeHandle n("~");

    QApplication a(argc, argv);
    Calibration calibration(n);
    calibration.show();

    return a.exec();
}
