#include "calibration.h"
#include "ui_calibration.h"

#include <QLabel>
#include <QSlider>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace std;

Calibration::Calibration(ros::NodeHandle &n, QWidget *parent) : QMainWindow(parent), ui(new Ui::Calibration)
{
    ui->setupUi(this);

    ui->horizontalSlider->setValue(38);
    ui->horizontalSlider_2->setValue(255);
    ui->horizontalSlider_3->setValue(0);
    ui->horizontalSlider_4->setValue(255);
    ui->horizontalSlider_5->setValue(0);
    ui->horizontalSlider_6->setValue(255);

    on_horizontalSlider_valueChanged(38);
    on_horizontalSlider_2_valueChanged(255);
    on_horizontalSlider_3_valueChanged(0);
    on_horizontalSlider_4_valueChanged(255);
    on_horizontalSlider_5_valueChanged(0);
    on_horizontalSlider_6_valueChanged(255);

    // ================================ TEMP ========================================
//    XmlRpc::XmlRpcValue params("ros_topic_list");
//    XmlRpc::XmlRpcValue results;
//    XmlRpc::XmlRpcValue r;
//    vector<string> topics;
//    if(ros::master::execute("getTopicTypes", params, results, r, false) == true)
//    {
//        if(results.getType() == XmlRpc::XmlRpcValue::TypeArray)
//        {
//            int32_t i = 2;
//            if(results[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
//            {
//                for (int32_t j = 0; j < results[i].size(); ++j)
//                {
//                    if(results[i][j].getType() == XmlRpc::XmlRpcValue::TypeArray)
//                    {
//                        if(results[i][j].size() == 2)
//                        {
//                            if(results[i][j][0].getType() == XmlRpc::XmlRpcValue::TypeString &&
//                               results[i][j][1].getType() == XmlRpc::XmlRpcValue::TypeString)
//                            {
//                                std::string topic = static_cast<std::string>(results[i][j][0]);
//                                std::string type = static_cast<std::string>(results[i][j][1]);
//                                if (type == "sensor_msgs/Image")
//                                topics.push_back(topic);
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
    // ==============================================================================

    h_swap = false;
    wrapper = new ROSWrapper(n);
    connect(wrapper, SIGNAL(gotFrame(const QImage&)), this, SLOT(onFrame(const QImage&)));
    connect(wrapper, SIGNAL(gotQuit()), this, SLOT(onQuit()));
    wrapper->start();
}
Calibration::~Calibration()
{
    delete wrapper;
    delete ui;
}

void Calibration::onFrame(const QImage &qimage)
{
    // Create opencv mat
    cv::Mat image_rgb(qimage.size().height(),
                      qimage.size().width(),
                      CV_8UC3,
                      const_cast<uchar*>(qimage.bits()),
                      qimage.bytesPerLine());

    // Convert to HSV
    cv::Mat image_hsv, image_vis(image_rgb.rows, image_rgb.cols, image_rgb.type());
    cv::cvtColor(image_rgb, image_hsv, CV_RGB2HSV);
    image_vis.setTo(cv::Vec3i(0,0,0));
    for (int y=0; y<image_hsv.rows; ++y)
        for (int x=0; x<image_hsv.cols; ++x)
        {
            cv::Vec3b pixel = image_hsv.at<cv::Vec3b>(y, x);
            if (pixel[1] >= s_min && pixel[1] <= s_max &&
                pixel[2] >= v_min && pixel[2] <= v_max)
            {
                if (!h_swap && pixel[0] >= h_min && pixel[0] <= h_max)
                    image_vis.at<cv::Vec3b>(y,x) = cv::Vec3b(255,255,255);

                if (h_swap && (pixel[0] < h_min || pixel[0] > h_max))
                    image_vis.at<cv::Vec3b>(y,x) = cv::Vec3b(255,255,255);
            }
        }

    // Show image
    QImage qimage_vis = QImage((uchar*) image_vis.data, image_vis.cols, image_vis.rows, image_vis.step, QImage::Format_RGB888).copy();
    ui->label->setPixmap(QPixmap::fromImage(qimage_vis));
}
void Calibration::onQuit()
{
    wrapper->quit();
    this->close();
}

void Calibration::on_horizontalSlider_valueChanged(int value)
{
    h_min = value;
    if (value<10) ui->label_8->setText(QString::number(0) + QString::number(value));
    else ui->label_8->setText(QString::number(value));
}
void Calibration::on_horizontalSlider_2_valueChanged(int value)
{
    h_max = value;
    if (value<10) ui->label_9->setText(QString::number(0) + QString::number(value));
    else ui->label_9->setText(QString::number(value));
}
void Calibration::on_horizontalSlider_3_valueChanged(int value)
{
    s_min = value;
    if (value<10) ui->label_10->setText(QString::number(0) + QString::number(value));
    else ui->label_10->setText(QString::number(value));
}
void Calibration::on_horizontalSlider_4_valueChanged(int value)
{
    s_max = value;
    if (value<10) ui->label_11->setText(QString::number(0) + QString::number(value));
    else ui->label_11->setText(QString::number(value));
}
void Calibration::on_horizontalSlider_5_valueChanged(int value)
{
    v_min = value;
    if (value<10) ui->label_12->setText(QString::number(0) + QString::number(value));
    else ui->label_12->setText(QString::number(value));
}
void Calibration::on_horizontalSlider_6_valueChanged(int value)
{
    v_max = value;
    if (value<10) ui->label_13->setText(QString::number(0) + QString::number(value));
    else ui->label_13->setText(QString::number(value));
}
void Calibration::on_radioButton_clicked(bool checked)
{
    h_swap = checked;
}

void Calibration::on_pushButton_clicked()
{
    YAML::Emitter yaml_emitter;
    yaml_emitter << YAML::BeginMap;

    yaml_emitter << YAML::Key << "h_swap";
    yaml_emitter << YAML::Value << h_swap;

    yaml_emitter << YAML::Key << "h_min";
    yaml_emitter << YAML::Value << h_min;

    yaml_emitter << YAML::Key << "h_max";
    yaml_emitter << YAML::Value << h_max;

    yaml_emitter << YAML::Key << "s_min";
    yaml_emitter << YAML::Value << s_min;

    yaml_emitter << YAML::Key << "s_max";
    yaml_emitter << YAML::Value << s_max;

    yaml_emitter << YAML::Key << "v_min";
    yaml_emitter << YAML::Value << v_min;

    yaml_emitter << YAML::Key << "v_max";
    yaml_emitter << YAML::Value << v_max;

    ofstream file((string(PROJECT_SOURCE_DIR) + string("/config/hsv.yaml")).c_str());
    file << yaml_emitter.c_str();
    file.close();
}
