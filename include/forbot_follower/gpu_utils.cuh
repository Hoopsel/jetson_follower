/**
 * @file    gpu_utils.cuh
 * @author  Daniel Koguciuk (daniel.koguciuk@gmail.com)
 * @date    June, 2016
 * @brief   This is module for all GPU methods - you can simply call the method
 *          like it was normal serial function.
 */

#ifndef GPU_UTILS_CUH
#define GPU_UTILS_CUH

#include <stdio.h>

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

void convertAndThreshold(cv::Mat &mat, sensor_msgs::ImageConstPtr &image, bool h_swap,
                         int h_min, int h_max, int s_min, int s_max, int v_min, int v_max);

#endif // GPU_UTILS_CUH
