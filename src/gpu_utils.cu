#include "gpu_utils.cuh"

__global__ void gpu_rgb2hsv(uchar* data, int width, int height)
{
    int index_x = blockIdx.x*blockDim.x+threadIdx.x;
    int index_y = blockIdx.y*blockDim.y+threadIdx.y;
    int index = 3*(index_y*width + index_x);

    if (index_x<width && index_y<height)
    {
        float r = ((float)data[index+0])/255;
        float g = ((float)data[index+1])/255;
        float b = ((float)data[index+2])/255;

        float c_max = max(max(r, g), b);
        float c_min = min(min(r, g), b);
        float delta = c_max - c_min;

        data[index+2] = c_max*255;                  // v = c_max

        if (delta < 0.003)                          // d < 1/255
        {
            data[index+0] = 0;
            data[index+1] = 0;
        } else
        {
            if (c_max == r)
            {
                int h = (60*(g-b)/delta)/2;
                data[index+0] = (h < 0) ?  h+=180 : h;
            }
            else if (c_max == g)
            {
                int h = (120+60*(b-r)/delta)/2;
                data[index+0] = (h < 0) ?  h+=180 : h;
            }
            else if (c_max == b)
            {
                int h = (240+60*(r-g)/delta)/2;
                data[index+0] = (h < 0) ?  h+=180 : h;
            }

            data[index+1] = (delta * 255 / c_max);  // s = delta/c_max
        }
    }
}
__global__ void gpu_threshold(uchar* data_in, uchar* data_out, int width, int height, bool ch1_swap,
                          uchar ch1_min, uchar ch1_max, uchar ch2_min, uchar ch2_max, uchar ch3_min, uchar ch3_max)
{
    int index_x = blockIdx.x*blockDim.x+threadIdx.x;
    int index_y = blockIdx.y*blockDim.y+threadIdx.y;
    int index = (index_y*width + index_x);
    int index_3 = 3*index;

    if (index_x<width && index_y<height)
    {
        uchar ch1 = data_in[index_3+0];
        uchar ch2 = data_in[index_3+1];
        uchar ch3 = data_in[index_3+2];

        uchar out = 0;
        if (ch2 >= ch2_min && ch2 <= ch2_max &&
            ch3 >= ch3_min && ch3 <= ch3_max)
        {
            if (!ch1_swap && ch1 >= ch1_min && ch1 <= ch1_max)
                out = 255;

            if (ch1_swap && (ch1 < ch1_min || ch1 > ch1_max))
                out = 255;
        }

        data_out[index] = out;
    }
}

void convertAndThreshold(cv::Mat &mat, sensor_msgs::ImageConstPtr &image, bool h_swap,
                         int h_min, int h_max, int s_min, int s_max, int v_min, int v_max)
{
    int width = image->width;
    int height = image->height;

    assert(width == 640 && height == 480 &&
           !strcmp(image->encoding.c_str(),"rgb8"));

    uchar *d_data_in, *d_data_out;
    cudaMalloc((void**)&d_data_in, 3*width*height*sizeof(uchar));
    cudaMalloc((void**)&d_data_out, width*height*sizeof(uchar));
    cudaMemcpy((void*)d_data_in, (void*)image->data.data(), 3*width*height*sizeof(uchar), cudaMemcpyHostToDevice);

    dim3 blocks(width/32, height/32, 1);
    dim3 threads(32,32,1);
    gpu_rgb2hsv<<<blocks,threads>>>(d_data_in, width, height);
    gpu_threshold<<<blocks,threads>>>(d_data_in, d_data_out, width, height, h_swap, h_min, h_max, s_min, s_max, v_min, v_max);

    mat = cv::Mat(height, width, CV_8UC1);
    cudaMemcpy((void*)mat.data, (void*)d_data_out, width*height*sizeof(uchar), cudaMemcpyDeviceToHost);

    cudaFree((void*)d_data_in);
    cudaFree((void*)d_data_out);
}
