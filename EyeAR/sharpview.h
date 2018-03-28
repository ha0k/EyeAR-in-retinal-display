/*
declarations of sharpview GPU implementation (partial)

Copyright Alexander Plopski
29.11.2016
*/

#pragma once
#include <opencv2/opencv.hpp>
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaarithm.hpp"
#include <opencv2/cudafilters.hpp>

struct SharpView
{
    SharpView(int deviceID);
    ~SharpView();
    void evaluate(const cv::Mat &input, cv::Mat &output, double sigma, float sn_inverse);
    
private:
    void init();
    void ShiftDFTGPUM(const cv::Mat &src_arr, int IMAGE_WIDTH, int IMAGE_HEIGHT, cv::Mat &dst);
    void initGaussianMatrix(double SIGMA, int IMAGE_HEIGHT, int IMAGE_WIDTH, cv::cuda::GpuMat &gaussianMatrix);
    void Wiener_Filter(const cv::cuda::GpuMat &image, const cv::cuda::GpuMat &gauss, float SN_inverse, int IMAGE_WIDTH, int IMAGE_HEIGHT, cv::Mat &result);
    
    cv::cuda::GpuMat inputGPU, inputGPU32; //input image uploaded to the GPU
    cv::cuda::GpuMat zeroFiller;
    cv::cuda::GpuMat gauss;
    int width, height;
    //vectors that hold information
    std::vector<cv::cuda::GpuMat> mergeVector2, splitVector2, splitVector3;
    //combined matrices
    cv::cuda::GpuMat defocusimg_complex, gaussian_complex;
    cv::cuda::GpuMat defocusimg_dft, gaussian_dft;

    cv::cuda::GpuMat defocusimg_dft_real, defocusimg_dft_imaginary;
    cv::cuda::GpuMat gaussian_dft_real, gaussian_dft_imaginary;

    cv::cuda::GpuMat gdftrealtempmult, gdftimaginarytempmult;

    cv::cuda::GpuMat gaussian_spectrum, gaussian_sqrt;

    cv::cuda::GpuMat left1, left2;
    cv::cuda::GpuMat right1, right2;

    cv::cuda::GpuMat mix_real2_gpu;
    cv::cuda::GpuMat mix_imaginary2_gpu;

    cv::cuda::GpuMat mix_real_gpu, mix_imaginary_gpu;

    cv::cuda::GpuMat Gaussian_deconvolution; //2C
    cv::cuda::GpuMat defocusimg_idft; //2C

    std::vector<cv::Mat> dftResult; //this should be GPU!!
    cv::Mat dst_arr, dst_arr32;
};

