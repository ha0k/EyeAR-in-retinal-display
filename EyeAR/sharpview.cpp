/*
GPU implementation of sharpview, adapted from original code
*/
#include <iostream>
#include "sharpview.h"


//forward declarations
//if all your matrices are 32F there is no reason to add a double

////variables


//functions

/*
 You are not recommended to leave static or global GpuMat variables allocated, that is, to rely on its destructor. The destruction order of such variables and CUDA context is undefined. GPU memory release function returns error if the CUDA context has been destroyed before.
 */

SharpView::~SharpView()
{
    dst_arr.release();
    dst_arr32.release();
    inputGPU.release();
    inputGPU32.release(); //input image uploaded to the GPU

    zeroFiller.release();

    gauss.release();

    //vectors that hold information
    mergeVector2.clear();
    splitVector2.clear();
    splitVector3.clear();
    //combined matrices
    defocusimg_complex.release();
    gaussian_complex.release();;
    defocusimg_dft.release(); 
    gaussian_dft.release();

    defocusimg_dft_real.release();
    defocusimg_dft_imaginary.release();
    gaussian_dft_real.release(); gaussian_dft_imaginary.release();

    gdftrealtempmult.release(); 
    gdftimaginarytempmult.release();

    gaussian_spectrum.release();
    gaussian_sqrt.release();

    left1.release();
    left2.release();
    right1.release();
    right2.release();

    mix_real2_gpu.release();
    mix_imaginary2_gpu.release();

    mix_real_gpu, mix_imaginary_gpu.release();

    Gaussian_deconvolution.release(); //2C
    defocusimg_idft.release(); //2C

    dftResult.clear();
}

SharpView::SharpView(int deviceID)
{
    cv::cuda::setDevice(deviceID);
}

void
SharpView::init()
{	
    dst_arr = cv::Mat::zeros(height, width, CV_64FC1);//HACK: why do we even need a double matrix if we convert it to float in the end anyway?
    dst_arr32 = cv::Mat::zeros(height, width, CV_32FC1);
    mergeVector2.resize(2);
    mergeVector2[0] = cv::cuda::GpuMat(height, width, CV_32FC1);
    mergeVector2[1] = cv::cuda::GpuMat(height, width, CV_32FC1);
    splitVector2.resize(2);
    splitVector2[0] = cv::cuda::GpuMat(height, width, CV_32FC1);
    splitVector2[1] = cv::cuda::GpuMat(height, width, CV_32FC1);
    splitVector3.resize(3);
    splitVector3[0] = cv::cuda::GpuMat(height, width, CV_32FC1);
    splitVector3[1] = cv::cuda::GpuMat(height, width, CV_32FC1);
    splitVector3[2] = cv::cuda::GpuMat(height, width, CV_32FC1);
    
    
    inputGPU = cv::cuda::GpuMat(height, width, CV_8UC3);
    inputGPU32 = cv::cuda::GpuMat(height, width, CV_32FC3);
    zeroFiller = cv::cuda::GpuMat(height, width, CV_32FC1);
    zeroFiller.setTo(0);

    gaussian_complex = cv::cuda::GpuMat(height, width, CV_32FC2);
    defocusimg_complex = cv::cuda::GpuMat(height, width, CV_32FC2);
    defocusimg_dft = cv::cuda::GpuMat(height, width, CV_32FC2);
    gaussian_dft = cv::cuda::GpuMat(height, width, CV_32FC2);

    defocusimg_dft_real = cv::cuda::GpuMat(height, width, CV_32FC1);
    defocusimg_dft_imaginary = cv::cuda::GpuMat(height, width, CV_32FC1);

    gaussian_dft_real = cv::cuda::GpuMat(height, width, CV_32FC1);
    gaussian_dft_imaginary = cv::cuda::GpuMat(height, width, CV_32FC1);

    gdftrealtempmult = cv::cuda::GpuMat(height, width, CV_32FC1);
    gdftimaginarytempmult = cv::cuda::GpuMat(height, width, CV_32FC1); 

    gaussian_spectrum = cv::cuda::GpuMat(height, width, CV_32FC1);
    gaussian_sqrt = cv::cuda::GpuMat(height, width, CV_32FC1);

    left1 = cv::cuda::GpuMat(height, width, CV_32FC1);
    right1 = cv::cuda::GpuMat(height, width, CV_32FC1);
    left2 = cv::cuda::GpuMat(height, width, CV_32FC1);
    right2 = cv::cuda::GpuMat(height, width, CV_32FC1);


    mix_real2_gpu = cv::cuda::GpuMat(height, width, CV_32FC1);
    mix_imaginary2_gpu = cv::cuda::GpuMat(height, width, CV_32FC1);

    mix_real_gpu = cv::cuda::GpuMat(height, width, CV_32FC1);
    mix_imaginary_gpu = cv::cuda::GpuMat(height, width, CV_32FC1);

    Gaussian_deconvolution = cv::cuda::GpuMat(height, width, CV_32FC2);
    defocusimg_idft = cv::cuda::GpuMat(height, width, CV_32FC2);

    dftResult.resize(3);
    dftResult[0] = cv::Mat::zeros(height, width, CV_32FC1);
    dftResult[1] = cv::Mat::zeros(height, width, CV_32FC1);
    dftResult[2] = cv::Mat::zeros(height, width, CV_32FC1);
}

void 
SharpView::initGaussianMatrix(double SIGMA, int IMAGE_HEIGHT, int IMAGE_WIDTH, cv::cuda::GpuMat &gaussianMatrix)
{
    dst_arr.setTo(0);
    if (SIGMA==0.0)
    {
        dst_arr.convertTo(dst_arr32, CV_32FC1);
        gaussianMatrix.upload(dst_arr32);
        return;
    }
    //TODO: convert SIGMA to pixels!!! 
    int cx = IMAGE_WIDTH/2, cy = IMAGE_HEIGHT/2;   
    int i, j;
    double SIGMA_variable = 1 / (2*SIGMA*SIGMA);
    double distance;

    ////make sure that the kernel size is SIGMA pixels, ignore all other values
    //for(j=0;j<abs(SIGMA);j++){
    //    for(i=0;i<abs(SIGMA);i++){
    //        distance = double(i) * double(i) + double(j) * double(j);
    //        double value = (SIGMA_variable / CV_PI) * exp(-distance * SIGMA_variable);
    //        dst_arr.at<double>(cx+j,cx+i) = value;
    //        dst_arr.at<double>(cx-j,cx-i) = value;
    //        dst_arr.at<double>(cx+j,cx-i) = value;
    //        dst_arr.at<double>(cx-j,cx+i) = value;
    //    }
    //}


    for(j=0;j<IMAGE_HEIGHT;j++){
        for(i=0;i<IMAGE_WIDTH;i++){
            distance = double(cx - i) * double(cx - i) + double(cy - j) * double(cy - j);
            dst_arr.at<double>(j,i) = (SIGMA_variable / CV_PI) * exp(-distance * SIGMA_variable);
        }
    }
    
    dst_arr.convertTo(dst_arr32, CV_32FC1);
    gaussianMatrix.upload(dst_arr32);
} //can be done directly on the GPU

//TODO: this function should be on the GPU
void 
SharpView::ShiftDFTGPUM(const cv::Mat &src_arr, int IMAGE_WIDTH, int IMAGE_HEIGHT, cv::Mat &dst)
{
  int cx = IMAGE_WIDTH / 2, cy = IMAGE_HEIGHT / 2;
  int i, j;
  if (dst.rows != IMAGE_WIDTH ||
      dst.cols != IMAGE_HEIGHT ||
      dst.type() != CV_32FC1)
  {
    dst = cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC1);
  }

  for (j = 0; j<cy; j++)
  {
    for (i = 0; i<cx; i++)
    {

      dst.at<float>(j, i) = src_arr.at<float>(j + cy, i + cx);
      dst.at<float>(j + cy, i) = src_arr.at<float>(j, i + cx);
      dst.at<float>(j, i + cx) = src_arr.at<float>(j + cy, i);
      dst.at<float>(j + cy, i + cx) = src_arr.at<float>(j, i);
    }
  }
}

void 
SharpView::Wiener_Filter(const cv::cuda::GpuMat &image, const cv::cuda::GpuMat &gauss, float SN_inverse, int IMAGE_WIDTH, int IMAGE_HEIGHT, cv::Mat &result)
{
    //Initialize
   //Generating 2-channel array for the real and imaginary parts
    mergeVector2[0] = image;
    mergeVector2[1] = zeroFiller;
    cv::cuda::merge(mergeVector2, defocusimg_complex);
    mergeVector2[0] = gauss;
    mergeVector2[1] = zeroFiller;
    cv::cuda::merge(mergeVector2, gaussian_complex);

  //  std::cout << "DFT" << std::endl;    
    cv::cuda::dft(defocusimg_complex, defocusimg_dft, defocusimg_complex.size());
    cv::cuda::dft(gaussian_complex, gaussian_dft, gaussian_complex.size());
   // std::cout << "SPlit1"<< std::endl;
    cv::cuda::split(defocusimg_dft, splitVector2);
    splitVector2[0].copyTo(defocusimg_dft_real);
    splitVector2[1].copyTo(defocusimg_dft_imaginary);
    //std::cout << "SPlit2"<< std::endl;    
    cv::cuda::split(gaussian_dft, splitVector2);
    splitVector2[0].copyTo(gaussian_dft_real);
    splitVector2[1].copyTo(gaussian_dft_imaginary);
   

    //gaussian power gpu
    //P^2
    cv::cuda::multiply(gaussian_dft_real, gaussian_dft_real, gdftrealtempmult);
    cv::cuda::multiply(gaussian_dft_imaginary, gaussian_dft_imaginary, gdftimaginarytempmult);

    //Gaussian power
    cv::cuda::add(gdftrealtempmult, gdftimaginarytempmult, gaussian_spectrum);
    //P^2 + C^2
    cv::cuda::add(gaussian_spectrum, SN_inverse, gaussian_sqrt);
    
    //O*P^-1
    cv::cuda::multiply(defocusimg_dft_real, gaussian_dft_real, left1);
    cv::cuda::multiply(defocusimg_dft_imaginary, gaussian_dft_imaginary, right1);
    cv::cuda::multiply(defocusimg_dft_imaginary, gaussian_dft_real, left2);
    cv::cuda::multiply(defocusimg_dft_real, gaussian_dft_imaginary, right2);

    cv::cuda::add(left1,right1, mix_real2_gpu);
    cv::cuda::subtract(left2,right2, mix_imaginary2_gpu);
    //O*P^-1/(P^2+C^2)
    cv::cuda::divide(mix_real2_gpu, gaussian_sqrt, mix_real_gpu);
    cv::cuda::divide(mix_imaginary2_gpu, gaussian_sqrt, mix_imaginary_gpu);

    splitVector2[0] = mix_real_gpu;
    splitVector2[1] = mix_imaginary_gpu;

	//std::cout << "Last merge" << std::endl;
    cv::cuda::merge(splitVector2, Gaussian_deconvolution);
    cv::cuda::dft(Gaussian_deconvolution, defocusimg_idft,Gaussian_deconvolution.size(), cv::DFT_INVERSE | cv::DFT_SCALE);
//	std::cout << "Last split" << std::endl;
    cv::cuda::split(defocusimg_idft, splitVector2);
    cv::Mat idft_result;
    splitVector2[0].download(idft_result);
    //why do we neet this?
    ShiftDFTGPUM(idft_result, idft_result.cols, idft_result.rows, result); //this founction should be written on GPU!!!
    //cv::imshow("res", idft_result);
    
}

void 
SharpView::evaluate(const cv::Mat &input, cv::Mat &output, double sigma, float sn_inverse)
{
  width = input.cols;
  height = input.rows;
  std::cout <<"sharpview running " << sigma << " " << height <<" " << width << " " << inputGPU.cols <<" " << inputGPU.rows << "\n";
  if(height != inputGPU.cols || width != inputGPU.rows){
	  init();
  }
  //   cv::imshow("inM", input);
  cv::Mat output32 = cv::Mat::zeros(height, width, CV_32FC3);

//   int sysTime1 = getMilliCount();
  initGaussianMatrix(sigma, height, width, gauss); //initialize the gaussian matrix based on the newly computed sigma.
  
  inputGPU.upload(input);
  inputGPU.convertTo(inputGPU32, CV_32FC3);
  cv::cuda::split(inputGPU32, splitVector3);
  double minIn,maxIn, maxOut;
  
  for(int i=0;i<3;i++)
  {
    Wiener_Filter(splitVector3[i], gauss, sn_inverse, width, height, dftResult[i]);
  }  

  cv::merge(dftResult, output32);
  //  int sysTime2 = getMilliCount();
  //  int totalTime = sysTime2 - sysTime1;
//   std::cout << "Took: " << totalTime << " ms" << std::endl;
  output32.convertTo(output, CV_8UC3);
 //  cv::imshow("outM", output);
   //cv::waitKey();
//  }
 // cv::resize(output8,output8,sz*2);
//    cv::imshow("blah",output8);
  output32.release();

} 
