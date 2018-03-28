/*
this struct handles the loading of the images for the experiment




*/

#ifndef _EXPERIMENT_IMAGELOADER_HPP_
#define _EXPERIMENT_IMAGELOADER_HPP_

#include <string>;
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <sstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>
#include <boost/thread.hpp>
#include <boost/timer.hpp>

#include "commonStructs.h"
#include "Server.hpp"


#include <ctime>
//#define CAMERA
#ifdef CAMERA
#include "Server.hpp"
#endif

#define NEWINTERFACE
#ifdef NEWINTERFACE
#include "Server.hpp"

class SimpleSerial
	{
		public:
		int i;
		char sBuf[1];
		int baudRate;	//Adjust baudrate value
		unsigned long nn;
		DCB dcb;
		COMMTIMEOUTS cto;
		DWORD dwBytesWritten;
		DWORD  dwBytesRead;
		/**

		 * Constructor.
		 * \param port device name, example "/dev/ttyUSB0" or "COM4"
		 * \param baud_rate communication speed, example 9600 or 115200
		 * \throws boost::system::system_error if cannot open the
		 * serial device
		 */
		HANDLE hhhh;
		SimpleSerial(){
		}
		void init(char* port, unsigned int baud_rate)
		{
	//        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

		hhhh = CreateFileA(							// Changed CreateFile() to CreateFileA()
							port,GENERIC_READ | GENERIC_WRITE,
							0,
							0,
							OPEN_EXISTING,
							0,
							0); 
					if (hhhh == INVALID_HANDLE_VALUE) {
						printf("Open Error!\n");
					}
			 GetCommState(hhhh, &dcb); // Get the status of the serial port
			dcb.BaudRate = 9600;
			SetCommState(hhhh, &dcb); // Set the status of the serial port

			/* ----------------------------------------------
				Timeout state operation of the serial port
			---------------------------------------------- */
			GetCommTimeouts(hhhh, &cto); // Get the setting state of the time-out
			cto.ReadIntervalTimeout = 100000;
			cto.ReadTotalTimeoutMultiplier = 0;
			cto.ReadTotalTimeoutConstant = 100000;
			cto.WriteTotalTimeoutMultiplier = 0;
			cto.WriteTotalTimeoutConstant = 0;
			SetCommTimeouts(hhhh, &cto); // Set the state of the time-out
		}

    void writeString(std::string s)
		{
			WriteFile(hhhh,&s,(DWORD)sizeof(s),&dwBytesWritten,NULL);
		}
		
    std::string readLine(){
			std::string result;
			for(;;)
			{
			  //  asio::read(serial,asio::buffer(&c,1));
				ReadFile(hhhh, sBuf, 1, &nn, 0);
        if (nn == 0)
        {
          Sleep(1);
          ReadFile(hhhh, sBuf, 1, &nn, 0);
        }
        if (nn == 0)
        {
          return std::string("NO");
        }

				switch(sBuf[0])
				{
					case '\r':
						break;
					case '\n':
						return result;
					default:
						result+=sBuf[0];
						//std::cout << sBuf[0] <<std::endl;
				}
			}
		}
	};

#endif





struct ImageLoader
{
public:
  ImageLoader();
  ImageLoader(const char* folderName);
  ~ImageLoader();
  void generateImage();
  void moveLeft();
  void moveRight();
  void moveUp();
  void moveDown();
  void setRadius(float pupilRadius);
  void setFocusDistance(float focusDistance);
  void runProgram();
  void stopProgram();
  void interpolation(float&radius, float&distance);

private:

  void openLoggers();

  int offsetX;//handles the offset from the left, the origin of the OpenGL coordinate system does not coincide with OpenCV
  int offsetY;//handles the offset from the top, the headers are different size
  std::ofstream loggerPresentation, loggerRadius, loggerDistance;

  float pupilRadius_;
  float focusDistance_;

  boost::mutex readWriteMutex;
  bool finish;

  float lastMeasurementRadius, curMeasurementRadius;
  float lastMeasurementDistance, curMeasurementDistance;
  boost::timer programStart;
  boost::timer lastMeasurement;

  //extrapolation //what would be a good extrapolation algorithm? The method in EyeAR is no good...
  //use 100 ms to move from current pupil radius to estimated pupil radius
  //use 100 ms to extrapolate from last 4 measurements?

  //sharpview off, on
  //front, mid, back
  //pupilDiameter
  //focusDistance
  std::vector<std::vector<std::vector<std::vector<cv::Mat>>>> imageDatabase;

  std::vector<cv::Mat> focusedImages;

  std::vector<cv::Point> topLeft;
  std::vector<cv::Size> size;
  cv::Size imageSize;

  int numPupilSizes, numDistances;

  int leftAugmentation; //can be either 0, or 1, for left, mid
  int rightAugmentation; //can be either 0, or 1, for left, mid
  int pillarPosition; //can be either 0, 1, or 2 for front, mid, back
  int correctionRight, correctionLeft;
  float scale; 

  float minRadius, minFocusDistance;
  char delimiter;
  int counter;
#ifdef CAMERA
  ServerNetwork* server;
#endif
#ifdef NEWINTERFACE
  ServerNetwork* server;
  SimpleSerial serial;
  cv::Point click;
  int currentPosition;
#endif

};

#endif