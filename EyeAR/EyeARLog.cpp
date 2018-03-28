
#include "EyeARLog.h"
#include <ctime>
//#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <Windows.h>

void EyeARLog::start()
{
  time_t        now        = time(NULL);
  struct        tm *ts    = localtime(&now);
  char          buf[80];
  char*         logFileName="";  
    
  SYSTEMTIME st;
  GetSystemTime(&st);
  initialTimeStamp = st.wMilliseconds;

  strftime(buf, sizeof(buf), "EyeARlog_%Y%m%d_%H%M%S.csv", ts);
  strcpy(logFileName, buf);
   
  std::cout << "Log file name: " << logFileName << std::endl; 

  logFile.open(logFileName);
	logFile << "TimeStamp,FocalLength,PupilSize" << std::endl;
}

void EyeARLog::stop()
{
  logFile.close();
}

void EyeARLog::write(std::string ss)
{
  SYSTEMTIME st;
  GetSystemTime(&st);

  logFile << st.wMilliseconds - initialTimeStamp << "," << ss << std::endl;
}


