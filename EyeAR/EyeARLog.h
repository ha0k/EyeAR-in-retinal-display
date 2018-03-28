#include <iostream>
#include <fstream>
#include <sstream>

class EyeARLog
{
  std::ofstream logFile;
  long          initialTimeStamp;

  public:

  EyeARLog(){};

  void start();
  void stop();
  void write(std::string ss);

};