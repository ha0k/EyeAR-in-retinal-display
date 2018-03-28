#include "ExperimentImageLoader.hpp"
#define NOMINMAX
#include <math.h>

inline float round( float val )
{
    if( val < 0 ) return ceil(val - 0.5);
    return floor(val + 0.5);
}

void MouseFunc(int event, int x, int y, int flags, void* data)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          cv::Point* p = (cv::Point*) data;
          p->x = x;
          p->y = y;
     }
     
}


#ifndef NEWINTERFACE
ImageLoader::ImageLoader():
offsetX(15), //experimentally found values, together with values stored in GLUTDisplay (search for OpenCV)
offsetY(0),//15),
numPupilSizes(63),
numDistances(31),
delimiter(';'),
scale(1.f)
{
  logger.open("log.csv");
  if (!logger.is_open())
  {
    printf("failed to open logger!\n");
    exit(-1);
  }
  logger << "timestamp;focusDepth;pupilRadius;fileID;readSuccess\n";
  cv::namedWindow("EyeARWindow");

  
  std::vector<std::string> folders;
  folders.push_back(std::string("./imageDatabase_Front"));
  folders.push_back(std::string("./imageDatabase_Mid"));
  folders.push_back(std::string("./imageDatabase_Back"));

  std::vector<std::string> sharpViewFolders;
  sharpViewFolders.push_back(std::string("/"));
  sharpViewFolders.push_back(std::string("/Merge/"));

  pupilRadius_ = lastMeasurementRadius = curMeasurementRadius = 8.00f;
  focusDistance_ = lastMeasurementDistance = curMeasurementDistance = 0.52f;
  
  topLeft.push_back(cv::Point(297, 319));
  topLeft.push_back(cv::Point(766, 489));
  topLeft.push_back(cv::Point(1108, 628));

  size.push_back(cv::Size(400, 1200));
  size.push_back(cv::Size(400, 1000));
  size.push_back(cv::Size(400, 800));
  leftAugmentation = -1;
  rightAugmentation = -1;
  pillarPosition = 0;
  correctionRight = 0;
  correctionLeft = 0;

    
  minRadius = 1.9f;
  minFocusDistance = 0.25f;
  imageDatabase.reserve(2);
  cv::Mat image;
  
  for (size_t folder = 0; folder < folders.size(); ++folder)
  {
    image = cv::imread(folders[folder]+sharpViewFolders[0]+"focused.png");
    focusedImages.push_back(image.colRange(topLeft[folder].x, topLeft[folder].x + size[folder].width).rowRange(topLeft[folder].y, topLeft[folder].y + size[folder].height).clone());
  }

  FILE * f;
  fopen_s(&f, "test.binary", "rb");
  for (size_t sharpView = 0; sharpView < sharpViewFolders.size(); ++sharpView)
  {
    imageDatabase.push_back(std::vector<std::vector<std::vector<cv::Mat>>>());
    imageDatabase.back().reserve(3);
    for (size_t folder = 0; folder < folders.size(); ++folder)
    {
      imageDatabase[sharpView].push_back(std::vector<std::vector<cv::Mat>>());
      
      cv::Mat tmp = cv::Mat::zeros(size[folder].height, size[folder].width, CV_8UC3);
      imageDatabase[sharpView].back().reserve(numPupilSizes);
      
      for (int diameter = 0; diameter < numPupilSizes; ++diameter)
      {
        imageDatabase[sharpView][folder].push_back(std::vector<cv::Mat>());
        imageDatabase[sharpView][folder][diameter].reserve(numDistances);
        for (float distance = 0; distance < numDistances; ++distance)
        {
          imageDatabase[sharpView][folder][diameter].push_back(tmp.clone());
        }
      }

      int diameter = 0;
      for (float pupilDiameter = minRadius; pupilDiameter < 8.15f; pupilDiameter +=0.1f)
      {
        //imageDatabase[sharpView][folder].push_back(std::vector<cv::Mat>());
        //imageDatabase[sharpView][folder].back().resize(31, cv::Mat::zeros(size[folder].height, size[folder].width, CV_8UC3));
        int distance = 0;
        for (float focusDistance = minFocusDistance; focusDistance < 0.555f; focusDistance += 0.01f)
        {
          std::ostringstream fileName;    
          fileName << std::fixed << std::setprecision(2);
          fileName << folders[folder] << sharpViewFolders[sharpView] << pupilDiameter << "_" << focusDistance << ".png";
          printf("%s %d %d\n", fileName.str().c_str(), diameter, distance);
          //image= cv::imread(fileName.str());
          //cv::flip(image, image, 0);
          /*imageDatabase[sharpView][folder].back().push_back(image.colRange(topLeft[folder].x, topLeft[folder].x + size[folder].width).rowRange(topLeft[folder].y, topLeft[folder].y + size[folder].height).clone());*/
          //(image.colRange(topLeft[folder].x, topLeft[folder].x + size[folder].width).rowRange(topLeft[folder].y, topLeft[folder].y + size[folder].height).clone()).copyTo(imageDatabase[sharpView][folder][diameter][distance]);
          
          fread(imageDatabase[sharpView][folder][diameter][distance].data, sizeof(unsigned char), size[folder].area() * 3, f);
          std::cout << imageDatabase[sharpView][folder][diameter][distance].cols  << " " << imageDatabase[sharpView][folder][diameter][distance].rows <<"\n";
          //if (sharpView > 0)
          //{
          //  cv::imshow("w", imageDatabase[sharpView][folder][diameter][distance]);
          //  cv::waitKey();
          //}
          
          distance++;
        }
        diameter++;
      }
    }
  }
 
  //for (size_t sharpView = 0; sharpView < sharpViewFolders.size(); ++sharpView)
  //{
  //  for (size_t folder = 0; folder < folders.size(); ++folder)
  //  {
  //    
  //    for (int diameter = 0; diameter < numPupilSizes; ++diameter)
  //    {
  //      //imageDatabase[sharpView][folder].push_back(std::vector<cv::Mat>());
  //      //imageDatabase[sharpView][folder].back().resize(31, cv::Mat::zeros(size[folder].height, size[folder].width, CV_8UC3));
  //      for (float distance = 0; distance < numDistances; ++distance)
  //      {
  //          cv::imshow("w", imageDatabase[sharpView][folder][diameter][distance]);
  //          cv::waitKey();
  //      }
  //    }
  //  }
  //}
  fclose(f);
  
  imageSize = cv::Size(2560, 1600);

#ifdef CAMERA
  server = new ServerNetwork();
  unsigned int id;
  printf("waiting for client\n");
  while(!server->acceptNewClient(id))
  {
    Sleep(1);
  }
#endif


  int closestRadius = static_cast<int>(round((pupilRadius_ - minRadius) * 10));
  int closestDistance = static_cast<int>(round((focusDistance_ - minFocusDistance) * 100));
  cv::Mat win =cv::Mat::zeros(imageSize.height, imageSize.width, CV_8UC3);

  if (rightAugmentation == -1)
  {
    focusedImages[pillarPosition].copyTo(win.colRange(topLeft[0].x + correctionRight, topLeft[0].x + correctionRight + size[pillarPosition].width).rowRange(topLeft[pillarPosition].y, topLeft[pillarPosition].y + size[pillarPosition].height));
  }
  else if (rightAugmentation == 0)
  {
    imageDatabase[0][pillarPosition][closestRadius][closestDistance].copyTo(win.colRange(topLeft[0].x + correctionRight, topLeft[0].x + correctionRight + size[pillarPosition].width).rowRange(topLeft[pillarPosition].y, topLeft[pillarPosition].y + size[pillarPosition].height));
  }
  else
  {
    imageDatabase[1][pillarPosition][closestRadius][closestDistance].copyTo(win.colRange(topLeft[0].x + correctionRight, topLeft[0].x + correctionRight + size[pillarPosition].width).rowRange(topLeft[pillarPosition].y, topLeft[pillarPosition].y + size[pillarPosition].height));
  }

  if (leftAugmentation == -1)
  {
    focusedImages[pillarPosition].copyTo(win.colRange(topLeft[1].x + correctionLeft, topLeft[1].x + correctionLeft + size[pillarPosition].width).rowRange(topLeft[pillarPosition].y, topLeft[pillarPosition].y + size[pillarPosition].height));
  }
  else if (leftAugmentation == 0)
  {
    imageDatabase[0][pillarPosition][closestRadius][closestDistance].copyTo(win.colRange(topLeft[1].x + correctionLeft, topLeft[1].x + correctionLeft + size[pillarPosition].width).rowRange(topLeft[pillarPosition].y, topLeft[pillarPosition].y + size[pillarPosition].height));
  }
  else
  {
    imageDatabase[1][pillarPosition][closestRadius][closestDistance].copyTo(win.colRange(topLeft[1].x + correctionLeft, topLeft[1].x  + correctionLeft + size[pillarPosition].width).rowRange(topLeft[pillarPosition].y, topLeft[pillarPosition].y + size[pillarPosition].height));
  }
  cv::Mat tmp;
cv::resize(image, tmp, cv::Size(1024, 1024.f/2560.f * 1600.f));//, 0.5, 0.5);
    cv::Mat tmp2 = tmp.colRange(0,1024.f/1280.f*850);
  cv::imshow("EyeARWindow", tmp2);
  cv::moveWindow("EyeARWindow", offsetX, offsetY);
}
#else
ImageLoader::ImageLoader():
delimiter(','),
click(cv::Point(-1,-1))
{
  counter = 0;
  openLoggers();

  lastMeasurement.restart();
  
  pupilRadius_ = lastMeasurementRadius = curMeasurementRadius = 8.00f;
  focusDistance_ = lastMeasurementDistance = curMeasurementDistance = 0.52f;
  
  std::cout << "Opening Port" << std::endl;
	serial.init("COM7",9600);
	std::cout << "Starting Read Thread" << std::endl;		
	//serial.startReadThread();
	std::cout << "Writing Port" << std::endl;
  serial.writeString("!:\r\n");
  std::cout << ":" << serial.readLine() << ":"<<std::endl;
	serial.writeString("Q:\r\n");
	std::cout << serial.readLine() << std::endl;
	//std::cout << serial.readLine() << std::endl;
		serial.writeString("D:1S200F20000R200\r\n");
	//std::cout << serial.readLine() << std::endl;
	serial.writeString("H:1\r\n");
	std::cout << serial.readLine() << std::endl;/*
  serial.writeString("!\r\n");
  std::string x = serial.readLine();
  while (std::strcmp(x.c_str(), "R") != 0 && std::strcmp(x.c_str(), "R") != 0)
  {
    Sleep(1);
    serial.writeString("Q\r\n");
    
    x = serial.readLine();
    std::cout << x << " " << std::strcmp(x.c_str(), "R") <<"\n";
  }*/
  currentPosition = 0;
  cv::Mat interfaceImage = cv::imread("interface.png");
  cv::imshow("Interface", interfaceImage);
  cv::moveWindow("Interface", 0, 0);
  cv::setMouseCallback("Interface", MouseFunc, &click);
  server = new ServerNetwork();
  unsigned int id;
  printf("waiting for client\n");
  while(!server->acceptNewClient(id))
  {
    Sleep(1);
  }
}
#endif

void
ImageLoader::openLoggers()
{
  if (loggerPresentation.is_open())
  {
    loggerPresentation.close();
    counter++;
  }
  if (loggerRadius.is_open())
  {
    loggerRadius.close();
  }
  if (loggerDistance.is_open())
  {
    loggerDistance.close();
  }
  std::ostringstream loggerName1, loggerName2, loggerName3;
    loggerName1 << "logs/logPresentation" << counter << ".csv";
  loggerPresentation.open(loggerName1.str().c_str());
  if (!loggerPresentation.is_open())
  {
    printf("failed to open loggerPresentation!\n");
    exit(-1);
  }
  loggerPresentation << "timestamp"<<delimiter << "blur applied"<<delimiter << "position Virtual pillar"<<delimiter << "rendering distance\n";
  loggerName2 << "logs/logRadius" << counter << ".csv";
  loggerRadius.open(loggerName2.str().c_str());
  if (!loggerRadius.is_open())
  {
    printf("failed to open loggerRadius!\n");
    exit(-1);
  }
  loggerRadius << "timestamp"<<delimiter << "pupil radius\n";
  loggerName3 << "logs/logDistance" << counter << ".csv";
  std::cout << loggerName3.str() <<"\n";
  loggerDistance.open(loggerName3.str().c_str());
  if (!loggerRadius.is_open())
  {
    printf("failed to open loggerDistance!\n");
    exit(-1);
  }
  loggerDistance << "timestamp"<<delimiter << "focus Distance\n";
}



ImageLoader::~ImageLoader()
{
  #ifdef CAMERA
  delete server;
  #endif
  cv::destroyAllWindows();
}

void
ImageLoader::setRadius(float pupilRadius)
{
  readWriteMutex.lock();
  lastMeasurementRadius = curMeasurementRadius;
  pupilRadius_ = static_cast<int>(round(pupilRadius * 10)) / 10.f;;
  //printf("setting distance %5.3f 5.3f, %d\n", pupilRadius_, pupilRadius, static_cast<int>(round(pupilRadius_ * 10)) / 10.f);
  printf("setting radius %5.3f\n",  pupilRadius_);
  char message[sizeof(int) +sizeof(float)];
  int messageID = 2;
  memcpy(message, &messageID, sizeof(int));
  memcpy(message + sizeof(int), &pupilRadius_, sizeof(float));
  server->sendToAll(message, sizeof(int) + sizeof(float));
  loggerRadius << lastMeasurement.elapsed()<<delimiter <<  pupilRadius_<<"\n";
  readWriteMutex.unlock();
}

void
ImageLoader::setFocusDistance(float focusDistance)
{
  readWriteMutex.lock();
  lastMeasurementDistance = curMeasurementDistance;
  focusDistance_ = focusDistance;
  printf("focus distance %5.3f\n", focusDistance_);
  loggerDistance << lastMeasurement.elapsed() <<delimiter <<  focusDistance_<<"\n";
  readWriteMutex.unlock();
}

#ifndef NEWINTERFACE
void
ImageLoader::runProgram()
{
  float radius, distance;
 
  finish = false;
  cv::Mat image = cv::Mat::zeros(imageSize.height, imageSize.width, CV_8UC3);
  programStart.restart();
  time_t timer;
  while (!finish)
  {
    #ifdef CAMERA
    float depth = server->receiveFromClients();
    if (depth < 0)
    {
      cv::waitKey(1000);
      continue;
    }
    if (depth > 1.0)
    {
      finish = true;
      continue;
    }
    setFocusDistance(depth);
    #endif
    readWriteMutex.lock();
    radius = pupilRadius_;
    distance = focusDistance_;
    readWriteMutex.unlock();
    //rounding the numbers
    
    //interpolation(radius, distance);
    radius = static_cast<int>(round(radius * 10)) / 10.f;
    distance =  static_cast<int>(round(distance * 100)) / 100.f;
    timer = std::time(0);
    /*std::ostringstream fileName;
    fileName << std::fixed << std::setprecision(2);
    fileName << "./imageDatabase/" << radius << "_" << distance << ".png";
    */
    //CAREFUL HERE DEPENDS ON HOW DEEP
    if (distance < 0.35f) distance = 0.3f;
    else if (distance < 0.43f) distance = 0.4f;
    else distance = 0.5f;
    int closestRadius = std::max(0, std::min(numPupilSizes, static_cast<int>(round((radius - minRadius) * 10))));
    int closestDistance = std::max(0, std::min(numDistances, static_cast<int>(round((distance - minFocusDistance) * 100))));


    if (rightAugmentation == -1)
    {
      cv::Mat tmp = focusedImages[pillarPosition].clone();
      cv::Mat tmpresized;
      cv::resize(tmp, tmpresized, cv::Size(), scale, scale);
      tmpresized.copyTo(image.colRange(topLeft[0].x + correctionRight, topLeft[0].x + correctionRight + tmpresized.cols).rowRange(topLeft[pillarPosition].y + offsetY, topLeft[pillarPosition].y + offsetY + tmpresized.rows));
    }
    else if (rightAugmentation == 0)
    {
      cv::Mat tmp = imageDatabase[0][pillarPosition][closestRadius][closestDistance].clone();
      cv::Mat tmpresized;
      cv::resize(tmp, tmpresized, cv::Size(), scale, scale);
      tmpresized.copyTo(image.colRange(topLeft[0].x + correctionRight, topLeft[0].x + correctionRight + tmpresized.cols).rowRange(topLeft[pillarPosition].y + offsetY, topLeft[pillarPosition].y  + offsetY + tmpresized.rows));
      //imageDatabase[0][pillarPosition][closestRadius][closestDistance].clone().copyTo(image.colRange(topLeft[eyeARPosition].x + correctionRight, topLeft[eyeARPosition].x + correctionRight + size[pillarPosition].width).rowRange(topLeft[pillarPosition].y + offsetY, topLeft[pillarPosition].y  + offsetY+ size[pillarPosition].height));
    }
    else
    {
      cv::Mat tmp = imageDatabase[1][pillarPosition][closestRadius][closestDistance].clone();
      cv::Mat tmpresized;
      cv::resize(tmp, tmpresized, cv::Size(), scale, scale);
      tmpresized.copyTo(image.colRange(topLeft[0].x + correctionRight, topLeft[0].x + correctionRight + tmpresized.cols).rowRange(topLeft[pillarPosition].y + offsetY, topLeft[pillarPosition].y  + offsetY + tmpresized.rows));
      //imageDatabase[1][pillarPosition][closestRadius][closestDistance].clone().copyTo(image.colRange(topLeft[sharpViewPosition].x + correctionRight, topLeft[sharpViewPosition].x + correctionRight + size[pillarPosition].width).rowRange(topLeft[pillarPosition].y + offsetY, topLeft[pillarPosition].y  + offsetY+ size[pillarPosition].height));
  
    }

    if (leftAugmentation == -1)
    {
      cv::Mat tmp = focusedImages[pillarPosition].clone();
      cv::Mat tmpresized;
      cv::resize(tmp, tmpresized, cv::Size(), scale, scale);
      tmpresized.copyTo(image.colRange(topLeft[1].x + correctionLeft, topLeft[1].x + correctionLeft + tmpresized.cols).rowRange(topLeft[pillarPosition].y + offsetY, topLeft[pillarPosition].y + offsetY+ tmpresized.rows));
   
    }
    else if (leftAugmentation == 0)
    {
      cv::Mat tmp = imageDatabase[0][pillarPosition][closestRadius][closestDistance].clone();
      cv::Mat tmpresized;
      cv::resize(tmp, tmpresized, cv::Size(), scale, scale);
      tmpresized.copyTo(image.colRange(topLeft[1].x + correctionLeft, topLeft[1].x + correctionLeft + tmpresized.cols).rowRange(topLeft[pillarPosition].y + offsetY, topLeft[pillarPosition].y  + offsetY+ tmpresized.rows));
    //imageDatabase[0][pillarPosition][closestRadius][closestDistance].clone().copyTo(image.colRange(topLeft[eyeARPosition].x + correctionLeft, topLeft[eyeARPosition].x + correctionLeft + size[pillarPosition].width).rowRange(topLeft[pillarPosition].y + offsetY, topLeft[pillarPosition].y  + offsetY+ size[pillarPosition].height));
   }
    else
    {
      cv::Mat tmp = imageDatabase[1][pillarPosition][closestRadius][closestDistance].clone();
      cv::Mat tmpresized;
      cv::resize(tmp, tmpresized, cv::Size(), scale, scale);
      tmpresized.copyTo(image.colRange(topLeft[1].x + correctionLeft, topLeft[1].x + correctionLeft + tmpresized.cols).rowRange(topLeft[pillarPosition].y + offsetY, topLeft[pillarPosition].y  + offsetY+ tmpresized.rows));
      //imageDatabase[1][pillarPosition][closestRadius][closestDistance].clone().copyTo(image.colRange(topLeft[sharpViewPosition].x + correctionLeft, topLeft[sharpViewPosition].x + correctionLeft + size[pillarPosition].width).rowRange(topLeft[pillarPosition].y + offsetY, topLeft[pillarPosition].y  + offsetY+ size[pillarPosition].height));
     }

    std::cout << "\n";
    cv::Mat tmp;
    cv::resize(image, tmp, cv::Size(1024, 1024.f/2560.f * 1600.f));//, 0.5, 0.5);
    cv::Mat tmp2 = tmp.colRange(0,1024.f/1280.f*850);
    cv::imshow("EyeARWindow", tmp2);
  //  for (size_t sharpView = 0; sharpView < 2; ++sharpView)
  //{
  //  imageDatabase.push_back(std::vector<std::vector<std::vector<cv::Mat>>>());
  //  imageDatabase.back().reserve(3);
  //  for (size_t folder = 0; folder < 3; ++folder)
  //  {
  //    imageDatabase[sharpView].push_back(std::vector<std::vector<cv::Mat>>());
  //    imageDatabase[sharpView].back().resize(numPupilSizes, std::vector<cv::Mat>(numDistances, cv::Mat::zeros(size[folder].height, size[folder].width, CV_8UC3)));
  //    int diameter = 0;
  //    for (float pupilDiameter = minRadius; pupilDiameter < 8.15f; pupilDiameter +=0.1f)
  //    {
  //      //imageDatabase[sharpView][folder].push_back(std::vector<cv::Mat>());
  //      //imageDatabase[sharpView][folder].back().resize(31, cv::Mat::zeros(size[folder].height, size[folder].width, CV_8UC3));
  //      int distance = 0;
  //      for (float focusDistance = minFocusDistance; focusDistance < 0.555f; focusDistance += 0.01f)
  //      {
  //        std::cout << diameter << " " << distance <<"\n";
  //        if (sharpView > 0)
  //        {
  //          cv::imshow("w", imageDatabase[sharpView][folder][diameter][distance]);
  //          cv::waitKey();
  //        }
  //        
  //        distance++;
  //      }
  //      diameter++;
  //    }
  //  }
  //}
   
    logger << programStart.elapsed()  << delimiter << distance << delimiter << radius<< delimiter /* << fileName.str()*/  <<delimiter << "ok"<<"\n";
    printf("left: %d right: %d focus:%d correctionRight:%d correctionLeft:%d scale%5.3f offsetY:%d\n", leftAugmentation, rightAugmentation, pillarPosition, correctionRight, correctionLeft, scale, offsetY);
    
    int key = cv::waitKey(10);
    if (key == 27)
    {
      printf("triggered exit\n");
      /*stopProgram();*/
      finish = true;
    }
    else if ((char)key == 'q') //display focus on the left
    {
      image.setTo(0);
      leftAugmentation = -1;
    }
    else if ((char)key == 'a') //display the eyear on the left
    {
      image.setTo(0);
      leftAugmentation = 0;
    }
    else if ((char)key == 'z') //display the sharpview on the left
    {
      image.setTo(0);
      leftAugmentation = 1;
    }
    else if ((char)key == 'w') //display focus on the right
    {
      image.setTo(0);
      rightAugmentation = -1;
    }
    else if ((char)key == 's') //display the eyear on the right
    {
      image.setTo(0);
      rightAugmentation = 0;
    }
    else if ((char)key == 'x') //display the sharpview on the right
    {
      image.setTo(0);
      rightAugmentation = 1;
    }
    else if ((char)key == 'c') //front pillar
    {
      image.setTo(0);
      pillarPosition = 0;
      correctionRight=-14;
      correctionLeft=-6;
      offsetY=-28;
      scale = 1.05f;
    }
    else if ((char)key == 'd') //mid pillar
    {
      image.setTo(0);
      pillarPosition = 1;      
      correctionRight=44;
      correctionLeft=-59;
      offsetY=0;
      scale = 1.f;
    }
    else if ((char)key == 'e') //back pillar
    {
      image.setTo(0);
      pillarPosition = 2;
      correctionRight=115;
      correctionLeft=-69;
      offsetY=0;
      scale = 1.f;
    }
    else if ((char)key == '1') //front pillar sharpView
    { 
      image.setTo(0);
      /*correctionRight--;*/
      //scale-=0.01;
    }
    else if ((char)key == '2') //front pillar sharpView
    {
      image.setTo(0);
      //correctionRight++;
      //scale+=0.01;
    }
    else if ((char)key == '3') //front pillar sharpView
    { 
      image.setTo(0);
      //correctionLeft--;
      //offsetY--;
    }
    else if ((char)key == '4') //front pillar sharpView
    {
      image.setTo(0);
      //correctionLeft++;
     // offsetY++;
    }
    else if ((char)key == 'u')
    {
      setFocusDistance(0.5f);
    }
    else if ((char)key == 'j')
    {
      setFocusDistance(0.4f);
    }
    else if ((char)key == 'n')
    {
      setFocusDistance(0.3f);
    }
  }
  printf("exit");
  image.release();

}
#else
void
ImageLoader::runProgram()
{
  cv::Mat interfaceImage = cv::imread("interface.png");
  finish = false;
  int size = 123;
  int offset = 20;
  float positionX = 0;
  int blur = 0;
  while(!finish)
  {
    if ((click.x) >=0)
    {
      if (click.y > 2 * interfaceImage.rows/3)
      {
        if (click.x < interfaceImage.cols/2)
        {
          char message[sizeof(int) +sizeof(int)];
          int messageID = 4;
          memcpy(message, &messageID, sizeof(int));
          int idd = 1;
          memcpy(message + sizeof(int), &idd, sizeof(int));
          server->sendToAll(message, sizeof(int) + sizeof(int));
          std::cout << "render no blur\n";
          blur = 0;
        }
        else
        {
          char message[sizeof(int) +sizeof(int)];
          int messageID = 4;
          memcpy(message, &messageID, sizeof(int));
          int idd = 0;
          memcpy(message + sizeof(int), &idd, sizeof(int));
          server->sendToAll(message, sizeof(int) + sizeof(int));
          std::cout << "render blur\n";
          blur = 1;
        }
      }
      else if (click.y > interfaceImage.rows/3)
      {
        if (click.x < interfaceImage.cols/2)
        {
          char message[sizeof(int) +sizeof(float)];
          int messageID = 3;
          memcpy(message, &messageID, sizeof(int));
          float idd = 1.4 + static_cast<float>(std::rand()%20 - 10)/200.f;
          positionX = idd;
          memcpy(message + sizeof(int), &idd, sizeof(float));
          server->sendToAll(message, sizeof(int) + sizeof(float));
          std::cout << "position X " << idd << "\n";
        }
        else
        {
          char message[sizeof(int) +sizeof(float)];
          int messageID = 3;
          memcpy(message, &messageID, sizeof(int));
          float idd = -1.35 + static_cast<float>(std::rand()%20 - 10)/200.f;
          positionX = idd;
          memcpy(message + sizeof(int), &idd, sizeof(float));
          server->sendToAll(message, sizeof(int) + sizeof(float));
          std::cout << "position X " << idd << "\n";
        }
      }
      else
      {
        int idX = click.x/(size + offset);
        int idY = click.y/ (size + offset);
        if (click.x - idX * (size + offset) < size && click.y - idY * (size + offset) < size)
        {
           interfaceImage = cv::imread("interface.png");
          
          
          int pos = idY * 10 + idX;
          std::cout << "moving to " << pos + 40 <<"\n";
          int movement = pos - currentPosition;
          currentPosition = pos;
          if (currentPosition == 0)
          {
            serial.writeString("H:1\r\n");
          }
          else
          {
            std::string message = "M:1";
            if (movement < 0)
            {
              message.append("-P");
            }
            else
            {
              message.append("+P");
            }
            movement = abs(movement);
            char buffer[4];
            std::sprintf(buffer, "%d", movement * 5000);
            message = message + buffer;
            message.append("\r\n");
            std::cout << "board info " << message <<":";
            serial.writeString(message);
            std::cout << "board info " << serial.readLine() <<":";
            serial.writeString("G:\r\n");
          }
          char message[sizeof(int) + sizeof(float)];
          int messageID = 1;
          float positionAndroid = static_cast<float>(currentPosition + 40)/100.0f;
          memcpy(message, &messageID, sizeof(int));
          memcpy(message + sizeof(int), &positionAndroid, sizeof(float));
          server->sendToAll(message, sizeof(int) + sizeof(float));
        }
      }
        interfaceImage = cv::imread("interface.png");
      if (blur == 0)
      {
        interfaceImage.rowRange(2*interfaceImage.rows/3, interfaceImage.rows).colRange(0, interfaceImage.cols/2).setTo(cv::Scalar(0,0,255));
      }
      else
      {
        interfaceImage.rowRange(2*interfaceImage.rows/3, interfaceImage.rows).colRange(interfaceImage.cols/2, interfaceImage.cols).setTo(cv::Scalar(0,0,255));
      }
      if (positionX < 0)
      {
        interfaceImage.rowRange(interfaceImage.rows/3, 2*interfaceImage.rows/3).colRange(interfaceImage.cols/2, interfaceImage.cols).setTo(cv::Scalar(0,0,255));
      }
      else
      {
           interfaceImage.rowRange(interfaceImage.rows/3, 2*interfaceImage.rows/3).colRange(0, interfaceImage.cols/2).setTo(cv::Scalar(0,0,255));
      }
      if (currentPosition == 20)
      {
          int idX = 10;
          int idY = 1;
          interfaceImage.rowRange(idY * (size + offset), idY * (size + offset) +size).colRange(
                              idX * (size + offset), std::min(interfaceImage.cols, idX * (size + offset)+size)).setTo(cv::Scalar(0,0,255));
      }
      else
      {
        int idX = currentPosition%10;
        int idY = currentPosition/10;
        interfaceImage.rowRange(idY * (size + offset), idY * (size + offset) +size).colRange(
                            idX * (size + offset), std::min(interfaceImage.cols, idX * (size + offset)+size)).setTo(cv::Scalar(0,0,255));
      }
      
      cv::imshow("Interface", interfaceImage);
      click.x = -1;
      loggerPresentation << lastMeasurement.elapsed() <<  delimiter << blur << delimiter << positionX << delimiter << currentPosition <<"\n";
    }
    int key = cv::waitKey(10);
    if (key == 27)
    {
      finish = true;
    }
    else if ((char)key == 'r')
    {
      openLoggers();
    }
  }
}
#endif
void
ImageLoader::stopProgram()
{
  finish = true;
  loggerPresentation.close();
  loggerDistance.close();
  loggerRadius.close();
}

void
ImageLoader::generateImage()
{
  //just generate images that will be loaded later on for testing
  for (float pupilSize = 1.00; pupilSize <= 8.00; pupilSize += 0.1)
  {
    for (float focusDistance = 0.35; focusDistance <= 0.7; focusDistance += 0.01)
    {
    cv::Mat image = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
    
    std::ostringstream fileNameNoSharpView;
    fileNameNoSharpView << std::fixed << std::setprecision(2);
    fileNameNoSharpView << "./imageDatabase/" << /*displayDistances[curDisplayDistance] << "_" <<*/ pupilSize << "_" << focusDistance << ".png";
    std::cout << fileNameNoSharpView.str() <<"\n";
    cv::putText(image, fileNameNoSharpView.str(), cv::Point(10, HEIGHT/2), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(255),2);
    cv::imwrite(fileNameNoSharpView.str(), image);
    std::ostringstream fileNameWithSharpView;
    fileNameWithSharpView << std::fixed << std::setprecision(2);
    fileNameWithSharpView << "./imageDatabase/sharpview/" << /*displayDistances[curDisplayDistance] << "_" <<*/ pupilSize << "_" << focusDistance << ".png";
    std::cout << fileNameWithSharpView.str() <<"\n";
    image = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
    cv::putText(image, fileNameNoSharpView.str(), cv::Point(10, HEIGHT/2), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(255),2);
    cv::imwrite(fileNameWithSharpView.str(), image);
    }
  }
}
void ImageLoader::moveLeft()
{
  offsetX--;

  cv::moveWindow("EyeARWindow", offsetX, offsetY);
    cv::waitKey(1);
    std::cout << offsetX <<"  " << offsetY <<"\n";
}
void ImageLoader::moveRight()
{
  offsetX++;
      cv::moveWindow("EyeARWindow", offsetX, offsetY);
    cv::waitKey(1);
    std::cout << offsetX <<"  " << offsetY <<"\n";
}
void ImageLoader::moveUp()
{
  offsetY--;
      cv::moveWindow("EyeARWindow", offsetX, offsetY);
    cv::waitKey(1);
    std::cout << offsetX <<"  " << offsetY <<"\n";
}
void ImageLoader::moveDown()
{
  offsetY++;
      cv::moveWindow("EyeARWindow", offsetX, offsetY);
    cv::waitKey(1);
    std::cout << offsetX <<"  " << offsetY <<"\n";
}

void ImageLoader::interpolation(float& radius, float&distance)
{
  float elapsed = static_cast<float>(lastMeasurement.elapsed());
 // printf("before interpolation: %5.3f, %5.3f, elapsed: %5.3f\n", radius, distance, elapsed);

  if (elapsed > 0.2f)
  {
    return;
  }
  
  curMeasurementRadius = std::min(8.1f, std::max(1.9f, lastMeasurementRadius + (radius  - lastMeasurementRadius) * elapsed/0.2f)); //this should be weighted by the jump size...
  curMeasurementDistance = std::min(0.55f, std::max(0.25f, lastMeasurementDistance + (distance - lastMeasurementDistance) * elapsed/0.2f)); //this should be weighted by the jump size...
  //printf("interpolation: %5.3f, %5.3f, %5.3f, %5.3f\n", lastMeasurementRadius, curMeasurementRadius, lastMeasurementDistance, curMeasurementDistance);
  radius = curMeasurementRadius;
  distance = curMeasurementDistance;
}
