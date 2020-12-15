//Done by Nathan Shankar
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <cmath>
#include <stdlib.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include <string.h>
#include <opencv2/ml/ml.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <wiringPi.h>
using namespace std;
using namespace cv;
using namespace raspicam;
CascadeClassifier stop_cascade;
vector<int> histogramLane;
vector<int> histogramLaneEnd;
vector<Mat> detectAndDisplayWarning( Mat frame );
Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDup, frameFinalDup1, ROILane, ROILaneEnd; 
RaspiCam_Cv Camera;
stringstream ss;
int LeftLanePos, RightLanePos, frameCenter, laneCenter, laneEnd, Result;
Point2f Source[] = {Point2f(45,110),Point2f(315,110),Point2f(0,160), Point2f(360,160)};
Point2f Destination[] = {Point2f(100,0),Point2f(260,0),Point2f(100,240), Point2f(260,240)};
Point2f Signs[]= {Point2f(280,20),Point2f(360,20),Point2f(280,180), Point2f(360,180)};
void Setup( int argc, char **argv, RaspiCam_Cv &Camera)
{
     Camera.set (CAP_PROP_FRAME_WIDTH,  ("-w",argc,argv,360));
     Camera.set (CAP_PROP_FRAME_HEIGHT,  ("-h",argc,argv,240));
     Camera.set (CAP_PROP_BRIGHTNESS,  ("-br",argc,argv,60));
     Camera.set (CAP_PROP_CONTRAST,  ("-co",argc,argv,50));
     Camera.set (CAP_PROP_SATURATION,  ("-sa",argc,argv,70));
     Camera.set (CAP_PROP_GAIN,  ("-g",argc,argv,50));
     Camera.set (CAP_PROP_FPS,  ("-fps",argc,argv,100));
}
void Perspective()
{
     line(frame,Source[0], Source[1], Scalar(0,0,255), 2);
     line(frame,Source[1], Source[3], Scalar(0,0,255), 2);
     line(frame,Source[3], Source[2], Scalar(0,0,255), 2);
     line(frame,Source[2], Source[0], Scalar(0,0,255), 2);
     
     //line(frame,Signs[0], Signs[1], Scalar(255,0,0), 2);
     //line(frame,Signs[1], Signs[3], Scalar(255,0,0), 2);
     //line(frame,Signs[3], Signs[2], Scalar(255,0,0), 2);
     //line(frame,Signs[2], Signs[0], Scalar(255,0,0), 2);
     
     //line(frame,Destination[0], Destination[1], Scalar(0,255,0), 2);
     //line(frame,Destination[1], Destination[3], Scalar(0,255,0), 2);
     //line(frame,Destination[3], Destination[2], Scalar(0,255,0), 2);
     //line(frame,Destination[2], Destination[0], Scalar(0,255,0), 2);
     
     Matrix = getPerspectiveTransform(Source, Destination);
     warpPerspective(frame, framePers, Matrix, Size(360,240));
}

void Threshold()
{
     cvtColor(framePers, frameGray, COLOR_RGB2GRAY);
     inRange(frameGray, 73, 240, frameThresh);
     Canny(frameThresh, frameEdge, 100, 500, 3, false);
     add(frameThresh, frameEdge, frameFinal);
     cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
     cvtColor(frameFinal, frameFinalDup, COLOR_RGB2BGR);
     cvtColor(frameFinal, frameFinalDup1, COLOR_RGB2BGR);
}
void Histogram()
{
     histogramLane.resize(360);
     histogramLane.clear();
     
     for(int i=0; i<360; i++)  //frame.size().width
     {
          ROILane = frameFinalDup(Rect(i,140,1,100));
          divide(255, ROILane, ROILane);
          histogramLane.push_back((int)(sum(ROILane)[0])); 
     }
     
     histogramLaneEnd.resize(360);
     histogramLaneEnd.clear();
     
     for(int i=0; i<frame.size().width; i++)
     {
          ROILane = frameFinalDup1(Rect(i,0,1,240));
          divide(255, ROILaneEnd, ROILaneEnd);
          histogramLaneEnd.push_back((int)(sum(ROILaneEnd)[0])); 
     }
     laneEnd = sum(histogramLaneEnd)[0];
     cout<<"Lane End="<<laneEnd;
          
}
//Done by Nathan Shankar
void LaneFinder()
{
     vector<int>:: iterator LeftPtr;
     LeftPtr=max_element(histogramLane.begin(), histogramLane.begin()+180);
     LeftLanePos = distance(histogramLane.begin(), LeftPtr);
     
     vector<int>:: iterator RightPtr;
     RightPtr=max_element(histogramLane.begin()+180, histogramLane.end());
     RightLanePos = distance(histogramLane.begin(), RightPtr);
     
     line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0,255,0), 2);
     line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2);
}
void LaneCenter()
{
     laneCenter = (RightLanePos -LeftLanePos)/2 +LeftLanePos;
     frameCenter =180;
     line(frameFinal, Point2f(frameCenter,0), Point2f(frameCenter,240), Scalar(255,0,0), 3);
     
     Result= laneCenter-frameCenter; 
}
void Capture()
{
     Camera.grab();
     Camera.retrieve(frame);
     cvtColor(frame, frame, COLOR_BGR2RGB);
      
}      
int main(int argc, char **argv)
{
     wiringPiSetup();
     pinMode(21,OUTPUT);
     pinMode(22,OUTPUT);
     pinMode(23,OUTPUT);
     pinMode(24,OUTPUT);
     Setup(argc, argv, Camera);
     cout<<"Connecting to Camera"<<endl;
     if(!Camera.open())
     {
          cout<<"Failed to connect"<<endl;
          return -1;
     }
     cout<<"Camera ID="<<Camera.getId()<<endl;
     while(1)
     {
      auto start = std::chrono::steady_clock::now(); 
      Capture(); 
      Perspective();
      Threshold();
      Histogram();
      LaneFinder();
      LaneCenter();
      if (laneEnd > 3000)
      { digitalWrite(21,1);
        digitalWrite(22,1);
        digitalWrite(23,1);
        digitalWrite(24,0); 
        cout<<"Lane End"<<endl;  
      }
     
      if (Result == 0)
      { digitalWrite(21,0);
        digitalWrite(22,0);
        digitalWrite(23,0);
        digitalWrite(24,0); 
        cout<<"Forward"<<endl;  
      }
      
      else if (Result>0 && Result<10)
      { digitalWrite(21,1);
        digitalWrite(22,0);
        digitalWrite(23,0);
        digitalWrite(24,0); 
        cout<<"Right1"<<endl;  
      }
      
      else if (Result>=10 && Result<20)
      { digitalWrite(21,0);
        digitalWrite(22,1);
        digitalWrite(23,0);
        digitalWrite(24,0); 
        cout<<"Right2"<<endl;  
      }
      
      else if (Result>20)
      { digitalWrite(21,1);
        digitalWrite(22,1);
        digitalWrite(23,0);
        digitalWrite(24,0); 
        cout<<"Right3"<<endl;  
      }
      
      else if (Result<0 && Result>-10)
      { digitalWrite(21,0);
        digitalWrite(22,0);
        digitalWrite(23,1);
        digitalWrite(24,0); 
        cout<<"Left1"<<endl;  
      }
      
      else if (Result<=-10 && Result>-20)
      { digitalWrite(21,1);
        digitalWrite(22,0);
        digitalWrite(23,1);
        digitalWrite(24,0); 
        cout<<"Left2"<<endl;  
      }
      
      else if (Result<-20)
      { digitalWrite(21,0);
        digitalWrite(22,1);
        digitalWrite(23,1);
        digitalWrite(24,0); 
        cout<<"Left3"<<endl;  
      }
      
      if(laneEnd > 3000)
      {
       ss.str(" ");
       ss.clear();
       ss<<"Lane End = ";
       putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(255,0,255), 2);
      }
      else if (Result == 0)
      {
       ss.str(" ");
       ss.clear();
       ss<<"Result = "<<Result<<" (Move Forward)";
       putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
      }
      else if (Result > 0)
      {
       ss.str(" ");
       ss.clear();
       ss<<"Result = "<<Result<<" (Move Right)";
       putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
      }
      else if (Result < 0)
      {
       ss.str(" ");
       ss.clear();
       ss<<"Result = "<<Result<<" (Move Left)";
       putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
      }
      
      namedWindow("original", WINDOW_KEEPRATIO);
      moveWindow("original",0,100);
      resizeWindow("original",360,240);
      imshow("original", frame);
      
      namedWindow("Perspective", WINDOW_KEEPRATIO);
      moveWindow("Perspective",360,100);
      resizeWindow("Perspective",360,240);
      imshow("Perspective", framePers);
      
      namedWindow("Gray", WINDOW_KEEPRATIO);
      moveWindow("Gray",720,100);
      resizeWindow("Gray",360,240);
      imshow("Gray", frameGray);
      
      namedWindow("Edge", WINDOW_KEEPRATIO);
      moveWindow("Edge",0,360);
      resizeWindow("Edge",360,240);
      imshow("Edge", frameEdge);
      
      namedWindow("Final", WINDOW_KEEPRATIO);
      moveWindow("Final",360,360);
      resizeWindow("Final",360,240);
      imshow("Final", frameFinal);
      
      waitKey(1);
      auto end = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed_seconds = end-start;
      float t=elapsed_seconds.count();
      int FPS = 1/t;
      cout<<"FPS="<<FPS<<endl;
      
      
     }
     return 0;
     
}
//Done by Nathan Shankar