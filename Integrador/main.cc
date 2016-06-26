/*
 *
 *  Created on: 17/06/2016
 *      Author: ezequiel & Tobi
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include <iostream>
#include <string>

#include "streetEdge.h"
#include "carSpeed.h"

using namespace cv;
using namespace std;

//system parameters
  #define HT_THRESHOLD 		((int)400)
  #define EDGE_THRESHOLD 	((unsigned char)120)
  #define HEIGHT_CAM 		((float)1.3)
  #define F_CAM 			((float)1685)		//Z coordinate in pixels to image plane  , need to calculate correctly
  #define FPS_DIV 			((int)2)
  #define X_WALL			((float)2.5)
  #define MAX_MATCH_ERROR 	((float)10)
  #define MAX_NUM_FEATURES 	((int)125)
  #define MAX_ST_SPEED 		((float)120)


int main(int, char**)
{
  //open video
   VideoCapture cap("/home/tobi/Documentos/Facu/vision/ProyectoFinal/fuentes-proyecto/vid1.mp4"); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
            return -2;

  //window declarations
      namedWindow("video",2);

  //program flags
    bool loop=true;
    bool draw=true;

  //video properties dependent constants
    const float FPS=cap.get(CV_CAP_PROP_FPS)/FPS_DIV;

  //discard first frames due to their lower quality
    for (int i = 0; i < 30; ++i){ cap.grab();}

  //get first frame to start process
    Mat frame;
    cap>>frame;

  //instantiation of CarSpeed
    CarSpeed car_speed(frame,HEIGHT_CAM,F_CAM,FPS,MAX_ST_SPEED/3.6 ,MAX_NUM_FEATURES,X_WALL,
    		HT_THRESHOLD,EDGE_THRESHOLD,MAX_MATCH_ERROR);


  vector<StreetFeaturesFlow> features;

  for(size_t now=0;loop;now++)
  {
    for (int i = 0; i < FPS_DIV-1; ++i) cap.grab();//FPS division

    cap >> frame;  // get a new frame from camera

    float Speed =3.6*car_speed.GetStreetSpeed(frame); // get speed in km/hr

    car_speed.GetStreetFeatures(features);


    // show results
			if(draw){
				vector<Point2f>* prev_ptr=features.back().GetPrevPtr();
				vector<Point2f>* next_ptr=features.back().GetNextPtr();
				Vec2f right_edge_param=car_speed.GetRightEdge();
				Vec2f left_edge_param=car_speed.GetLeftEdge();

				//draw features detected
				for( unsigned int i=0; i < (*prev_ptr).size(); i++ ){
						Point pi( ceil( (*prev_ptr)[i].x ), ceil( (*prev_ptr)[i].y ) );
						Point pf( ceil( (*next_ptr)[i].x ), ceil( (*next_ptr)[i].y ) );
						//line( video, pi, pf, CV_RGB(255,0,0), 2 );
						circle(frame,pf,4,Scalar(200,200,0));
						line(frame, pi, pf, Scalar(0,50,255),2,8,0);

				}
				//print left edge
					Point l1(frame.cols/2,left_edge_param[0] +frame.rows/2);
					Point l2(0,left_edge_param[0]-frame.cols/2*left_edge_param[1]+frame.rows/2);
					line(frame, l1, l2, Scalar(200,100,0),2,8,0);

				//print right_edge_param
					Point r1(frame.cols/2,right_edge_param[0]+frame.rows/2 );
					Point r2(frame.cols-1,right_edge_param[0]+frame.cols/2*right_edge_param[1]+frame.rows/2);
					line(frame, r1, r2, Scalar(100,200,0),2,8,0);


			}

			//write speed in frame
				int intSpeed=(int)Speed;
				int FracSpeed=(int)((int)(Speed*10)%10);

				std::ostringstream text;
				text<<"Speed :"<<intSpeed<<','<<FracSpeed<<"km/hr";
				//rectangle(frame,Point(frame.cols/2-470,120),Point(frame.cols/2+470,10),Scalar(255,255,255),CV_FILLED,8,0);
				putText(frame,text.str(),Point(frame.cols/2-470,10+100),FONT_HERSHEY_SIMPLEX,3,Scalar(255,185,10),15,CV_AA,false);
				putText(frame,text.str(),Point(frame.cols/2-470,10+100),FONT_HERSHEY_SIMPLEX,3,Scalar(0,0,0),4,CV_AA,false);

			imshow("video",frame);

			cout<<"\n Car speed:"<<Speed<<"		Last measure: "<<3.6*car_speed.GetLastStMeasure()<<endl;

  //menu
    char key=waitKey(1);

    switch (key) {
      case 'q':
        loop=false;
        break;
      case 's':
        waitKey();
        break;
      case 'd':
      	draw= not draw;
      	break;
    }

  }
  // the camera will be deinitialized automatically in VideoCapture destructor




  //ordenar, mandar a file

  return 0;
}

