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
  #define HT_THRESHOLD 		((int)400)					//minimum Hough transform accumulator value to consider it find a line
  #define EDGE_THRESHOLD 	((int)120)					//minimum value of the derived image to consider a point as an edge
  #define HEIGHT_CAM 			((Feature3D_p)1.3)	//Y coordinate of the floor
  #define F_CAM 					((Feature3D_p)1685)	//Z coordinate in pixels to image plane
  #define FPS_DIV 				((int)3)						//value use to divide the camara's FPS
  #define X_WALL					((Feature3D_p)4)		//distance from the street edge to the wall of that side
  #define MAX_MATCH_ERROR ((Feature2D_p)10) 	//Lucas Kanade's algorithm maximum error to consider it find the point
  #define MAX_NUM_FEATURES ((int)125)					//Maximum number of features GoodFeatuersToTrack function finds
  #define MAX_ST_SPEED 		((Feature3D_p)120)	//Maximum speed of the car


int main(int, char**)
{
  //open video
   VideoCapture cap("/home/tobi/Documentos/Facu/vision/ProyectoFinal/fuentes-proyecto/vid1.mp4"); // open the camera/video
    if(!cap.isOpened())
            return -1; //fail to open camera/video


  namedWindow("video",2); //window declaration for video

  //program flags
    bool loop=true;
    bool draw=true;

  //video properties dependent constants
    const float FPS=cap.get(CV_CAP_PROP_FPS)/FPS_DIV; //FPS that the system uses

  //discard first frames due to their lower quality
    for (int i = 0; i < 30; ++i){ cap.grab();}

  //get first frame to start process
    Mat frame;
    if(!(cap.read(frame)))
    	loop=false;// no video, end process

  //instantiation of CarSpeed
    CarSpeed car_speed(frame,HEIGHT_CAM,F_CAM,FPS,MAX_ST_SPEED/3.6 ,MAX_NUM_FEATURES,X_WALL,
    		HT_THRESHOLD,EDGE_THRESHOLD,MAX_MATCH_ERROR);

   vector<StreetFeaturesFlow> features;		//instantiation of vector to store the features detected


  for(;loop;){ //process loop

    for (int i = 0; i < FPS_DIV-1; ++i) cap.grab();//FPS division

    if (!(cap.read(frame))) // get a new frame from camera
        break;// end of the video



    Feature3D_p Speed =3.6*car_speed.GetStreetSpeed(frame); // get speed in km/hr

    car_speed.GetStreetFeatures(features);	//store features tracked


    // show results
			if(draw){
				//features tracked
					//get features tracked
						vector<ImgPoint_t>* prev_ptr=features.back().GetPrevPtr();
						vector<ImgPoint_t>* next_ptr=features.back().GetNextPtr();

					//draw features detected
						for( unsigned int i=0; i < (*prev_ptr).size(); i++ ){
							Point pi( ceil( (*prev_ptr)[i].x ), ceil( (*prev_ptr)[i].y ) );
							Point pf( ceil( (*next_ptr)[i].x ), ceil( (*next_ptr)[i].y ) );
							circle(frame,pf,4,Scalar(200,200,0));
							line(frame, pi, pf, Scalar(0,50,255),2,8,0);
						}
				//Street edges
					//get street edges
						edge_t right_edge_param=car_speed.GetRightEdge();
						edge_t left_edge_param=car_speed.GetLeftEdge();

					//draw left edge
						Point l1(frame.cols/2,left_edge_param[0] +frame.rows/2);
						Point l2(0,left_edge_param[0]-frame.cols/2*left_edge_param[1]+frame.rows/2);
						line(frame, l1, l2, Scalar(200,100,0),2,8,0);

					//draw right edge
						Point r1(frame.cols/2,right_edge_param[0]+frame.rows/2 );
						Point r2(frame.cols-1,right_edge_param[0]+frame.cols/2*right_edge_param[1]+frame.rows/2);
						line(frame, r1, r2, Scalar(100,200,0),2,8,0);
			}


			//write speed in frame
				int intSpeed= (int)Speed;
				std::ostringstream text; text<<"Speed : "<<intSpeed<<"km/hr";
				putText(frame,text.str(),Point(frame.cols/2-470,10+100),FONT_HERSHEY_SIMPLEX,3,Scalar(255,185,10),15,CV_AA,false);
				putText(frame,text.str(),Point(frame.cols/2-470,10+100),FONT_HERSHEY_SIMPLEX,3,Scalar(0,0,0),4,CV_AA,false);

			imshow("video",frame); //show frame with results

			cout<<"\n Car speed:"<<Speed<<"		Last measure: "<<3.6*car_speed.GetLastStMeasure()<<endl; //show results in the console

  //menu
    char key=waitKey(1);

    switch (key) {
      case 'q':	//End process
        loop=false;
        break;
      case 's':	//Pause process
        waitKey();
        break;
      case 'd':	//switch between drawing or not street edges and features tracked
      	draw= not draw;
      	break;
    }

  }

  //show last frame with our names
		Mat last_frame= Mat::ones(1080,1940,CV_8U)*255;
		string header="Project develop by:";
		string tobi="Alonso, Tobias";
		string and_char="&";
		string eze="Flores, Ezequiel";
		putText(last_frame,header,Point(last_frame.cols/2-420,110),FONT_HERSHEY_SIMPLEX,3,Scalar(0,0,0),4,CV_AA,false);
		putText(last_frame,tobi,Point(last_frame.cols/2-300,310),FONT_HERSHEY_SIMPLEX,3,Scalar(0,0,0),4,CV_AA,false);
		putText(last_frame,and_char,Point(last_frame.cols/2-5,450),FONT_HERSHEY_SIMPLEX,3,Scalar(0,0,0),4,CV_AA,false);
		putText(last_frame,eze,Point(last_frame.cols/2-330,590),FONT_HERSHEY_SIMPLEX,3,Scalar(0,0,0),4,CV_AA,false);
		imshow("video",last_frame);
		cout<<"end , press a key to close the program"<<endl;
		waitKey();


  //store in file, sort, get from file

  return 0;
}

