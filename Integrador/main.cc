

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include <iostream>
#include "Calcspeed.h"
#include "streetEdge.h"

using namespace cv;
using namespace std;

//parameters
  const int HTthreshold=400;
  const unsigned char Sobelthreshold=120;    //poner int, mas eficiente
  const float HeightCamera=1.3;
  const int fCamara=1685;     //Z coordinate in pixels to image plane  , need to calculate correctly
  const int FPS_DIV=2;


int main(int, char**)
{
	 VideoCapture cap("/home/tobi/Documentos/Facu/vision/ProyectoFinal/fuentes-proyecto/vid1.mp4"); // open the default camera
	//VideoCapture cap(0);
		if(!cap.isOpened())  // check if we succeeded
		        return -1;

	//window declarations
	    namedWindow("video",2);

  //program flags
  	bool loop=true;

  //video properties dependant constants
    const float FPS=cap.get(CV_CAP_PROP_FPS)/FPS_DIV;

  //instantiation of street edges detectors
		Street_edge	left_edge(HTthreshold,Sobelthreshold,LEFT_SIDE);
		Street_edge right_edge(HTthreshold,Sobelthreshold,RIGHT_SIDE);

	//discard first frames due to their lower quality
		for (int i = 0; i < 30; ++i){ cap.grab();}

  //get first frame to start process
		Mat frame,gray_frame;
		cap>>frame;
        cvtColor(frame,gray_frame,CV_BGR2GRAY);

  for(;loop;)
  {
    //old frame actualization
    	Mat old_gray_frame;
    	gray_frame.copyTo(old_gray_frame);

    //FPS division 
      for (int i = 0; i < FPS_DIV-1; ++i) cap.grab();
     
    // get a new frame from camera
      cap >> frame; 
      cvtColor(frame,gray_frame,CV_BGR2GRAY);

    //get the street edges from the gray frame 
      Vec2f LEdge=left_edge.GetEdge(gray_frame);
      Vec2f REdge=right_edge.GetEdge(gray_frame);

    float Speed = GetSpeed(gray_frame,old_gray_frame,LEdge,REdge);
    Speed*=HeightCamera*fCamara*FPS*3.6; //denormalize Speed to get km/hr

   	
    //Use of Kalman Filter to filter noise 

    cout<<"\n car's Speed:"<<Speed<<endl;

    char key=waitKey(1);

    switch (key) {
  		case 'q':
  			loop=false;
  			break;
  		case 's':
  			waitKey();
  			break;
  		case 'f':
  			int video_speed;
  			cout<<"\n Introduce video speed"<<endl;
  			cin>>video_speed;
  			for(;;){
  				for (int i = 0; i <video_speed-1; ++i){
  				            cap.grab();
  				        }
  				cap >> frame; // get a new frame from camera
  				imshow("video",frame);
  				if(waitKey(1) >= 0) break;
  			}
  			//do what needs to be done before coming back
  				cvtColor(frame,gray_frame,CV_BGR2GRAY);
  				equalizeHist(gray_frame,gray_frame);
  			break;
    }


     // if(waitKey(1) >= 0) break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}




