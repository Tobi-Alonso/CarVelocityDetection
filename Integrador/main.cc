

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include <iostream>

#include "streetEdge.h"
#include "carSpeed.h"

using namespace cv;
using namespace std;

//sistem parameters
  #define HT_THRESHOLD 		((int)400)
  #define SOBEL_THRESHOLD 	((int)120)
  #define HEIGHT_CAM 		((float)1.3)
  #define F_CAM 			((int)1685)		//Z coordinate in pixels to image plane  , need to calculate correctly
  #define FPS_DIV 			((int)2)
  #define X_WALL			((float)2.5)
  #define MAX_MATCH_ERROR 	((float)10)
  #define MAX_NUM_FEATURES 	((int)125)

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



  //video properties dependant constants
    const float FPS=cap.get(CV_CAP_PROP_FPS)/FPS_DIV;
    const float K_SPEED= HEIGHT_CAM*F_CAM*FPS*3.6;

  //discard first frames due to their lower quality
    for (int i = 0; i < 30; ++i){ cap.grab();}

  //get first frame to start process
    Mat frame;
    cap>>frame;

  //instantiation of CarSpeed
    CarSpeed car_speed(HT_THRESHOLD,SOBEL_THRESHOLD,frame,MAX_MATCH_ERROR,
    		MAX_NUM_FEATURES,HEIGHT_CAM,F_CAM,X_WALL);


  vector<StreetFeaturesFlow> features;

  for(size_t now=0;loop;now++)
  {

      //FPS division 
        for (int i = 0; i < FPS_DIV-1; ++i) cap.grab();
       
      // get a new frame from camera
        cap >> frame;



    float Speed =K_SPEED*car_speed.GetStreetSpeed(frame,now); //denormalize Speed to get km/hr

    car_speed.GetStreetFeatures(features);
    cout<<"\n car's Speed:"<<Speed<<"		utlima medida: "<<K_SPEED*car_speed.GetLastStMeasure()<<endl;

  //menu
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
        break;
    }

  }
  // the camera will be deinitialized automatically in VideoCapture destructor




  //ordenar, mandar a file

  return 0;
}

