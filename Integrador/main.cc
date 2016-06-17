

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include <iostream>
#include "street_edge.h"
#include "Calcspeed.h"

using namespace cv;
using namespace std;

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

    //parameters
    	const int HTthreshold=400;
    	const unsigned char Sobelthreshold=120;    //poner int, mas eficiente
        const float HeightCamera=1.3;
        const int fCamara=1685;     //Z coordinate in pixels to image plane  , need to calculate correctly
        const int FPS_DIV=2;
        const float DeltaTime=FPS_DIV/cap.get(CV_CAP_PROP_FPS);

    //instantiation of street edges detectors
		Street_edge	left_edge(HTthreshold,Sobelthreshold,LEFT_SIDE);
		Street_edge right_edge(HTthreshold,Sobelthreshold,RIGHT_SIDE);

	//discard first frames due to their lower quality
		for (int i = 0; i < 30; ++i){ cap.grab();}

		Mat frame,gray_frame;
		cap>>frame;
        cvtColor(frame,gray_frame,CV_BGR2GRAY);
        equalizeHist(gray_frame,gray_frame);

    for(;loop;)
    {
    	Mat old_gray_frame;
    	gray_frame.copyTo(old_gray_frame);
  

      	for (int i = 0; i < FPS_DIV-1; ++i){
            cap.grab();
        }
        
        cap >> frame; // get a new frame from camera

        cvtColor(frame,gray_frame,CV_BGR2GRAY);

        //get the street edges from the gray frame not ecualized
            Vec2f LEdge=left_edge.GetEdge(gray_frame);
            Vec2f REdge=right_edge.GetEdge(gray_frame);


        equalizeHist(gray_frame,gray_frame);

        float Velocity = GetSpeed(gray_frame,old_gray_frame,LEdge,REdge,HeightCamera,fCamara,DeltaTime);
     	
        //Use of Kalman Filter to filter noise 

        cout<<"\n car's Velocity:"<<Velocity*3.6<<endl;

        //cout<<"Left coef"<<LEdge[0]<<" y "<< LEdge[1]<<endl;
       // cout<<"Right coef"<<REdge[0]<<" y "<< REdge[1]<<endl;

        //imshow("video", frame);
        //imshow("video", GetPoints(frame,old_frame));

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




