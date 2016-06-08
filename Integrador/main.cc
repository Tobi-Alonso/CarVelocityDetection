

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include <iostream>
#include "street_edge.h"
#include "Calcspeed.h"

using namespace cv;
using namespace std;

#define DIBUJAR_LINEA(edge,side){\
	Point pt1, pt2;		\
	double a = edge[0], b = edge[1];\
	pt1.x = 0;\
	pt1.y = a;\
	pt2.x = frame.cols;\
	pt2.y = a + b*frame.cols;\
	line(  frame, pt1, pt2, Scalar(0,0,255), 3, CV_AA);\
}



int main(int, char**)
{
	 VideoCapture cap("/home/tobi/Documentos/Facu/vision/fuentes-proyecto/vid1.mp4"); // open the default camera
	//VideoCapture cap(0);
		if(!cap.isOpened())  // check if we succeeded
		        return -1;

	//window declarations
	    namedWindow("video",2);

    //program flags
    	bool loop=true;

    //parameters
    	const int HTthreshold=400;
    	const unsigned char Sobelthreshold=120;

    //instantiation of street edges detectors
		Street_edge	left_edge(HTthreshold,Sobelthreshold,LEFT_SIDE);
		Street_edge right_edge(HTthreshold,Sobelthreshold,RIGHT_SIDE);

	//discard first frames due to their lower quality
		for (int i = 0; i < 30; ++i)
		{
			cap.grab();
		}

		Mat frame;
		cap>>frame;

    for(;loop;)
    {
    	Mat old_frame;
    	frame.copyTo(old_frame);
    	//olframe=frame
    	//cap.grab();
    	cap.grab();
        cap >> frame; // get a new frame from camera

        Vec2f Lline=left_edge.GetEdge(frame);
        Vec2f Rline=right_edge.GetEdge(frame);



     	//llamar a goodfeatures
     	//llamar calcflow
     	//determinar velocidad de punto



        //imshow("video", frame);
        imshow("video", GetPoints(frame,old_frame));

        char key=waitKey(1);

        switch (key) {
			case 'q':
				loop=false;
				break;
			case 's':
				waitKey();
				break;

        }


       // if(waitKey(1) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}




