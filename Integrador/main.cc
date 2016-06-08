

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include <iostream>
#include "street_edge.h"

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

void CalcVelocity(vector<Vec2f>&,vector<Vec2f>&,vector<float>&,Vec2f,Vec2f,float,int);

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

    for(;loop;)
    {
    	Mat frame;
    	//olframe=frame
    	//cap.grab();
    	cap.grab();
        cap >> frame; // get a new frame from camera

        Vec2f Lline=left_edge.GetEdge(frame);
        Vec2f Rline=right_edge.GetEdge(frame);



     	//llamar a goodfeatures
     	//llamar calcflow
     	//determinar velocidad de punto



        imshow("video", frame);

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

void CalcVelocity(vector<Vec2f>& prevPts,vector<Vec2f>& nextPts,vector<float>& Vel,
        Vec2f leftEdge,Vec2f rightEdge,float yFloor,int fCamara,float DeltaTime,int rows,int cols){
	CV_Assert(Vel.empty());
    int a0left=leftEdge[0] +rows/2-leftEdge[1]*cols/2;
    int a0right=rightEdge[0] +rows/2-rightEdge[1]*cols/2;

    for (int i = 0; i < prevPts.size(); ++i){
        //check where is the point and calc velocity
        int x=prevPts[i][0];
        if (x <cols/2) {  //left side
            if (prevPts[i][1]<leftEdge[1]*x + a0left ){//on the wall
                Vel.push_back(yFloor*fCamara*(1/(leftEdge[0]+nextPts[i][0]*leftEdge[1])-1/(leftEdge[0]+x*leftEdge[1]))/DeltaTime);
            }else{
                Vel.push_back(yFloor*fCamara*(1/nextPts[i][1]-1/prevPts[i][1])/DeltaTime);
            }

        }else{//right side
            if (prevPts[i][1]<rightEdge[1]*x + a0right ){//on the wall
                Vel.push_back(yFloor*fCamara*(1/(rightEdge[0]+nextPts[i][0]*rightEdge[1])-1/(rightEdge[0]+x*rightEdge[1]))/DeltaTime);
            }else{
                Vel.push_back(yFloor*fCamara*(1/nextPts[i][1]-1/prevPts[i][1])/DeltaTime);
            }
        }

    }

}


