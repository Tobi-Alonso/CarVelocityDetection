/*
 * Calcspeed.cpp
 *
 *  Created on: 07/06/2016
 *      Author: ezequiel
 */

#include "Calcspeed.h"

Mat GetPoints(Mat frame,Mat old_frame)
{
	//parametros shi tomasi
    int win_size = 15;
    int maxCorners = 125;
	double qualityLevel = 0.01;
	double minDistance = 10.0;
	int blockSize = 3;
	double k = 0.04;
	Mat gray;

	std::vector<cv::Point2f> cornersB;
	cornersB.reserve(maxCorners);
	cvtColor(old_frame, gray, COLOR_BGR2GRAY);

	goodFeaturesToTrack( gray,cornersB,maxCorners,qualityLevel,minDistance,cv::Mat(),blockSize,false,k);
	cornerSubPix( gray, cornersB, Size( win_size, win_size ), Size( -1, -1 ),
			                   TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

    std::vector<uchar> features_found;
    features_found.reserve(maxCorners);
    std::vector<float> feature_errors;
    feature_errors.reserve(maxCorners);

	std::vector<cv::Point2f> cornersA;cornersA.reserve(maxCorners);
	calcOpticalFlowPyrLK( old_frame, frame, cornersB, cornersA, features_found, feature_errors ,
	                      Size( win_size, win_size ), 5, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );



	//CalcVelocity(vector<Vec2f>&,vector<Vec2f>&,vector<float>&,Vec2f,Vec2f,float,int);


	// Make an image of the results
	for( unsigned int i=0; i < features_found.size(); i++ )
	{
	//cout<<"Error is "<<feature_errors[i]<<endl;
	//continue;
	//cout<<"Got it"<<endl;
		if(features_found[i]&&feature_errors[i]<10)
	    {
			Point p0( ceil( cornersA[i].x ), ceil( cornersA[i].y ) );
	        Point p1( ceil( cornersB[i].x ), ceil( cornersB[i].y ) );
	        line( frame, p0, p1, CV_RGB(0,0,255), 2 );
	    }
	}
	return frame;
};

static void CalcVelocity(vector<Vec2f>& prevPts,vector<Vec2f>& nextPts,vector<float>& Vel,
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


