/*
 * Calcspeed.cpp
 *
 *  Created on: 07/06/2016
 *      Author: ezequiel
 */

#include "Calcspeed.h"

static void CalcVelocity(vector<Point2f>& prevPts,vector<Point2f>& nextPts,vector<uchar>& features_found,vector<float>& feature_errors,
		vector<float>& Vel, Vec2f leftEdge,Vec2f rightEdge,float yFloor,int fCamara,float DeltaTime,int rows,int cols){
	CV_Assert(Vel.empty());
    int a0left=leftEdge[0] +rows/2-leftEdge[1]*cols/2;
    int a0right=rightEdge[0] +rows/2-rightEdge[1]*cols/2;

    for (unsigned int i = 0; i < prevPts.size(); ++i){
    	if(features_found[i]&&feature_errors[i]<10){
			//check where is the point and calc velocity
			int x=prevPts[i].x;
			if (x <cols/2) {  //left side
				if (prevPts[i].y<leftEdge[1]*x + a0left ){//on the wall
					Vel.push_back(yFloor*fCamara*(1/(leftEdge[0]+nextPts[i].x*leftEdge[1])-1/(leftEdge[0]+x*leftEdge[1]))/DeltaTime);
				}else{
					Vel.push_back(yFloor*fCamara*(1/nextPts[i].y-1/prevPts[i].y)/DeltaTime);
				}

			}else{//right side
				if (prevPts[i].y<rightEdge[1]*x + a0right ){//on the wall
					Vel.push_back(yFloor*fCamara*(1/(rightEdge[0]+nextPts[i].x*rightEdge[1])-1/(rightEdge[0]+x*rightEdge[1]))/DeltaTime);
				}else{
					Vel.push_back(yFloor*fCamara*(1/nextPts[i].y-1/prevPts[i].y)/DeltaTime);
				}
			}
    	}
    }
}

float GetSpeed(Mat& frame,Mat& old_frame,Vec2f leftEdge,Vec2f rightEdge,float yFloor,int fCamara,float DeltaTime)
{
	//parametros shi tomasi
    const int win_size = 15;
    const int maxCorners = 125;
	const double qualityLevel = 0.01;
	const double minDistance = 10.0;
	const int blockSize = 3;
	const double k = 0.04;

	//Mat gray;
	//cvtColor(old_frame, gray, COLOR_BGR2GRAY);

	std::vector<cv::Point2f> PrevPts;
	PrevPts.reserve(maxCorners);

	//get easily trackable points
		goodFeaturesToTrack( old_frame,PrevPts,maxCorners,qualityLevel,minDistance,cv::Mat(),blockSize,false,k);
		//cornerSubPix( gray, PrevPts, Size( win_size, win_size ), Size( -1, -1 ),
		//		                   TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

	// track them in the next frame
	    std::vector<uchar> features_found;
	    features_found.reserve(maxCorners);

	    std::vector<float> feature_errors;
	    feature_errors.reserve(maxCorners);

		std::vector<cv::Point2f> NextPts;NextPts.reserve(maxCorners);
		calcOpticalFlowPyrLK( old_frame, frame, PrevPts, NextPts, features_found, feature_errors ,
		                      Size( win_size, win_size ), 5, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );

	//draw them
		Mat aux;
		frame.copyTo(aux);
		for( unsigned int i=0; i < features_found.size(); i++ ){
						if(features_found[i]&&feature_errors[i]<10){
							Point pi( ceil( PrevPts[i].x ), ceil( PrevPts[i].y ) );
							Point pf( ceil( NextPts[i].x ), ceil( NextPts[i].y ) );
							//line( video, pi, pf, CV_RGB(255,0,0), 2 );
							//circle(aux,pf,);
							line(aux, pi, pf, CV_RGB(255,0,0),2,8,0);
						}
					}
		imshow("video",aux);

	//Calculate their velocity based on ours hypothesis
		std::vector<float> Velocity;
		Velocity.reserve(maxCorners);

		CalcVelocity(PrevPts,NextPts,features_found,feature_errors,Velocity,leftEdge,rightEdge,
				yFloor,fCamara,DeltaTime,
				frame.rows,
				frame.cols
				);

	
	//Get the average Velocity
		float Vel_acc=0;
		for (unsigned int i = 0; i < Velocity.size(); ++i)
		{
			Vel_acc+=Velocity[i];
		}
		float Vel=Vel_acc/Velocity.size();

	
		

	// Make an image of the results
	/*
	for( unsigned int i=0; i < features_found.size(); i++ )
	{
	//cout<<"Error is "<<feature_errors[i]<<endl;
	//continue;
	//cout<<"Got it"<<endl;
		if(features_found[i]&&feature_errors[i]<10)
	    {
			Point p0( ceil( NextPts[i].x ), ceil( NextPts[i].y ) );
	        Point p1( ceil( PrevPts[i].x ), ceil( PrevPts[i].y ) );
	        line( frame, p0, p1, CV_RGB(0,0,255), 2 );
	    }
	}*/

	return Vel;
};



