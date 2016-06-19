
/*
 * Calcspeed.cpp
 *
 *  Created on: 07/06/2016
 *      Author: ezequiel
 */

#include "Calcspeed.h"


//yFloor*fCamara/DeltaTime se repite en todos, aplicarlo a la velocidad promedio.

static void CalcSpeed(vector<Point2f>& prevPts,vector<Point2f>& nextPts,vector<uchar>& features_found,vector<float>& feature_errors,
		vector<float>& Vel, Vec2f leftEdge,Vec2f rightEdge,int rows,int cols){
	CV_Assert(Vel.empty());
    int a0left=leftEdge[0] +rows/2-leftEdge[1]*cols/2;
    int a0right=rightEdge[0] +rows/2-rightEdge[1]*cols/2;

    for (unsigned int i = 0; i < prevPts.size(); ++i){
    	if(features_found[i]&&feature_errors[i]<10){
				//check where is the point and calc velocity
				int x=prevPts[i].x;
				if (x <cols/2) {  //left side
					if (prevPts[i].y<leftEdge[1]*x + a0left ){//on the wall
						Vel.push_back((1/(leftEdge[0]+(x-cols/2)*leftEdge[1])-1/(leftEdge[0]+(nextPts[i].x-cols/2)*leftEdge[1])));
					}else{
						Vel.push_back((1/(prevPts[i].y-rows/2)-1/(nextPts[i].y-rows/2)));
					}

				}else{//right side
					if (prevPts[i].y<rightEdge[1]*x + a0right ){//on the wall
						Vel.push_back((1/(rightEdge[0]+(x-cols/2)*rightEdge[1])-1/(rightEdge[0]+(nextPts[i].x-cols/2)*rightEdge[1])));
					}else{
						Vel.push_back((1/(prevPts[i].y-rows/2)-1/(nextPts[i].y-rows/2)));
					}
				}
    	}

    }
}


float GetSpeed(Mat& frame,Mat& old_frame,Vec2f leftEdge,Vec2f rightEdge)
{

	//get a mask to avoid bad points
		Mat mask(frame.size(),CV_8U,Scalar(255));
		Mat aux_mask=mask(Range(0,frame.rows/2),Range::all());
		aux_mask= aux_mask*0;
		int width=500;
		rectangle(mask,Point(mask.cols/2-width/2,mask.rows/2),Point(mask.cols/2+width/2,mask.rows/2+100),Scalar(0),CV_FILLED,8,0);


	//get easily trackable points
		//parametros shi tomasi
			const int win_size = 15;
			const int maxCorners = 125;
			const double qualityLevel = 0.01;
			const double minDistance = 10.0;
			const int blockSize = 3;
			const double k = 0.04;

		std::vector<cv::Point2f> PrevPts;
			PrevPts.reserve(maxCorners);

		goodFeaturesToTrack( old_frame,PrevPts,maxCorners,qualityLevel,minDistance,mask,blockSize,false,k);
	
	
	// track them in the next frame
    std::vector<uchar> features_found;
    	features_found.reserve(maxCorners);

    std::vector<float> feature_errors;
    	feature_errors.reserve(maxCorners);

		std::vector<cv::Point2f> NextPts;
			NextPts.reserve(maxCorners);

		calcOpticalFlowPyrLK( old_frame, frame, PrevPts, NextPts, features_found, feature_errors ,
		                      Size( win_size, win_size ), 5, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );

	//Calculate their velocity based on ours hypothesis
		std::vector<float> speed;
			speed.reserve(maxCorners);

		CalcSpeed(PrevPts,NextPts,features_found,feature_errors,speed,leftEdge,rightEdge,
				frame.rows,frame.cols);

#if 1	//draw them

		Mat aux;
		frame.copyTo(aux);
		Mat color_old_frame;
		cvtColor(old_frame,color_old_frame,CV_GRAY2BGR);

		cvtColor(aux,aux,CV_GRAY2BGR);
		for( unsigned int i=0; i < features_found.size(); i++ ){
			if(features_found[i]&&feature_errors[i]<10){
				Point pi( ceil( PrevPts[i].x ), ceil( PrevPts[i].y ) );
				Point pf( ceil( NextPts[i].x ), ceil( NextPts[i].y ) );
				//line( video, pi, pf, CV_RGB(255,0,0), 2 );
				circle(aux,pf,4,CV_RGB(0,255,0));
				line(aux, pi, pf, CV_RGB(255,0,0),2,8,0);

				//print left edge
					Point l1(frame.cols/2,leftEdge[0] +frame.rows/2);
					Point l2(0,leftEdge[0]-frame.cols/2*leftEdge[1]+frame.rows/2);
					line(aux, l1, l2, CV_RGB(0,0,255),2,8,0);

				//print rightedge
					Point r1(frame.cols/2,rightEdge[0]+frame.rows/2 );
					Point r2(frame.cols-1,rightEdge[0]+frame.cols/2*rightEdge[1]+frame.rows/2);
					line(aux, r1, r2, CV_RGB(0,255,0),2,8,0);

			}
		}

		imshow("video",aux);
#endif

	
	//Get the average Velocity
		if (speed.empty()){
			//excepcion
		}

		float speed_acc=0;
		for (unsigned int i = 0; i < speed.size(); ++i){
			speed_acc+=speed[i];
		}
		float av_speed=speed_acc/speed.size();

		

	return av_speed;
};



