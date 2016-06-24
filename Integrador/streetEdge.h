/*
 * streetEdge.h
 *
 *  Created on: 02/06/2016
 *      Author: tobi
 */

#ifndef STREETEDGE_H_
#define STREETEDGE_H_

#include <iostream>
#include <fstream>
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

using namespace std;
using namespace cv;
//using namespace cv::ocl;

#define RIGHT_SIDE true
#define LEFT_SIDE false

//criteria filter parameters
  #define  THETA_MAX  ((float)2.48)
  #define  THETA_MIN  ((float)(0.17+CV_PI/2))
  #define  RHO_MAX  	((float)150)

//Hough transform parameters
#define  NUM_LINES 				((int)50)
#define  RHO_RESULUTION 	((float)1)
#define  THETA_RESULUTION ((float)CV_PI/180)
//#define  REDUCTION 			((double)0.5)


class streetEdge
{
	private:
		Mat last_measure;		//last measure of real street edge
		Mat filtered_measure;
		Mat kernel;				//gradient kernel
		Mat gray_img; 			//part of the gray frame used to get the edges
		vector<Vec2f> lines;	//vector used to store detected lines
		vector<int> accum;		//vector to store the score of the detected lines

		int thresholdH;
		unsigned char thresholdS;
		bool street_side;				//false for left side, true for right side
		KalmanFilter KF;

		//procesing methods
		void DetectEdges();
		void myHoughLines( const Mat &img,vector<Vec2f> &lines,vector<int> &weights,
						float rho, float theta, int threshold,int linesMax );
		void criteriaFilter(vector<Vec2f>&,vector<int>&);


		inline void Clear(){lines.clear();accum.clear();}



		// SetThings
		void kalmanConfig(void);
		Vec2f coordinateConv(Mat&,Vec2f&);


	public:
		// Constructores
		streetEdge(void);
		streetEdge(int,unsigned char ,bool);

		Vec2f GetEdge(const Mat&);
		inline bool GetSide(){ return street_side;}
		void SetThresholdHS(int,int);

	};

#endif /* STREETEDGE_H_ */

