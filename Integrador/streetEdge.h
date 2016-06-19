/*
 * street_edge.h
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


class Street_edge
{
	private:
		Mat last_measure;		//best guess of real street edge
		Mat sobel_kernel;		//sobel kernel used in Canny

		int thresholdH;
		unsigned char thresholdS;
		bool street_side;				//false for left side, true for right side
		KalmanFilter KF;

		//procesing methods
		Mat imConditioning(Mat);
		Mat DetectEdges(Mat);
		void myHoughLines( const Mat &img,vector<Vec2f> &lines,vector<int> &weights,
						float rho, float theta, int threshold,int linesMax );
		void criteriaFilter(vector<Vec2f>&,vector<int>&,Mat&);
		// SetThings
		void SetThresholdHS(int,int);
		void kalmanConfig(void);
		Vec2f coordinateConv(Mat&);

	public:
		// Constructores
		Street_edge(void);
		Street_edge(int,unsigned char ,bool);

		Vec2f GetEdge(const Mat&);
		inline bool GetSide(){ return street_side;}

	};

#endif /* STREETEDGE_H_ */

