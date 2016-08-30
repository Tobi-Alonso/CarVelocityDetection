/*
 * streetEdge.h
 *
 *  Created on: 02/06/2016
 *      Author: Author: ezequiel & Tobi
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

#define edge_p float		//float or double
#define edge_t Vec<edge_p,2>

#define RIGHT_SIDE true
#define LEFT_SIDE false

//criteria filter parameters
  #define  THETA_MAX  ((edge_p)2.48)						//this value corresponds to a minimum edge distance to the car of 1 m.
  #define  THETA_MIN  ((edge_p)(0.17+CV_PI/2))	//this value corresponds to a maximum edge distance to the car of 7 m.
  #define  RHO_MAX  	((edge_p)150)							//coefficient that allows certain lack of parallelism of the edge with the car's direction

//Hough transform parameters
	#define  NUM_LINES 				((int)50)						//maximum number of lines detected by the Hough transform
	#define  RHO_RESULUTION 	((edge_p)1)					
	#define  THETA_RESULUTION ((edge_p)CV_PI/180)
  #define  HT_THRESHOLD 		((int)400)					//minimum Hough transform accumulator value to consider it find a line

//edge detector parameters
  #define EDGE_THRESHOLD 	((int)120)					//minimum value of the derived image to consider a point as an edge


class streetEdge
{
	private:
		Mat last_measure;					//last measure of real street edge
		edge_t filtered_measure;	//final measure of edge coefficients
		Mat kernel;								//gradient kernel
		Mat gray_img; 						//part of the gray frame used to get the edges
		vector<edge_t> lines;			//vector used to store detected lines
		vector<int> accum;				//vector to store the score of the detected lines

		int thresholdH;					
		int thresholdEdge;
		bool street_side;				//false for left side, true for right side
		KalmanFilter KF;				//object that implements a kalman filter, used on the edge's coefficients

		//procesing methods
		void DetectEdges();			//method that detects edges with the same direction of the main diagonal
		void myHoughLines( const Mat &img,vector<edge_t> &lines,vector<int> &weights,
						edge_p rho, edge_p theta, int threshold,int linesMax );	//method that implements the classic hough transform
		void criteriaFilter(vector<edge_t>&,vector<int>&);		//methond that filters the lines that don't match the edge specifications


		inline void Clear(){lines.clear();accum.clear();}			//method that clears the content of lines and accum vectors


		void kalmanConfig(void);			//method used to configure the kalman filter
		void coordinateConv(Mat&);		//method that takes lines coefficients in polar coordinates and returns them in cartesian cordinates


	public:
		// Constructor
		streetEdge(bool side,int TH_thres=HT_THRESHOLD, int TE_thres=EDGE_THRESHOLD,int num_lines=NUM_LINES);

		//gets
		edge_t GetEdge(const Mat&);		//top level method that returns edges coefficiet from a video
		inline edge_t GetEdge(){return filtered_measure;}  //method that returns the last measure
		inline bool GetSide(){ return street_side;}					

		//set
		void SetThresholdHS(int,int);

	};

#endif /* STREETEDGE_H_ */

