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
  #define  THETA_MAX  ((edge_p)2.48)
  #define  THETA_MIN  ((edge_p)(0.17+CV_PI/2))
  #define  RHO_MAX  	((edge_p)150)

//Hough transform parameters
	#define  NUM_LINES 				((int)50)
	#define  RHO_RESULUTION 	((edge_p)1)
	#define  THETA_RESULUTION ((edge_p)CV_PI/180)
	#define HT_THRESHOLD 		((int)400)

//edge detecting parameters
	#define EDGE_THRESHOLD 	((int)120)

//#define  REDUCTION 			((double)0.5)


class streetEdge
{
	private:
		Mat last_measure;		//last measure of real street edge
		edge_t filtered_measure;
		Mat kernel;				//gradient kernel
		Mat gray_img; 			//part of the gray frame used to get the edges
		vector<edge_t> lines;	//vector used to store detected lines
		vector<int> accum;		//vector to store the score of the detected lines

		int thresholdH;
		int thresholdEdge;
		bool street_side;				//false for left side, true for right side
		KalmanFilter KF;

		//procesing methods
		void DetectEdges();
		void myHoughLines( const Mat &img,vector<edge_t> &lines,vector<int> &weights,
						edge_p rho, edge_p theta, int threshold,int linesMax );
		void criteriaFilter(vector<edge_t>&,vector<int>&);


		inline void Clear(){lines.clear();accum.clear();}



		void kalmanConfig(void);
		void coordinateConv(Mat&);


	public:
		// Constructores
		streetEdge(bool side,int TH_thres=HT_THRESHOLD, int TE_thres=EDGE_THRESHOLD,int num_lines=NUM_LINES);

		//gets
		edge_t GetEdge(const Mat&);
		inline edge_t GetEdge(){return filtered_measure;}
		inline bool GetSide(){ return street_side;}

		//set
		void SetThresholdHS(int,int);

	};

#endif /* STREETEDGE_H_ */

