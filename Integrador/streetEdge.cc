/*
 * streetEdge.cc
 *
 *  Created on: 02/06/2016
 *      Author: ezequiel & Tobi
 */

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "streetEdge.h"



//full constructor
streetEdge::streetEdge(bool side,int TH_thres, int TE_thres,int num_lines):
filtered_measure(0,CV_PI*3/4),
gray_img(1,1,CV_8U),
lines(num_lines),
accum(num_lines),
thresholdH(TH_thres),
thresholdEdge(TE_thres),
street_side(side),
KF(4,2)
{
	//set kernel used to get edges with the same direction of the main diagonal
	kernel=(Mat_<float>(5,5)<< 	0,-1,-1,-1,-1,
															1, 0,-1,-1,-1,
															1, 1, 0,-1,-1,
															1, 1, 1, 0,-1,
															1, 1, 1, 1, 0);

    last_measure=(Mat_<edge_p>(2,1)<< 0,CV_PI*3/4);
    kalmanConfig();
}


void streetEdge::SetThresholdHS(int TH,int TS)
{
	thresholdH = TH;
	thresholdEdge = TS;
}


edge_t streetEdge::GetEdge(const Mat &frame){
	Clear();	//clear vectors

    /*cut source image and flip if necesary*/
	if (street_side){//Right side
		gray_img=frame(Range(frame.rows/2,frame.rows),Range(frame.cols/2,frame.cols)).clone();
	}else{ //left side
        gray_img=frame(Range(frame.rows/2,frame.rows),Range(0,frame.cols/2)).clone();
		flip(gray_img,gray_img,1);
	}

    DetectEdges();

    myHoughLines( gray_img, lines, accum,RHO_RESULUTION, THETA_RESULUTION, thresholdH,NUM_LINES );  //detect lines in src image
    criteriaFilter(lines,accum);        //filter lines that don't seem to be a street edge

    KF.predict();                       
    Mat KF_output=KF.correct(last_measure);     //filter the measure with a kalman filter
    coordinateConv(KF_output);                  //get cartesian coordinates
    return filtered_measure;
}

void streetEdge::DetectEdges()
{
	equalizeHist(gray_img,gray_img);                           //equalize src img
	GaussianBlur(gray_img, gray_img, Size(5,5), 3.5, 3.5);     //filter gaussian noise
	filter2D(gray_img,gray_img,-1,kernel,Point(-1,-1));        //enhance diagonal edges
		//future improve, blurring and derivation together
	threshold(gray_img, gray_img,thresholdEdge, 255,THRESH_BINARY);    //filter non-edge pixels
}

void streetEdge::myHoughLines( const Mat &img,vector<edge_t> &lines,vector<int> &weights,
    edge_p rho, edge_p theta, int threshold,int linesMax )
{
	cv::AutoBuffer<int> _accum, _sort_buf;
  cv::AutoBuffer<edge_p> _tabSin, _tabCos;

  const uchar* image;
  int step, width, height;
	int numangle, numrho;
	int total = 0;
	int i, j;
	edge_p irho = 1 / rho;
  double scale;

	//CV_Assert( CV_IS_MAT(img) && CV_MAT_TYPE(img.type) == CV_8UC1 );
    image = img.data;
    step = img.step;
    width = img.cols;
    height = img.rows;

    numangle = cvRound(CV_PI / theta);
    numrho = cvRound(((width + height) * 2 + 1) / rho);
    _accum.allocate((numangle+2) * (numrho+2));
    _sort_buf.allocate(numangle * numrho);
    _tabSin.allocate(numangle);
    _tabCos.allocate(numangle);
    int *accum = _accum, *sort_buf = _sort_buf;
    edge_p *tabSin = _tabSin, *tabCos = _tabCos;

    memset( accum, 0, sizeof(accum[0]) * (numangle+2) * (numrho+2) );

	edge_p ang = 0;
    for(int n = 0; n < numangle; ang += theta, n++ )
    {
        tabSin[n] = (edge_p)(sin((double)ang) * irho);
        tabCos[n] = (edge_p)(cos((double)ang) * irho);
    }

    // stage 1. fill accumulator y could change lower and upper limit depending on the previous edge and some criteria
    for( i = 0; i < height; i++ )
        for( j = 0; j < width; j++ )
        {
            if( image[i * step + j] != 0 )
                for(int n = 0; n < numangle; n++ )
                {
                    int r = cvRound( j * tabCos[n] + i * tabSin[n] );
                    r += (numrho - 1) / 2;
                    accum[(n+1) * (numrho+2) + r+1]++;
                }
        }
    // stage 2. find local maximums
    for(int r = 0; r < numrho; r++ )
        for(int n = 0; n < numangle; n++ )
        {
            int base = (n+1) * (numrho+2) + r+1;
            if( accum[base] > threshold &&
                accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
                accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2] )
                sort_buf[total++] = base;
        }

    // stage 3. store the first min(total,linesMax) lines to the output buffer
    linesMax = MIN(linesMax, total);
    scale = 1./(numrho+2);
    for( i = 0; i < linesMax; i++ )
    {
    	edge_t line;
        int idx = sort_buf[i];
        int n = cvFloor(idx*scale) - 1;
        int r = idx - (n+1)*(numrho+2) - 1;
        line[0] = (r - (numrho - 1)*0.5f) * rho;
        line[1]= n * theta;
        lines.push_back( line );
        weights.push_back(accum[(n+1) * (numrho+2) + r+1]);
    }
}

void streetEdge::kalmanConfig(void){
    // Initialization of KF...
    KF.transitionMatrix = (Mat_<edge_p>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,.3,0,  0,0,0,.7);
    //Mat_<edge_p> measurement(2,1); measurement.setTo(Scalar(0));

    KF.statePre.at<edge_p>(0) = last_measure.at<edge_p>(0);   //rho
    KF.statePre.at<edge_p>(1) = last_measure.at<edge_p>(1);    //theta

    setIdentity(KF.measurementMatrix,Scalar::all(1));
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    //setIdentity(KF.measurementNoiseCov, Scalar::all(10));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
}

void streetEdge::criteriaFilter(vector<edge_t> &lines,vector<int> &accum){

    edge_p theta_max_detected=0;

    vector<edge_t>::iterator ll=lines.begin();
    vector<int>::iterator   aa=accum.begin();

    size_t num_lines=lines.size();

    /*delete lines that do not match the specifications:
        * the line can't be to horizontal. This implies a maximun distance of the edge to the car
        * the line can't be to vertical. This implies a minimun distance of the edge to the car
        * the must be close to the center of the image. This implies certain level of parallelism of the edge with the car's direction
    */
    for( size_t i = 0; i < num_lines && ll!=lines.end(); i++ ) {  
        edge_p rho =(*ll)[0] , theta = (*ll)[1];

        if(theta> THETA_MIN && theta < THETA_MAX && abs(rho)< RHO_MAX){
            ll++;
            aa++;
            if(theta>theta_max_detected) theta_max_detected=theta;
        }else{	//bad line
        	ll=lines.erase(ll);
        	aa=accum.erase(aa);
        }
    }

    /*get a measure.
        if the number of detected lines that seem to be edges is:
            0  : the new measure is equal to the last measure
            1  : the new measure is equal to this new line
            1+ : the new measure is equal to a weighted average (based on hough transform accumulator) of the group lines, within 50cm of the closer edge to the car
    */
    switch(lines.size()){
        case 0:
            break;
        case 1:
        	last_measure=(Mat_<edge_p>(2,1)<<	lines[0][0],lines[0][1]);
            break;
        default:
            //average of the closest lines to the car in a range of 50cm
            edge_p ac_rho=0,ac_theta=0;
            int acc_acc=0;
            for( size_t i = 0; i < lines.size(); i++ ) {
                if(lines[i][1]>(.80925*theta_max_detected+.32435)){  //aprox. of theta(x+.5)| x(thetamax_detected)
                        ac_rho+=lines[i][0]*accum[i];
                        ac_theta+=lines[i][1]*accum[i];
                        acc_acc+=accum[i];
                }

            }

            last_measure=(Mat_<edge_p>(2,1)<<ac_rho/acc_acc,ac_theta/acc_acc);
    }

}


void streetEdge::coordinateConv(Mat &bestGuess){
	int k=street_side?-1:1;
	filtered_measure=edge_t(bestGuess.at<edge_p>(0)/sin(bestGuess.at<edge_p>(1)) ,
					k*cos(bestGuess.at<edge_p>(1))/sin(bestGuess.at<edge_p>(1)) );
}

