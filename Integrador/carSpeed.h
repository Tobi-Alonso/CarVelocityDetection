/*
 * carSpeed.h
 *
 *  Created on: 19/06/2016
 *      Author: ezequiel & Tobi
 */

#ifndef CARSPEED_H_
#define CARSPEED_H_

//######################################################################################################
//################################---------CarSpeed Class---------######################################
//######################################################################################################
/* Class design to compute the speed of a car relative to cars and the floor. (currently it just computes
 * the relative speed to the floor)
 *
 */

#include <iostream>
#include <fstream>
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "streetEdge.h"
#include "FeaturesFlow.h"

using namespace std;
using namespace cv;


//Shi-Tomasi & Lucas Kanade parameters
#define WIN_SIZE ((int)15)
#define QUALITY_LEVEL ((float)0.01)
#define MIN_DISTANCE ((float)10.0)
#define	BLOCK_SIZE ((int)3)
#define K_HARRIS ((float)0.04)


class CarSpeed{
	Mat gray_frame;  			//last frame passed to the object
	Mat old_gray_frame;		//Previous frame passed to the object

	//street identification
		streetEdge left_edge;			//object that computes the position of the left street edge
		streetEdge right_edge; 		//object that computes the position of the right street edge

	//street speed
		Feature3D_p K_ST_SPEED;			//constant used to get speed in m/s from the detected position variation
		Feature3D_p MAX_ST_SPEED;		//Max speed in m/s we know the car can reach
		Mat street_mask;						//mask use to avoid detection points that can throw a bad measure of street speed
		Mat last_street_measure;		//last street speed value measured from video's frames
		Mat last_street_speed;			//last street speed value delivered by Kalman filter
		const Feature3D_p MAX_MATCH_ERROR;	//max error acceptable to considered that a feature was detected in the next frame by calcOpticalFlowPyrLK function
		KalmanFilter street_speed_KF;				//objects that implements a Kalman filter
		const size_t max_num_features;			//Max number of features to be detected by goodFeaturesToTrack function
		StreetFeaturesFlow street_features;	//object that stores and computes the info about each feature detected. the most important: their 3D coordinates
		vector<uchar> aux_status;						//vector used to store calcOpticalFlowPyrLK status output vector avoiding to allocate and deallocate memory in each call of the GetStreetSpeed function

	//private methods
		void GetStreetMask(const Mat&);			//method that computes street_mask
		Feature3D_p RangeAverage(Feature3D_p ,Feature3D_p );	//method that computes the average of the value between a lower and higher bound
		void kalmanConfig(void);				//method use to configure the kalman filter when a CarSpeed object is instantiated


	public:
		//constructor
			CarSpeed(const Mat& frame, Feature3D_p _y_floor,Feature3D_p _f_camara,float _FPS, Feature3D_p _MAX_ST_SPEED=150, int _max_num_features=100,
					Feature3D_p _x_wall=2,int HT_threshold=400, int Edge_threshold=120,Feature2D_p max_match_error=10.0);

		//destructor
		~CarSpeed(){}

		//gets
				inline void GetStreetFeatures(vector<StreetFeaturesFlow>& v){v.push_back(street_features);}  			//method that pushes to a passed vector the StreetFeaturesFlow object
				inline Feature3D_p GetLastStMeasure(){return last_street_measure.at<Feature3D_p>(0)*K_ST_SPEED;}	//method that returns the last_street_measure value
				inline edge_t GetRightEdge(){edge_t edge=right_edge.GetEdge();return edge;}												//method that returns the right street edge parameters
				inline edge_t GetLeftEdge(){return left_edge.GetEdge();}																					//method that returns the left street edge parameters

		//Method to get the speed of the car relative to the floor
			//just z
				Feature3D_p GetStreetSpeed(const Mat& frame);		//method that computes the z coordinate car's relative speed to the street

			//full3d, future development, not jet defined
				//SpacePoint_t GetStreetSpeed3D(const Mat& frame);	//method that computes the car's relative speed vector to the street


};



#endif /* CARSPEED_H_ */
