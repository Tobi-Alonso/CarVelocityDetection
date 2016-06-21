/*
 * carSpeed.h
 *
 *  Created on: 19/06/2016
 *      Author: tobi
 */

#ifndef CARSPEED_H_
#define CARSPEED_H_



#include <iostream>
#include <fstream>
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "streetEdge.h"
//#include "Calcspeed.h"
#include "FeaturesFlow.h"

using namespace std;
using namespace cv;

class CarSpeed{
	//street identification
		streetEdge left_edge;
		streetEdge right_edge;

	Mat old_gray_frame;

	//street speed
		float last_street_measure; 	//	inicializar como 0
		const float MAX_MATCH_ERROR;
		KalmanFilter street_speed_KF;


		void kalmanConfig(void);
	public:
		CarSpeed(int HT_threshold,unsigned char Sobel_threshold,const Mat& frame,float max_match_error);

		~CarSpeed(){}

		//just z
		float GetStreetSpeed(Mat& frame);
		float GetStreetSpeed(Mat& frame ,size_t time_now,vector<StreetFeaturesFlow>&);

		//full3d
		float GetStreetSpeed(Mat& frame,float y_floor,float f_camara );
		float GetStreetSpeed(Mat& frame ,size_t time_now,vector<StreetFeaturesFlow>&,float y_floor,float f_camara );
};

	



#endif /* CARSPEED_H_ */
