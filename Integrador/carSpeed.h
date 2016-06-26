/*
 * carSpeed.h
 *
 *  Created on: 19/06/2016
 *      Author: ezequiel & Tobi
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
	Mat old_gray_frame;
	Mat gray_frame;


	//street identification
		streetEdge left_edge;
		streetEdge right_edge;

	//street speed
		float K_ST_SPEED;
		float MAX_ST_SPEED;
		Mat street_mask;
		Mat last_street_measure;
		Mat last_street_speed;
		const float MAX_MATCH_ERROR;

		KalmanFilter street_speed_KF;
		const size_t max_num_features;
		StreetFeaturesFlow street_features;
		vector<uchar> aux_status;

	//private methods
		void GetStreetMask(const Mat&);
		float RangeAverage(float ,float );
		void kalmanConfig(void);
	public:
		//constructor
			CarSpeed(const Mat& frame, float _y_floor,float _f_camara,float _FPS, float _MAX_ST_SPEED=150, int _max_num_features=100,
					float _x_wall=2,int HT_threshold=400, unsigned char Edge_threshold=120,float max_match_error=10.0);

		//destructor
		~CarSpeed(){}

		//gets
			//StreetFeaturesFlow* GetStreetFeaturesPtr(){return &street_features;}
				inline void GetStreetFeatures(vector<StreetFeaturesFlow>& v){v.push_back(street_features);}
				inline float GetLastStMeasure(){return last_street_measure.at<float>(0)*K_ST_SPEED;}
				inline Vec2f GetRightEdge(){Vec2f edge=right_edge.GetEdge();return edge;}
				inline Vec2f GetLeftEdge(){return left_edge.GetEdge();}

		//Method to get the speed of the car relative to the floor
			//just z
				float GetStreetSpeed(const Mat& frame);
				float GetStreetSpeed(const Mat& frame ,float FPS);

			//full3d
				//float GetStreetSpeed(const Mat& frame,float y_floor,float f_camara );
				//float GetStreetSpeed(const Mat& frame ,size_t time_now,float y_floor,float f_camara );


};

	


#endif /* CARSPEED_H_ */
