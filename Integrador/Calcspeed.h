/*
 * Calcspeed.h
 *
 *  Created on: 07/06/2016
 *      Author: ezequiel
 */


#ifndef CALCSPEED_H_
#define CALCSPEED_H_


#include <iostream>
#include <fstream>
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

using namespace std;
using namespace cv;

Mat GetPoints(Mat frame,Mat old_frame);

//void CalcVelocity(vector<Vec2f>&,vector<Vec2f>&,vector<float>&,Vec2f,Vec2f,float,int);

#endif /* CALCSPEED_H_ */
