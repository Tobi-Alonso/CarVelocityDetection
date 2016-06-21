/*
 * carSpeed.cc
 *
 *  Created on: 19/06/2016
 *      Author: tobi
 */

#include "carSpeed.h"
#include  <algorithm> 

void CarSpeed::kalmanConfig(void){
    //KF.init(2,1);
    // intialization of KF...
	street_speed_KF.transitionMatrix = (Mat_<float>(2, 2) << 1,1,
    											0,.5);
    //Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));

	street_speed_KF.statePre.at<float>(0) = last_street_measure;

    setIdentity(street_speed_KF.measurementMatrix,Scalar::all(1));
    setIdentity(street_speed_KF.processNoiseCov, Scalar::all(1e-4));
    //setIdentity(KF.measurementNoiseCov, Scalar::all(10));
    setIdentity(street_speed_KF.errorCovPost, Scalar::all(.1));
}

CarSpeed::CarSpeed(int HT_threshold,unsigned char Sobel_threshold,const Mat& frame,float max_match_error):
	left_edge(HT_threshold,Sobel_threshold,LEFT_SIDE),
	right_edge(HT_threshold,Sobel_threshold,RIGHT_SIDE),
	last_street_measure(0),
	MAX_MATCH_ERROR(max_match_error),
	street_speed_KF(2,1)

{
	cvtColor(frame,old_gray_frame,CV_BGR2GRAY);
	kalmanConfig();
}
/*
template <typename Pt3D>
static bool ZComp(Pt3D lhs,Pt3D rhs){return (lhs.z<rhs.z); }*/

static bool ZComp(Point3f lhs,Point3f rhs){return (lhs.z<rhs.z); }


static bool Point3DComp(Point3f lhs,Point3f rhs){
	return ((lhs.x*lhs.x + lhs.y*lhs.y + lhs.z*lhs.z )<(rhs.x*rhs.x + rhs.y*rhs.y + rhs.z*rhs.z )); }

static Point3f  nth_value(vector<Point3f> v,float position, bool full_3d)///verifiar que usa una copia del vector
{
		//position is a % of vector size
    size_t n = (size_t)floor( v.size()*position);
    if (full_3d)
    {
    	nth_element(v.begin(), v.begin()+n, v.end(),Point3DComp);
    }else{
    	nth_element(v.begin(), v.begin()+n, v.end(),ZComp);
    }
    return v[n];
}

static void GetStreetMask(Mat& frame,Mat& mask){
	mask= Mat::ones(frame.size(),CV_8U);
	Mat aux_mask=mask(Range(0,frame.rows/2),Range::all());
	aux_mask.setTo(0);
	int width=500;
	rectangle(mask,Point(mask.cols/2-width/2,mask.rows/2),Point(mask.cols/2+width/2,mask.rows/2+100),Scalar(0),CV_FILLED,8,0);
}

float CarSpeed::GetStreetSpeed(Mat& frame ,size_t time_now, vector<StreetFeaturesFlow>& features){

	//get the intensity channel
		Mat gray_frame;
	  cvtColor(frame,gray_frame,CV_BGR2GRAY);

	//get the street edges from the gray frame 
	  Vec2f left_edge_param=left_edge.GetEdge(gray_frame);
	  Vec2f right_edge_param=right_edge.GetEdge(gray_frame);

	//get a mask for the street
	  Mat mask;
	  GetStreetMask(frame,mask);

	//get easily trackable points with goodFeaturesToTrack
		//parameters shi tomasi & Lucas Kanade
			const int win_size = 15;
			const int maxCorners = 125;
			const double qualityLevel = 0.01;
			const double minDistance = 10.0;
			const int blockSize = 3;
			const double k = 0.04;

		std::vector<cv::Point2f> prev_pts;
			prev_pts.reserve(maxCorners);

		goodFeaturesToTrack( old_gray_frame,prev_pts,maxCorners,qualityLevel,minDistance,mask,blockSize,false,k);
		
	// track them in the next frame with calcOpticalFlowPyrLK
		//variable declaration for calcOpticalFlowPyrLK
	    std::vector<uchar> features_found;
	    	features_found.reserve(maxCorners);

	    //	std::vector<uchar> features_found(maxCorners);    opcion

	    std::vector<float> feature_errors;
	    	feature_errors.reserve(maxCorners);

			std::vector<cv::Point2f> next_pts;
				next_pts.reserve(maxCorners);

		calcOpticalFlowPyrLK( old_gray_frame, gray_frame, prev_pts, next_pts, features_found, feature_errors ,
		                      Size( win_size, win_size ), 5, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );


	//instantiate class that calculates their velocity based on ours hypothesis


		StreetFeaturesFlow street_features(prev_pts,next_pts ,feature_errors,time_now,
				 left_edge_param, right_edge_param,frame.rows,frame.cols ,features_found,MAX_MATCH_ERROR);




		if(street_features.GetNumOfFeatures()==0){
			vector<float> r(1,0);
			street_features.SetReliability(r);
		}







	//Get a baised weighted average speed. the mean is baised because of ours hypothesis
		Mat measurement;

		if (street_features.GetReliability(0)){
			float lower_spd=nth_value(street_features.GetSpeedVector(),0.35,false).z;
			float higher_spd=nth_value(street_features.GetSpeedVector(),0.85,false).z;//,0.0012681021583098734); //always less than 150km/hr
			float speed_acc=0;
			float reliability_acc=0;
			for (unsigned int i = 0; i < street_features.GetNumOfFeatures(); ++i){
				float speed=(street_features.GetSpeed(i)).z;
				if (lower_spd < speed ||speed < higher_spd ){
					float rty=street_features.GetReliability(i);
					speed_acc+=speed*rty;
					reliability_acc+=rty;
				}

			}
			if(speed_acc/reliability_acc < 0.0012681021583098734){
				measurement=(Mat_<float>(1,1)<< speed_acc/reliability_acc);
			}else{
				measurement=(Mat_<float>(1,1)<< 0.0012681021583098734);
			}
			last_street_measure=measurement.at<float>(0);

		}else{
			measurement=(Mat_<float>(1,1)<< last_street_measure);
		}


    //push features info to vector
	  street_features.SetSpeedMeasure(Point3f(0,0,measurement.at<float>(0)));
		features.push_back(street_features);

	//Filter measure with Kalman
		street_speed_KF.predict();
    Mat bestGuess=street_speed_KF.correct(measurement);



#if 1	//draw them

		Mat aux;
		gray_frame.copyTo(aux);
		//Mat color_old_frame;
		//cvtColor(old_gray_frame,color_old_frame,CV_GRAY2BGR);

		cvtColor(aux,aux,CV_GRAY2BGR);
		for( unsigned int i=0; i < features_found.size(); i++ ){
			if(features_found[i]&&feature_errors[i]<10){
				Point pi( ceil( prev_pts[i].x ), ceil( prev_pts[i].y ) );
				Point pf( ceil( next_pts[i].x ), ceil( next_pts[i].y ) );
				//line( video, pi, pf, CV_RGB(255,0,0), 2 );
				circle(aux,pf,4,CV_RGB(0,255,0));
				line(aux, pi, pf, CV_RGB(255,0,0),2,8,0);

				//print left edge
					Point l1(gray_frame.cols/2,left_edge_param[0] +gray_frame.rows/2);
					Point l2(0,left_edge_param[0]-gray_frame.cols/2*left_edge_param[1]+gray_frame.rows/2);
					line(aux, l1, l2, CV_RGB(0,0,255),2,8,0);

				//print right_edge_param
					Point r1(gray_frame.cols/2,right_edge_param[0]+gray_frame.rows/2 );
					Point r2(gray_frame.cols-1,right_edge_param[0]+gray_frame.cols/2*right_edge_param[1]+gray_frame.rows/2);
					line(aux, r1, r2, CV_RGB(0,255,0),2,8,0);

			}
		}

		imshow("video",aux);
#endif
		
	//old_frame act
  old_gray_frame=gray_frame.clone();

	return bestGuess.at<float>(0);

}
	


