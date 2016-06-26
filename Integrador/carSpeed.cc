/*
 * carSpeed.cc
 *
 *  Created on: 19/06/2016
 *      Author: ezequiel & Tobi
 */

#include "carSpeed.h"
#include  <algorithm> 

void CarSpeed::kalmanConfig(void){
    //KF.init(2,1);
    // intialization of KF...
	street_speed_KF.transitionMatrix = (Mat_<float>(2, 2) << 	1,1,
																0,.75);
    //Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));

	street_speed_KF.statePre.at<float>(0) = last_street_measure.at<float>(0);

    setIdentity(street_speed_KF.measurementMatrix,Scalar::all(1));
    setIdentity(street_speed_KF.processNoiseCov, Scalar::all(1e-4));
    //setIdentity(KF.measurementNoiseCov, Scalar::all(10));
    setIdentity(street_speed_KF.errorCovPost, Scalar::all(.1));
}
void CarSpeed::GetStreetMask(const Mat& frame){//it has frame as argument and not rows and cols due to in the future we can upgrade this method
										//e.g. we can eliminate the cars on the street
	street_mask= Mat::ones(frame.size(),CV_8U);
	Mat aux_mask=street_mask(Range(0,frame.rows/2),Range::all());
	aux_mask.setTo(0);
	int width=500;
	rectangle(street_mask,Point(frame.cols/2-width/2,frame.rows/2),Point(frame.cols/2+width/2,frame.rows/2+100),Scalar(0),CV_FILLED,8,0);
}

CarSpeed::CarSpeed(const Mat& frame, float _y_floor,float _f_camara,float _FPS, float _MAX_ST_SPEED, int _max_num_features,
				float _x_wall,int HT_threshold, unsigned char Edge_threshold,float max_match_error):
	left_edge(LEFT_SIDE,HT_threshold,Edge_threshold),
	right_edge(RIGHT_SIDE,HT_threshold,Edge_threshold),
	K_ST_SPEED(_y_floor*_f_camara*_FPS),
	MAX_ST_SPEED(_MAX_ST_SPEED),
	last_street_measure(1,1,0),
	MAX_MATCH_ERROR(max_match_error),
	street_speed_KF(2,1),
	max_num_features(_max_num_features),
	street_features(_max_num_features,_y_floor,_f_camara,_x_wall),
	aux_status(_max_num_features)
{
	cvtColor(frame,old_gray_frame,CV_BGR2GRAY);
	street_features.SetFrameSize(frame.rows,frame.cols);
	//get a mask for the street
	GetStreetMask(frame);
	kalmanConfig();
}


template <typename Pt3D>
static bool ZComp(Pt3D lhs,Pt3D rhs){return (lhs.z<rhs.z); }

//static bool ZComp(Point3f lhs,Point3f rhs){return (lhs.z<rhs.z); }

template <typename Pt3D>
static bool Point3DComp(Pt3D lhs,Pt3D rhs){
	return ((lhs.x*lhs.x + lhs.y*lhs.y + lhs.z*lhs.z )<(rhs.x*rhs.x + rhs.y*rhs.y + rhs.z*rhs.z )); }

template <typename Pt3D>
static Pt3D  nth_value(vector<Pt3D> v,float position, bool full_3d=false)
{
		//position is a % of vector size
    size_t n = (size_t)floor( v.size()*position);
    if (full_3d)
    {
    	nth_element(v.begin(), v.begin()+n, v.end(),Point3DComp<Pt3D>);
    }else{
    	nth_element(v.begin(), v.begin()+n, v.end(),ZComp<Pt3D>);
    }
    return v[n];
}

float CarSpeed::RangeAverage(float lower_spd,float higher_spd){
	float speed_acc=0;
	float reliability_acc=0;
	for (unsigned int i = 0; i < street_features.GetNumOfFeatures(); ++i){
		float speed=(street_features.GetSpeed(i)).z;
		if (lower_spd < speed && speed < higher_spd ){
			float rty=street_features.GetReliability(i);
			speed_acc+=speed*rty;
			reliability_acc+=rty;
		}
	}
	return speed_acc/reliability_acc;
}



float CarSpeed::GetStreetSpeed(const Mat& frame){
	//clean old data
		street_features.Clear();
		street_features.IncTimeNext();//first features have timNext=1, this is right, since the prevPTs belong to time=0
		aux_status.clear(); //necesario?

	//get the intensity channel
	  cvtColor(frame,gray_frame,CV_BGR2GRAY);

	//get the street edges from the gray frame 
	  Vec2f left_edge_param=left_edge.GetEdge(gray_frame);
	  Vec2f right_edge_param=right_edge.GetEdge(gray_frame);

	//get easily trackable points with goodFeaturesToTrack

		vector<Point2f>* prev_ptr=street_features.GetPrevPtr();

		goodFeaturesToTrack( old_gray_frame,*prev_ptr,max_num_features,QUALITY_LEVEL,MIN_DISTANCE ,street_mask,BLOCK_SIZE,false,K_HARRIS);
		
	// track them in the next frame with calcOpticalFlowPyrLK

		vector<Point2f>* next_ptr=street_features.GetNextPtr();
		vector<float>* error_ptr=street_features.GetMatchErrorPtr();

		calcOpticalFlowPyrLK( old_gray_frame, gray_frame, *prev_ptr, *next_ptr, aux_status, *error_ptr ,
		                     Size( WIN_SIZE, WIN_SIZE ), 5, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );

	//Filter bad points and calculate velocity based on ours hypothesis

		if(street_features.FeaturesFlow::FilterPts(MAX_MATCH_ERROR,aux_status)){
			//if we get new good measurements,last_street_measure is update
			street_features.CalZparam(left_edge_param,right_edge_param);

			//Get a 30-70 average, 30th is -MAX_ST_SPEED or more and 70th is MAX_ST_SPEED or less
			float lower_spd=max(nth_value<Point3f>(street_features.GetSpeedVector(),0.30).z,-MAX_ST_SPEED/K_ST_SPEED);
			float higher_spd=min(nth_value<Point3f>(street_features.GetSpeedVector(),0.70).z,MAX_ST_SPEED/K_ST_SPEED);
			float av_measure=RangeAverage(lower_spd,higher_spd);


			last_street_measure=(Mat_<float>(1,1)<< av_measure);
		}


    //push speed measure info to vector
			street_features.SetSpeedMeasure(Point3f(0,0,last_street_measure.at<float>(0)));

	//Filter measure with Kalman
		street_speed_KF.predict();
		last_street_speed=street_speed_KF.correct(last_street_measure);

#if 0
		gray_frame.copyTo(aux);
		//Mat color_old_frame;
		//cvtColor(old_gray_frame,color_old_frame,CV_GRAY2BGR);

		cvtColor(aux,aux,CV_GRAY2BGR);
		for( unsigned int i=0; i < aux_status.size(); i++ ){
			if(aux_status[i]&& (*error_ptr)[i]<10){
				Point pi( ceil( (*prev_ptr)[i].x ), ceil( (*prev_ptr)[i].y ) );
				Point pf( ceil( (*next_ptr)[i].x ), ceil( (*next_ptr)[i].y ) );
				//line( video, pi, pf, CV_RGB(255,0,0), 2 );
				circle(aux,pf,4,CV_RGB(0,255,0));
				line(aux, pi, pf, CV_RGB(255,0,0),2,8,0);

			}
		}
		//print left edge
			Point l1(gray_frame.cols/2,left_edge_param[0] +gray_frame.rows/2);
			Point l2(0,left_edge_param[0]-gray_frame.cols/2*left_edge_param[1]+gray_frame.rows/2);
			line(aux, l1, l2, CV_RGB(0,0,255),2,8,0);

		//print right_edge_param
			Point r1(gray_frame.cols/2,right_edge_param[0]+gray_frame.rows/2 );
			Point r2(gray_frame.cols-1,right_edge_param[0]+gray_frame.cols/2*right_edge_param[1]+gray_frame.rows/2);
			line(aux, r1, r2, CV_RGB(0,255,0),2,8,0);

		imshow("video",aux);
#endif
		
	//old_frame act
  old_gray_frame=gray_frame.clone();

	return last_street_speed.at<float>(0)*K_ST_SPEED;

}
	


