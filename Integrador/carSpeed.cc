/*
 * carSpeed.cc
 *
 *  Created on: 19/06/2016
 *      Author: ezequiel & Tobi
 */

#include "carSpeed.h"
#include  <algorithm> 

void CarSpeed::kalmanConfig(void){
    // intialization of KF...
	street_speed_KF.transitionMatrix = (Mat_<Feature3D_p>(2, 2) << 	1,1,
																0,.75);
  Mat_<Feature3D_p> measurement(2,1); measurement.setTo(Scalar(10));

	street_speed_KF.statePre.at<Feature3D_p>(0) = last_street_measure.at<Feature3D_p>(0);

    setIdentity(street_speed_KF.measurementMatrix,Scalar::all(1));
    setIdentity(street_speed_KF.processNoiseCov, Scalar::all(1e-4));
    //setIdentity(KF.measurementNoiseCov, Scalar::all(10));
    setIdentity(street_speed_KF.errorCovPost, Scalar::all(.1));
}

void CarSpeed::GetStreetMask(const Mat& frame){//it has frame as argument and not rows and cols due to in the future we can upgrade this method
										//e.g. we can eliminate the cars on the street
	street_mask= Mat::ones(frame.rows/2,frame.cols,CV_8U);
	int width=500;
	//mask the vanishing point due to the sensitivity of it's surroundings
		rectangle(street_mask,Point(frame.cols/2-width/2,0),Point(frame.cols/2+width/2,100),Scalar(0),CV_FILLED,8,0);
}

CarSpeed::CarSpeed(const Mat& frame, Feature3D_p _y_floor,Feature3D_p _f_camara,float _FPS, Feature3D_p _MAX_ST_SPEED, int _max_num_features,
				Feature3D_p _x_wall,int HT_threshold, int Edge_threshold,Feature2D_p max_match_error):
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
		street_features.SetFrameSize(frame.rows,frame.cols);
		//get a mask for the street
		GetStreetMask(frame);
		cvtColor(frame(Range(frame.rows/2,frame.rows),Range::all()),old_gray_frame,CV_BGR2GRAY);
		kalmanConfig();
}


// function that returns true if lhs's z coordinate is lower than rhs's
static bool ZComp(SpacePoint_t lhs,SpacePoint_t rhs){return (lhs.z<rhs.z); }

// function that returns true if lhs's L2 norm is lower than rhs's
static bool Point3DComp(SpacePoint_t lhs,SpacePoint_t rhs){
	return ((lhs.x*lhs.x + lhs.y*lhs.y + lhs.z*lhs.z )<(rhs.x*rhs.x + rhs.y*rhs.y + rhs.z*rhs.z )); }

//function that returns the nth lower value of the SpacePoint_t vector
static SpacePoint_t  nth_value(vector<SpacePoint_t> v,float position, bool full_3d=false)
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

//method that computes the average of the value between a lower and higher bound
Feature3D_p CarSpeed::RangeAverage(Feature3D_p lower_speed,Feature3D_p higher_speed){
	Feature3D_p speed_acc=0;
	Feature3D_p reliability_acc=0;
	for (unsigned int i = 0; i < street_features.GetNumOfFeatures(); ++i){
		Feature3D_p speed=(street_features.GetSpeed(i)).z;
		if (lower_speed < speed && speed < higher_speed ){
			Feature3D_p rty=street_features.GetReliability(i);
			speed_acc+=speed*rty;
			reliability_acc+=rty;
		}
	}
	return speed_acc/reliability_acc;
}



Feature3D_p CarSpeed::GetStreetSpeed(const Mat& frame){
	//clean old data
		street_features.Clear();
		street_features.IncTime();//first features have timNext=1, this is right, since the prevPTs belong to time=0
		aux_status.clear(); //necesario?

	//get the intensity channel
	  cvtColor(frame,gray_frame,CV_BGR2GRAY);

	//get the street edges from the gray frame 
	  Vec2f left_edge_param=left_edge.GetEdge(gray_frame);
	  Vec2f right_edge_param=right_edge.GetEdge(gray_frame);

	  gray_frame=gray_frame(Range(frame.rows/2,frame.rows),Range::all());

	//get easily trackable points with goodFeaturesToTrack

		vector<ImgPoint_t>* prev_ptr=street_features.GetPrevPtr();



		goodFeaturesToTrack( old_gray_frame,*prev_ptr,max_num_features,QUALITY_LEVEL,MIN_DISTANCE ,street_mask,BLOCK_SIZE,false,K_HARRIS);
		
	// track them in the next frame with calcOpticalFlowPyrLK

		vector<ImgPoint_t>* next_ptr=street_features.GetNextPtr();
		vector<Feature2D_p>* error_ptr=street_features.GetMatchErrorPtr();

		calcOpticalFlowPyrLK( old_gray_frame, gray_frame, *prev_ptr, *next_ptr, aux_status, *error_ptr ,
		                     Size( WIN_SIZE, WIN_SIZE ), 5, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );



	//Filter bad points and calculate velocity based on ours hypothesis

		if(street_features.FeaturesFlow::FilterPts(MAX_MATCH_ERROR,aux_status)){
			//if we get new good measurements,last_street_measure is update
			street_features.YCoordCorrect();
			street_features.CalZparam(left_edge_param,right_edge_param);

			//Get a 30-70 average, 30th is -MAX_ST_SPEED or more and 70th is MAX_ST_SPEED or less
			Feature3D_p lower_spd=max(nth_value(street_features.GetSpeedVector(),0.30).z,-MAX_ST_SPEED/K_ST_SPEED);
			Feature3D_p higher_spd=min(nth_value(street_features.GetSpeedVector(),0.70).z,MAX_ST_SPEED/K_ST_SPEED);
			Feature3D_p av_measure=RangeAverage(lower_spd,higher_spd);


			last_street_measure=(Mat_<Feature3D_p>(1,1)<< av_measure);
		}


    //push speed measure info to vector
			street_features.SetSpeedMeasure(SpacePoint_t(0,0,last_street_measure.at<Feature3D_p>(0)));

	//Filter measure with Kalman
		street_speed_KF.predict();
		last_street_speed=street_speed_KF.correct(last_street_measure);

		
	//old_frame act
  old_gray_frame=gray_frame.clone();

	return last_street_speed.at<Feature3D_p>(0)*K_ST_SPEED;

}
	


