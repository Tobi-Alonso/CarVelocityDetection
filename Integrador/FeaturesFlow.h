/*
 * FeaturesFlow.h
 *
 *  Created on: 17/06/2016
 *      Author: ezequiel & Tobi
 */

#ifndef FEATURESFLOW_H_
#define FEATURESFLOW_H_


#include <iostream>
#include <fstream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

#define Feature2D_p float //precision use for all 2D operation, just float for now
#define Feature3D_p	float //precision use for all 3D operation,float or double


//######################################################################################################
//################################---------FeaturesFlow Class---------################################
//######################################################################################################
/*
 * class design to store the information about easily traceable points in video's frames
 */



#define ImgPoint_t Point_<Feature2D_p>		//definition of the type used for image points

class FeaturesFlow
{
	protected:
	vector<ImgPoint_t > Prev;				//features detected in previous frame
	vector<ImgPoint_t > Next;				//same features found in current frame
	vector<Feature2D_p> MatchError;	//Matching error between Prev points and Next points
	size_t timeNext;								//time of current frame

	public:
		//constructors
			//just get enough memory
				FeaturesFlow(int MaxElements,size_t _time=0):Prev(MaxElements),Next(MaxElements),MatchError(MaxElements),timeNext(_time){}
			//to store attributes
				FeaturesFlow(vector<ImgPoint_t >& , vector<ImgPoint_t >& , vector<Feature2D_p>& , size_t);
			//to store the attributes that were actually tracked by calcOpticalFlowPyrLK
				FeaturesFlow(vector<ImgPoint_t >& , vector<ImgPoint_t >& , vector<Feature2D_p>& , size_t, vector<uchar>&);
			//to store the attributes that were actually tracked by calcOpticalFlowPyrLK and have a matching error below the provided threshold
				FeaturesFlow(vector<ImgPoint_t >& , vector<ImgPoint_t >& , vector<Feature2D_p>& , size_t, vector<uchar>&,Feature2D_p);

		//Destructor
			virtual ~FeaturesFlow(){}

		//sets
			inline void SetTimeNext(size_t t){ timeNext=t;}
			inline void IncTime(){timeNext++;}

		//gets
			inline size_t GetTimeNext(){return timeNext;}
			inline vector<ImgPoint_t >* GetPrevPtr(){return &Prev;}
			inline vector<ImgPoint_t >* GetNextPtr(){return &Next;}
			inline vector<Feature2D_p>* GetMatchErrorPtr(){return &MatchError;}
			inline virtual size_t GetNumOfFeatures(){return Prev.size();}

		//virtual
			inline virtual void Clear(){Prev.clear(); Next.clear(); MatchError.clear();}

			//method used to erase points that do not have a MatchError below the provided threshold
				virtual bool FilterPts(Feature2D_p max_error);
			//method used to erase points that were not actually tracked by calcOpticalFlowPyrLK
				virtual bool FilterPts(vector<uchar>& status);
			//method used to erase points that were not actually tracked by calcOpticalFlowPyrLK or do not have a MatchError below the provided threshold
				virtual bool FilterPts(Feature2D_p max_error,vector<uchar>& status);
};




//######################################################################################################
//################################---------FeaturesFlow3D Class---------################################
//######################################################################################################
/*
 * class design to store the information about easily traceable points in video's frames
 * which correspond to points in 3D space
 */



#define SpacePoint_t Point3_<Feature3D_p>		//definition of the type used for space Points

class FeaturesFlow3D : public FeaturesFlow {

	protected:
		vector<SpacePoint_t> speed_3D;	//3D coordinates of speed vector of each features
		vector<SpacePoint_t> next_3D;		//3D coordinates of features detected in current frame
		SpacePoint_t 	speed_measure;		//3D coordinates of average speed vector of all features

	public:
		//constructors
			//just get enough memory
				FeaturesFlow3D(int MaxElements):FeaturesFlow(MaxElements),speed_3D(MaxElements),next_3D(MaxElements){}
			//to store all attributes
				FeaturesFlow3D(vector<SpacePoint_t>& speed,vector<SpacePoint_t>& next3d, vector<ImgPoint_t >& previous, vector<ImgPoint_t >& next,
								vector<Feature2D_p>& error, size_t time) :FeaturesFlow(previous , next , error , time){
						speed_3D = speed; next_3D = next3d;}
			//to store just 2D attributes
				FeaturesFlow3D(vector<ImgPoint_t >& previous,vector<ImgPoint_t >& next, vector<Feature2D_p>& error, size_t time)
							:FeaturesFlow(previous,next,error,time){}
			//to store just the 2D attributes that were actually tracked by calcOpticalFlowPyrLK and have a matching error below the provided threshold
				FeaturesFlow3D(vector<ImgPoint_t >& previous,vector<ImgPoint_t >& next, vector<Feature2D_p>& error, size_t time,
						vector<uchar>& features_found,Feature2D_p max_match_error)
								:FeaturesFlow(previous,next,error,time,features_found,max_match_error){}

		//Destructor
			virtual ~FeaturesFlow3D(){}

		//sets
			void Setspeed_3D(vector<SpacePoint_t> speed){ speed_3D = speed;}
			void Setnext_3D(vector<SpacePoint_t> Next){next_3D = Next;}
			void GetPrev3D(vector<SpacePoint_t>& prev_3D);
			inline void SetSpeedMeasure(SpacePoint_t _measure){speed_measure=_measure;}

		//gets
			inline vector<SpacePoint_t> GetSpeedVector(void){ return speed_3D;}
			inline SpacePoint_t GetSpeed(int i){return speed_3D[i];}
			inline virtual size_t GetNumOfFeatures(){return next_3D.size();}
			inline vector<SpacePoint_t>* GetSpeed3DPtr(){return &speed_3D;}
			inline vector<SpacePoint_t>* GetNext3DPtr(){return &next_3D;}

		//virtual
			//future development, not jet defined
				//virtual bool FilterPts(Feature2D_p max_error);
				//virtual bool FilterPts(vector<uchar>& status);
				//virtual bool FilterPts(Feature2D_p max_error,vector<uchar>& status);

			inline virtual void Clear(){FeaturesFlow::Clear();speed_3D.clear();next_3D.clear();} //empty all vectors

};






//######################################################################################################
//################################---------StreetFeaturesFlow Class---------############################
//######################################################################################################
/*
 * class design to store the information about easily traceable points in video's frames
 * which correspond to points of a street and it's surroundings.
 * In order to get 3d information from 2D frames, we assume that this space has a known geometric description
 */




#include "streetEdge.h"

class StreetFeaturesFlow: public FeaturesFlow3D
{

	vector<Feature3D_p> reliability;	//vector that store the reliability of the 3D speed measure of a feature
	int frame_rows;										//number of rows of camera's frames
	int frame_cols;										//number of columns of camera's frames

	//geometry parameters
		float y_floor;									//distance from the camera to the floor
		float f_camara;									//Z coordinate in pixels to image plane
		float x_wall;  									//distance from street edge to the wall

	public:
		//constructors
			//get enough memory for vectores and set most parameters
				StreetFeaturesFlow(int MaxElements, Feature3D_p _y_floor,Feature3D_p _f_camara, Feature3D_p _x_wall):FeaturesFlow3D(MaxElements),
								reliability(MaxElements),
								y_floor(_y_floor),
								f_camara(_f_camara),
								x_wall(_x_wall){}
			//to store all attributes corresponding to 2D points that were actually tracked by calcOpticalFlowPyrLK

				StreetFeaturesFlow( vector<ImgPoint_t >& previous, vector<ImgPoint_t >& next,vector<Feature2D_p>& error, size_t time,
								edge_t left_edge_param,edge_t right_edge_param, int rows, int cols,vector<uchar>& features_found,
								 Feature3D_p _y_floor,Feature3D_p _f_camara, Feature3D_p _x_wall)
								:FeaturesFlow3D(previous,next,error,time),
								frame_rows(rows),
								frame_cols(cols),
								y_floor(_y_floor),
								f_camara(_f_camara),
								x_wall(_x_wall)
							{CalZparam(left_edge_param,right_edge_param);}

			//to store all attributes corresponding to 2D points that were actually tracked by calcOpticalFlowPyrLK and
			//have a matching error below the provided threshold
				StreetFeaturesFlow( vector<ImgPoint_t >& previous, vector<ImgPoint_t >& next,vector<Feature2D_p>& error, size_t time,
								edge_t left_edge_param,edge_t right_edge_param, int rows, int cols,vector<uchar>& features_found,
								Feature2D_p max_match_error,Feature3D_p _y_floor,Feature3D_p _f_camara, Feature3D_p _x_wall)
								:FeaturesFlow3D(previous,next,error,time,features_found,max_match_error),
								frame_rows(rows),
								frame_cols(cols),
								y_floor(_y_floor),
								f_camara(_f_camara),
								x_wall(_x_wall)
							{CalZparam(left_edge_param,right_edge_param);}


		//Destructor
			virtual ~StreetFeaturesFlow(){}


		//functions with the geometric info of the street
			void CalZparam(edge_t,edge_t); 					//calculates the Z coordinate of speed,of Next points, and reliability

			//future development, not jet defined
				void Cal3Dparam(edge_t,edge_t);				 	//calculates the 3D speed,3d coordinates of Next points, and reliability


		//set
			inline void SetReliability(vector<Feature3D_p> v){reliability=v;}
			inline void SetFrameSize(int rows,int cols){frame_rows=rows;frame_cols=cols;}
			//Y coordinate translation. y=0 form the center of frame to top of the frame
				void YCoordCorrect(){		for(int i=0;i<Prev.size();i++)
																				{Prev[i].y+=frame_rows/2;
																				Next[i].y+=frame_rows/2;}}

		//gets

			inline vector<Feature3D_p>* GetReliabilityPtr(){return &reliability;}
			inline Feature3D_p GetReliability(int i){return reliability[i];}

		//virtual
			//future development, not jet defined
				//virtual bool FilterPts(Feature2D_p max_error);
				//virtual bool FilterPts(vector<uchar>& status);
				//virtual bool FilterPts(Feature2D_p max_error,vector<uchar>& status);

			inline virtual void Clear(){FeaturesFlow3D::Clear();reliability.clear();}

		// Sobrecarga del operador de asignación
		//StreetFeaturesFlow& operator= (const StreetFeaturesFlow&);

		// Sobrecarga de operadores de comparación
	    friend bool operator== (const StreetFeaturesFlow&,const StreetFeaturesFlow&);
		friend bool operator!= (const StreetFeaturesFlow&,const StreetFeaturesFlow&);

		// Sobrecarga del operador menor, usado en ordenamiento
		/*inline bool operator< (const StreetFeaturesFlow& rhs)
		{bool minor;	minor = speed_3D.z<rhs.speed_3D.z:true,false;	return minor;}*/


		// Sobrecarga del operador de inserción en el flujo de salida
		friend ostream& operator<< (ostream&, const StreetFeaturesFlow&);
		friend istream& operator>> (istream&, StreetFeaturesFlow&);

		// Sobrecarga del operador de inserción en el flujo de salida
		friend ofstream& operator<< (ofstream&, const StreetFeaturesFlow&);
		friend ifstream& operator>> (ifstream&, StreetFeaturesFlow&);

};

#endif /* FEATURESFLOW_H_ */
