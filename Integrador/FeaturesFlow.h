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

//######################################################################################################
//################################---------FeaturesFlow Class---------################################
//######################################################################################################


class FeaturesFlow
{
	protected:
	vector<Point2f> Prev;
	vector<Point2f> Next;
	vector<float> MatchError;
	size_t timeNext;

	public:
		//constructors
			FeaturesFlow(int MaxElements,size_t _time=0):Prev(MaxElements),Next(MaxElements),MatchError(MaxElements),timeNext(_time){}
			FeaturesFlow(vector<Point2f>& , vector<Point2f>& , vector<float>& , size_t);
			FeaturesFlow(vector<Point2f>& , vector<Point2f>& , vector<float>& , size_t, vector<uchar>&);
			FeaturesFlow(vector<Point2f>& , vector<Point2f>& , vector<float>& , size_t, vector<uchar>&,float);

		//Destructor
			virtual ~FeaturesFlow(){}


		//sets
			inline void SetTimeNext(size_t t){ timeNext=t;}
			inline void IncTimeNext(){timeNext++;}

		//gets
			inline size_t GetTimeNext(){return timeNext;}
			inline vector<Point2f>* GetPrevPtr(){return &Prev;}
			inline vector<Point2f>* GetNextPtr(){return &Next;}
			inline vector<float>* GetMatchErrorPtr(){return &MatchError;}
			inline virtual size_t GetNumOfFeatures(){return Prev.size();}


		//virtual
			inline virtual void Clear(){Prev.clear(); Next.clear(); MatchError.clear();}

			virtual bool FilterPts(float max_error){
				vector<Point2f>::iterator pp=Prev.begin();
				vector<Point2f>::iterator nn=Next.begin();
				vector<float>::iterator ee=MatchError.begin();
				size_t vec_size=Prev.size();

				for (size_t i = 0; i < vec_size; ++i){
					if(*ee>max_error){
						pp=Prev.erase(pp);
						nn=Next.erase(nn);
						ee=MatchError.erase(ee);
					}else{
						pp++;
						nn++;
						ee++;
					}
				}

				if (Prev.size()==0)
					return false;
				else
					return true;
			}

			virtual bool FilterPts(vector<uchar>& status){
				vector<Point2f>::iterator pp=Prev.begin();
				vector<Point2f>::iterator nn=Next.begin();
				vector<float>::iterator ee=MatchError.begin();
				size_t vec_size=Prev.size();

				for (size_t i = 0; i < vec_size; ++i){
					if(status[i]){
						pp=Prev.erase(pp);
						nn=Next.erase(nn);
						ee=MatchError.erase(ee);
					}else{
						pp++;
						nn++;
						ee++;
					}
				}

				if (Prev.size()==0)
					return false;
				else
					return true;
			}

			virtual bool FilterPts(float max_error,vector<uchar>& status){
				vector<Point2f>::iterator pp=Prev.begin();
				vector<Point2f>::iterator nn=Next.begin();
				vector<float>::iterator ee=MatchError.begin();
				size_t vec_size=Prev.size();

				for (size_t i = 0; i < vec_size; ++i){
					if(*ee>max_error && status[i]){
						pp=Prev.erase(pp);
						nn=Next.erase(nn);
						ee=MatchError.erase(ee);
					}else{
						pp++;
						nn++;
						ee++;
					}
				}

				if (Prev.size()==0)
					return false;
				else
					return true;
			}



};



//######################################################################################################
//################################---------FeaturesFlow3D Class---------################################
//######################################################################################################


class FeaturesFlow3D : public FeaturesFlow {

	protected:
		vector<Point3f> speed_3D;
		vector<Point3f> next_3D;
		Point3f 	speed_measure;

	public:
		//constructors
			FeaturesFlow3D(int MaxElements):FeaturesFlow(MaxElements),speed_3D(MaxElements),next_3D(MaxElements){}
			FeaturesFlow3D(vector<Point3f>& speed,vector<Point3f>& next3d, vector<Point2f>& previous, vector<Point2f>& next,
								vector<float>& error, size_t time) :FeaturesFlow(previous , next , error , time){
				speed_3D = speed; next_3D = next3d;}

			FeaturesFlow3D(vector<Point2f>& previous,vector<Point2f>& next, vector<float>& error, size_t time)
							:FeaturesFlow(previous,next,error,time){}

			FeaturesFlow3D(vector<Point2f>& previous,vector<Point2f>& next, vector<float>& error, size_t time,vector<uchar>& features_found,float max_match_error)
								:FeaturesFlow(previous,next,error,time,features_found,max_match_error){}

		//Destructor
			virtual ~FeaturesFlow3D(){}

		//sets
			void Setspeed_3D(vector<Point3f>);
			void Setnext_3D(vector<Point3f>);
			vector<Point3f> GetPrev3D();
			inline void SetSpeedMeasure(Point3f _measure){speed_measure=_measure;}

		//gets
			inline vector<Point3f> GetSpeedVector(void){ return speed_3D;}
			inline Point3f GetSpeed(int i){return speed_3D[i];}
			inline virtual size_t GetNumOfFeatures(){return next_3D.size();}
			inline vector<Point3f>* GetSpeed3DPtr(){return &speed_3D;}
			inline vector<Point3f>* GetNext3DPtr(){return &next_3D;}

		//virtual
			//virtual bool FilterPts(float max_error);
			//virtual bool FilterPts(vector<uchar>& status);
			//virtual bool FilterPts(float max_error,vector<uchar>& status);

			inline virtual void Clear(){FeaturesFlow::Clear();speed_3D.clear();next_3D.clear();}

};




//######################################################################################################
//################################---------StreetFeaturesFlow Class---------################################
//######################################################################################################


class StreetFeaturesFlow: public FeaturesFlow3D
{

	vector<float> reliability;
	int frame_rows;
	int frame_cols;

	//geometry parameters
		float y_floor;
		float f_camara;
		float x_wall;  //distance from street edge to the wall

	public:
		//constructors
			StreetFeaturesFlow(int MaxElements, float _y_floor,float _f_camara, float _x_wall):FeaturesFlow3D(MaxElements),
										reliability(MaxElements),
										y_floor(_y_floor),
										f_camara(_f_camara),
										x_wall(_x_wall){}
			StreetFeaturesFlow( vector<Point2f>& previous, vector<Point2f>& next,vector<float>& error, size_t time,
								Vec2f left_edge_param,Vec2f right_edge_param, int rows, int cols,vector<uchar>& features_found,
								 float _y_floor,float _f_camara, float _x_wall)
								:FeaturesFlow3D(previous,next,error,time),
								frame_rows(rows),
								frame_cols(cols),
								y_floor(_y_floor),
								f_camara(_f_camara),
								x_wall(_x_wall)
								{
				CalZparam(left_edge_param,right_edge_param);
			}

			StreetFeaturesFlow( vector<Point2f>& previous, vector<Point2f>& next,vector<float>& error, size_t time,
										Vec2f left_edge_param,Vec2f right_edge_param, int rows, int cols,vector<uchar>& features_found,
										float max_match_error,float _y_floor,float _f_camara, float _x_wall)
										:FeaturesFlow3D(previous,next,error,time,features_found,max_match_error),
										frame_rows(rows),
										frame_cols(cols),
										y_floor(_y_floor),
										f_camara(_f_camara),
										x_wall(_x_wall)
										{
						CalZparam(left_edge_param,right_edge_param);
					}


		//Destructor
			virtual ~StreetFeaturesFlow(){}


		//function with the geometrical info of the street
			void Cal3Dparam(Vec2f,Vec2f,int,int);
			void Cal3Dparam(Vec2f,Vec2f);			
			void CalZparam(Vec2f,Vec2f,int,int);
			void CalZparam(Vec2f,Vec2f);



		//set
			inline void SetReliability(vector<float> v){reliability=v;}
			inline void SetFrameSize(int rows,int cols){frame_rows=rows;frame_cols=cols;}

		//gets

			inline vector<float>* GetReliabilityPtr(){return &reliability;}
			inline float GetReliability(int i){return reliability[i];}

		//virtual
			//virtual bool FilterPts(float max_error);
			//virtual bool FilterPts(vector<uchar>& status);
			//virtual bool FilterPts(float max_error,vector<uchar>& status);

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
