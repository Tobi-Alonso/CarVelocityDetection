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

#define Feature2D_p float //just float for now
#define Feature3D_p	float //float or double


//######################################################################################################
//################################---------FeaturesFlow Class---------################################
//######################################################################################################


#define ImgPoint_t Point_<Feature2D_p>

class FeaturesFlow
{
	protected:
	vector<ImgPoint_t > Prev;
	vector<ImgPoint_t > Next;
	vector<Feature2D_p> MatchError;
	size_t timeNext;

	public:
		//constructors
			FeaturesFlow(int MaxElements,size_t _time=0):Prev(MaxElements),Next(MaxElements),MatchError(MaxElements),timeNext(_time){}
			FeaturesFlow(vector<ImgPoint_t >& , vector<ImgPoint_t >& , vector<Feature2D_p>& , size_t);
			FeaturesFlow(vector<ImgPoint_t >& , vector<ImgPoint_t >& , vector<Feature2D_p>& , size_t, vector<uchar>&);
			FeaturesFlow(vector<ImgPoint_t >& , vector<ImgPoint_t >& , vector<Feature2D_p>& , size_t, vector<uchar>&,Feature2D_p);

		//Destructor
			virtual ~FeaturesFlow(){}


		//sets
			inline void SetTimeNext(size_t t){ timeNext=t;}
			inline void IncTimeNext(){timeNext++;}

		//gets
			inline size_t GetTimeNext(){return timeNext;}
			inline vector<ImgPoint_t >* GetPrevPtr(){return &Prev;}
			inline vector<ImgPoint_t >* GetNextPtr(){return &Next;}
			inline vector<Feature2D_p>* GetMatchErrorPtr(){return &MatchError;}
			inline virtual size_t GetNumOfFeatures(){return Prev.size();}


		//virtual
			inline virtual void Clear(){Prev.clear(); Next.clear(); MatchError.clear();}

			virtual bool FilterPts(Feature2D_p max_error){
				vector<ImgPoint_t >::iterator pp=Prev.begin();
				vector<ImgPoint_t >::iterator nn=Next.begin();
				vector<Feature2D_p>::iterator ee=MatchError.begin();
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
				vector<ImgPoint_t >::iterator pp=Prev.begin();
				vector<ImgPoint_t >::iterator nn=Next.begin();
				vector<Feature2D_p>::iterator ee=MatchError.begin();
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

			virtual bool FilterPts(Feature2D_p max_error,vector<uchar>& status){
				vector<ImgPoint_t >::iterator pp=Prev.begin();
				vector<ImgPoint_t >::iterator nn=Next.begin();
				vector<Feature2D_p>::iterator ee=MatchError.begin();
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

#define SpacePoint_t Point3_<Feature3D_p>

class FeaturesFlow3D : public FeaturesFlow {

	protected:
		vector<SpacePoint_t> speed_3D;
		vector<SpacePoint_t> next_3D;
		SpacePoint_t 	speed_measure;

	public:
		//constructors
			FeaturesFlow3D(int MaxElements):FeaturesFlow(MaxElements),speed_3D(MaxElements),next_3D(MaxElements){}
			FeaturesFlow3D(vector<SpacePoint_t>& speed,vector<SpacePoint_t>& next3d, vector<ImgPoint_t >& previous, vector<ImgPoint_t >& next,
								vector<Feature2D_p>& error, size_t time) :FeaturesFlow(previous , next , error , time){
				speed_3D = speed; next_3D = next3d;}

			FeaturesFlow3D(vector<ImgPoint_t >& previous,vector<ImgPoint_t >& next, vector<Feature2D_p>& error, size_t time)
							:FeaturesFlow(previous,next,error,time){}

			FeaturesFlow3D(vector<ImgPoint_t >& previous,vector<ImgPoint_t >& next, vector<Feature2D_p>& error, size_t time,vector<uchar>& features_found,Feature2D_p max_match_error)
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
			//virtual bool FilterPts(Feature2D_p max_error);
			//virtual bool FilterPts(vector<uchar>& status);
			//virtual bool FilterPts(Feature2D_p max_error,vector<uchar>& status);

			inline virtual void Clear(){FeaturesFlow::Clear();speed_3D.clear();next_3D.clear();}

};






//######################################################################################################
//################################---------StreetFeaturesFlow Class---------################################
//######################################################################################################

#ifndef edge_t
#define edge_t Vec<float, 2>
#endif

class StreetFeaturesFlow: public FeaturesFlow3D
{

	vector<Feature3D_p> reliability;
	int frame_rows;
	int frame_cols;

	//geometry parameters
		float y_floor;
		float f_camara;
		float x_wall;  //distance from street edge to the wall

	public:
		//constructors
			StreetFeaturesFlow(int MaxElements, Feature3D_p _y_floor,Feature3D_p _f_camara, Feature3D_p _x_wall):FeaturesFlow3D(MaxElements),
										reliability(MaxElements),
										y_floor(_y_floor),
										f_camara(_f_camara),
										x_wall(_x_wall){}
			StreetFeaturesFlow( vector<ImgPoint_t >& previous, vector<ImgPoint_t >& next,vector<Feature2D_p>& error, size_t time,
								edge_t left_edge_param,edge_t right_edge_param, int rows, int cols,vector<uchar>& features_found,
								 Feature3D_p _y_floor,Feature3D_p _f_camara, Feature3D_p _x_wall)
								:FeaturesFlow3D(previous,next,error,time),
								frame_rows(rows),
								frame_cols(cols),
								y_floor(_y_floor),
								f_camara(_f_camara),
								x_wall(_x_wall)
								{
				CalZparam(left_edge_param,right_edge_param);
			}

			StreetFeaturesFlow( vector<ImgPoint_t >& previous, vector<ImgPoint_t >& next,vector<Feature2D_p>& error, size_t time,
										edge_t left_edge_param,edge_t right_edge_param, int rows, int cols,vector<uchar>& features_found,
										Feature2D_p max_match_error,Feature3D_p _y_floor,Feature3D_p _f_camara, Feature3D_p _x_wall)
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
			void Cal3Dparam(edge_t,edge_t,int,int);
			void Cal3Dparam(edge_t,edge_t);
			void CalZparam(edge_t,edge_t,int,int);
			void CalZparam(edge_t,edge_t);



		//set
			inline void SetReliability(vector<Feature3D_p> v){reliability=v;}
			inline void SetFrameSize(int rows,int cols){frame_rows=rows;frame_cols=cols;}

		//gets

			inline vector<Feature3D_p>* GetReliabilityPtr(){return &reliability;}
			inline Feature3D_p GetReliability(int i){return reliability[i];}

		//virtual
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
