/*
 * FeaturesFlow.h
 *
 *  Created on: 17/06/2016
 *      Author: ezequiel
 */

#ifndef FEATURESFLOW_H_
#define FEATURESFLOW_H_


#include <iostream>
#include <fstream>
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

using namespace cv;
using namespace std;

class FeaturesFlow
{
	protected:
	vector<Point2f> Prev;
	vector<Point2f> Next;
	vector<float> MatchError;
	size_t timeNext;


	public:
	FeaturesFlow(vector<Point2f>& , vector<Point2f>& , vector<float>& , size_t);
	FeaturesFlow(vector<Point2f>& , vector<Point2f>& , vector<float>& , size_t, vector<uchar>&);
	FeaturesFlow(vector<Point2f>& , vector<Point2f>& , vector<float>& , size_t, vector<uchar>&,float);
	//virtual ~FeaturesFlow(){};

	//sets

	//gets
};

class FeaturesFlow3D : public FeaturesFlow
{
	protected:
	vector<Point3f> speed_3D;
	vector<Point3f> next_3D;
	Point3f 	speed_measure;

	public:
	FeaturesFlow3D(vector<Point3f>& speed,vector<Point3f>& next3d, vector<Point2f>& previous, vector<Point2f>& next,
						vector<float>& error, size_t time)
						:FeaturesFlow(previous , next , error , time)
	{
		speed_3D = speed;
		next_3D = next3d;
	}

	FeaturesFlow3D(vector<Point2f>& previous,vector<Point2f>& next, vector<float>& error, size_t time)
					:FeaturesFlow(previous,next,error,time){};

	FeaturesFlow3D(vector<Point2f>& previous,vector<Point2f>& next, vector<float>& error, size_t time,vector<uchar>& features_found,float max_match_error)
						:FeaturesFlow(previous,next,error,time,features_found,max_match_error){};


	//virtual ~FeaturesFlow3D(){};

	//sets
	void Setspeed_3D(vector<Point3f>);
	void Setnext_3D(vector<Point3f>);
	vector<Point3f> GetPrev3D();
	inline void SetSpeedMeasure(Point3f _measure){speed_measure=_measure;}

	//gets
		inline vector<Point3f> GetSpeedVector(void){ return speed_3D;}
		inline Point3f GetSpeed(int i){return speed_3D[i];}
		inline size_t GetNumOfFeatures(){return next_3D.size();}


};



class StreetFeaturesFlow: public FeaturesFlow3D
{

	vector<float> reliability;

	public:
		StreetFeaturesFlow( vector<Point2f>& previous, vector<Point2f>& next,vector<float>& error, size_t time,
							Vec2f left_edge_param,Vec2f right_edge_param, int rows, int cols,vector<uchar>& features_found)
							:FeaturesFlow3D(previous,next,error,time)
		{
			CalZparam(left_edge_param,right_edge_param,rows,cols);
		}

		StreetFeaturesFlow( vector<Point2f>& previous, vector<Point2f>& next,vector<float>& error, size_t time,
									Vec2f left_edge_param,Vec2f right_edge_param, int rows, int cols,vector<uchar>& features_found, float max_match_error)
				:FeaturesFlow3D(previous,next,error,time,features_found,max_match_error){
					CalZparam(left_edge_param,right_edge_param,rows,cols);
				}

		StreetFeaturesFlow( vector<Point2f>& previous, vector<Point2f>& next,vector<float>& error, size_t time,
									Vec2f left_edge_param,Vec2f right_edge_param, int rows, int cols,vector<uchar>& features_found,
									float yfloor,float fcamera)
									:FeaturesFlow3D(previous,next,error,time)
		{
			Cal3Dparam(left_edge_param,right_edge_param,rows,cols,yfloor,fcamera);
		}



		//virtual ~StreetFeaturesFlow(){};

		void Cal3Dparam(void); //function with the geometrical info of the street
		void Cal3Dparam(Vec2f,Vec2f,int,int,float,float);
		void CalZparam(Vec2f,Vec2f,int,int);
		/* data */

		//set
			void SetReliability(vector<float> v){reliability=v;}

		//gets
			inline float GetReliability(int i){return reliability[i];}

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
