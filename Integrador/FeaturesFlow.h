/*
 * FeaturesFlow.h
 *
 *  Created on: 16/06/2016
 *      Author: tobi
 */

#ifndef FEATURESFLOW_H_
#define FEATURESFLOW_H_

#include <iostream>
#include <fstream>


class FeaturesFlow
{
	protected:
	Point2f Prev;
	Point2f Next;
	float MatchError;
	long int timeNext;

	public:
	FeaturesFlow(arguments);
	~FeaturesFlow();

	//sets

	//gets

};



class FeaturesFlow3D: public FeaturesFlow
{
	protected:
	Point3f Speed3D;
	Point3f Next3D;

	public:
	FeaturesFlow3D(arguments);
	~FeaturesFlow3D();

	//sets
	void SetSpeed3D();
	void SetNext3D();
	Point3f GetPrev3D();

	//gets

	/* data */
};


enum place {street,LeftWall,RightWall};

class StreetFeaturesFlow: public FeaturesFlow3D
{

	place position;

	public:
		StreetFeaturesFlow(arguments); 
		~StreetFeaturesFlow();

		Cal3Dparam(); //function with the geometrical info of the street

		/* data */

		// Sobrecarga del operador de asignación
		StreetFeaturesFlow& operator= (const StreetFeaturesFlow&);

		// Sobrecarga de operadores de comparación
		bool operator== (const StreetFeaturesFlow&, const StreetFeaturesFlow&);
		bool operator!= (const StreetFeaturesFlow&, const StreetFeaturesFlow&);

		// Sobrecarga del operador menor, usado en ordenamiento
		inline bool operator< (const StreetFeaturesFlow& lhs, const StreetFeaturesFlow& rhs)
					{ bool minor= lhs.Speed3D.z<rhs.Speed3D.z:true,false;   return minor;} 

		// Sobrecarga del operador de inserción en el flujo de salida
		friend ostream& operator<< (ostream&, const StreetFeaturesFlow&);
		friend istream& operator>> (istream&, StreetFeaturesFlow&);

		// Sobrecarga del operador de inserción en el flujo de salida
		friend ofstream& operator<< (ofstream&, const StreetFeaturesFlow&);
		friend ifstream& operator>> (ifstream&, StreetFeaturesFlow&);

};





#endif /* FEATURESFLOW_H_ */
