/*
 * FeaturesFlow.cc
 *
 *  Created on: 17/06/2016
 *      Author: ezequiel
 */


#include "FeaturesFlow.h"

FeaturesFlow::FeaturesFlow(vector<Point2f>& Previous, vector<Point2f>& _next, vector<float>& error, size_t time)
{
	for (unsigned int i=0;i<Previous.size();i++)
	{
		Prev.push_back(Previous[i]);
		Next.push_back(_next[i]);
		MatchError.push_back(error[i]);
	}
	timeNext=time;

}
FeaturesFlow::FeaturesFlow(vector<Point2f>& Previous, vector<Point2f>& _next, vector<float>& error, size_t time,
															vector<uchar>& features_found)
{
	for (unsigned int i=0;i<Previous.size();i++)
	{
		if(features_found[i])
		{
			Prev.push_back(Previous[i]);
			Next.push_back(_next[i]);
			MatchError.push_back(error[i]);
		}
		timeNext=time;
	}

}
FeaturesFlow::FeaturesFlow(vector<Point2f>& Previous, vector<Point2f>& _next, vector<float>& error, size_t time,
							vector<uchar>& features_found,float max_match_error)
{
	for (unsigned int i=0;i<Previous.size();i++)
	{
		if(features_found[i]&&error[i]<max_match_error)
		{
			Prev.push_back(Previous[i]);
			Next.push_back(_next[i]);
			MatchError.push_back(error[i]);
		}
		timeNext=time;
	}

}






void FeaturesFlow3D::Setspeed_3D(vector<Point3f> speed)
{
	speed_3D = speed;
}

void FeaturesFlow3D::Setnext_3D(vector<Point3f> Next)
{
	next_3D = Next;
}

vector<Point3f> FeaturesFlow3D::GetPrev3D()
{
	Point3f Prevpoint;
	vector<Point3f> Prev;

	for(unsigned int i=0;i<speed_3D.size();i++)
	{
		Prev[i] = speed_3D[i]+next_3D[i];
	}
	return Prev;
}
bool operator== (const StreetFeaturesFlow& a,const StreetFeaturesFlow& b)
{
	if (a.next_3D==b.next_3D)
	{
		return true;
	}else{
		return false;
	}
}
bool operator!= (const StreetFeaturesFlow& a,const StreetFeaturesFlow& b)
{
	if (a.next_3D==b.next_3D)
	{
		return true;
	}else	{
		return false;
	}
}

ostream& operator<< (ostream& ost, const StreetFeaturesFlow& nieta)
{
	for (unsigned int i=0;i<nieta.next_3D.size();i++)
	{
		ost<<"Posición en el espacio: "<<nieta.next_3D<<"Velocidad: "<< nieta.speed_3D <<endl;
	}
	return ost;

}
istream& operator>> (istream& ist, StreetFeaturesFlow& nieta)
{
	unsigned int n;
	//vector<Point3f> features, speed;

	cout << "Introduzca la cantidad de Features: ";
	ist >> n;
	for(unsigned int i=0;i<n;i++)
	{
		cout << "Introduzca Posicion del Feature de la forma (x , y , z): ";
		cout << "Introduzca Posicion X: ";
		ist >> nieta.next_3D[i].x;
		cout << "Introduzca Posicion Y: ";
		ist >> nieta.next_3D[i].y;
		cout << "Introduzca Posicion Z: ";
		ist >> nieta.next_3D[i].z;
		cout << "Introduzca Velocidad del Feature de la forma (vel x ,vel y ,vel z): ";
		cout << "Introduzca Componente de la velocidad en X: ";
		ist >> nieta.speed_3D[i].x;
		cout << "Introduzca Componente de la velocidad en Y: ";
		ist >> nieta.speed_3D[i].y;
		cout << "Introduzca Componente de la velocidad en Z: ";
		ist >> nieta.speed_3D[i].z;
	}
	return ist;
}
ofstream& operator<< (ofstream& ofs, const StreetFeaturesFlow& nieta)
{

	for(unsigned int i=0;i<nieta.next_3D.size();i++)
	{
		ofs<<"\t Posición en el espacio: "<<nieta.next_3D<<"			"<<"Velocidad: "<< nieta.speed_3D <<endl;
	}
	return ofs;
}
ifstream& operator>> (ifstream& ifs, StreetFeaturesFlow& nieta)
{
	return ifs;
}




void StreetFeaturesFlow::CalZparam(Vec2f leftEdge,Vec2f rightEdge,int rows,int cols){//function with the geometrical info of the street

  int a0left=leftEdge[0] +rows/2-leftEdge[1]*cols/2;
  int a0right=rightEdge[0] +rows/2-rightEdge[1]*cols/2;

  for (unsigned int i = 0; i < Prev.size(); ++i){
		//check where is the point and calc velocity
		int x=Prev[i].x;
		if (x <cols/2) {  //left side

			if (Prev[i].y<leftEdge[1]*x + a0left ){//on the wall

				reliability.push_back(1);
				next_3D.push_back(Point3f(0,0,1/(leftEdge[0]+(Next[i].x-cols/2)*leftEdge[1])));
				speed_3D.push_back(Point3f(0,0,1/(leftEdge[0]+(x-cols/2)*leftEdge[1])-next_3D.back().z ));

			}else{
				reliability.push_back(10);
				next_3D.push_back(Point3f(0,0,1/(Next[i].y-rows/2)));
				speed_3D.push_back(Point3f(0,0,1/(Prev[i].y-rows/2)-next_3D.back().z));
			}

		}else{//right side

			if (Prev[i].y<rightEdge[1]*x + a0right ){//on the wall
				reliability.push_back(1);
				next_3D.push_back(Point3f(0,0,1/(rightEdge[0]+(Next[i].x-cols/2)*rightEdge[1])));
				speed_3D.push_back(Point3f(0,0,1/(rightEdge[0]+(x-cols/2)*rightEdge[1])-next_3D.back().z));

			}else{
				reliability.push_back(10);
				next_3D.push_back(Point3f(0,0,1/(Next[i].y-rows/2)));
				speed_3D.push_back(Point3f(0,0,1/(Prev[i].y-rows/2)-next_3D.back().z));
			}
		}
  }
}




//por si nos dice que cambiemos y no usemos pint3f sino vector de float solo para z
/*
CalZparam(Vec2f leftEdge,Vec2f rightEdge,int rows,int cols){//function with the geometrical info of the street

  int a0left=leftEdge[0] +rows/2-leftEdge[1]*cols/2;
  int a0right=rightEdge[0] +rows/2-rightEdge[1]*cols/2;

  for (unsigned int i = 0; i < Prev.size(); ++i){
		//check where is the point and calc velocity
		int x=Prev[i].x;
		if (x <cols/2) {  //left side
			
			if (Prev[i].y<leftEdge[1]*x + a0left ){//on the wall
				reliability.push_back(1);
				Next_z.push_back(1/(leftEdge[0]+(Next[i].x-cols/2)*leftEdge[1]));
				speed_z.push_back(1/(leftEdge[0]+(x-cols/2)*leftEdge[1])-Next_z.back() );

			}else{
				reliability.push_back(10);
				Next_z.push_back(1/(Next[i].y-rows/2));
				speed_z.push_back(1/(Prev[i].y-rows/2)-Next_z.back());
			}

		}else{//right side

			if (Prev[i].y<rightEdge[1]*x + a0right ){//on the wall
				reliability.push_back(1);
				Next_z.push_back(1/(rightEdge[0]+(Next[i].x-cols/2)*rightEdge[1]));
				speed_z.push_back(1/(rightEdge[0]+(x-cols/2)*rightEdge[1])-Next_z.back());

			}else{
				reliability.push_back(10);
				Next_z.push_back(1/(Next[i].y-rows/2));
				speed_z.push_back(1/(Prev[i].y-rows/2)-Next_z.back());
			}
		}
  }
}
*/
