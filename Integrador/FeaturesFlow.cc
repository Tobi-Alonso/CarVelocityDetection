/*
 * FeaturesFlow.cc
 *
 *  Created on: 17/06/2016
 *      Author: ezequiel & Tobi
 */


#include "FeaturesFlow.h"
//######################################################################################################
//################################---------FeaturesFlow Methods---------################################
//######################################################################################################


/*FeaturesFlow::FeaturesFlow(int MaxElements){
	Prev.reserve(MaxElements);
	Next.reserve(MaxElements);
	MatchError.reserve(MaxElements);
}*/


FeaturesFlow::FeaturesFlow(vector<ImgPoint_t >& Previous, vector<ImgPoint_t >& _next, vector<Feature2D_p>& error, size_t time)
:	timeNext(time){
	//profe conviene haber llamado al contructor vector<Feature2D_p> v(size_type n); antes, de la sig forma: por ej
	//Prev(Previous.size);		???
	Prev=Previous;
	Next=_next;
	MatchError=error;

}
 FeaturesFlow::FeaturesFlow(vector<ImgPoint_t >& Previous, vector<ImgPoint_t >& _next, vector<Feature2D_p>& error, size_t time,
															vector<uchar>& features_found):	timeNext(time)
{
	for (unsigned int i=0;i<Previous.size();i++)
	{
		if(features_found[i])
		{
			Prev.push_back(Previous[i]);
			Next.push_back(_next[i]);
			MatchError.push_back(error[i]);
		}

	}

}
FeaturesFlow::FeaturesFlow(vector<ImgPoint_t >& Previous, vector<ImgPoint_t >& _next, vector<Feature2D_p>& error, size_t time,
							vector<uchar>& features_found,Feature2D_p max_match_error):	timeNext(time)
{
	for (unsigned int i=0;i<Previous.size();i++)
	{
		if(features_found[i]&&error[i]<max_match_error)
		{
			Prev.push_back(Previous[i]);
			Next.push_back(_next[i]);
			MatchError.push_back(error[i]);
		}

	}

}




//########################################################################################################
//################################---------FeaturesFlow3D Methods---------################################
//########################################################################################################


/*FeaturesFlow3D::FeaturesFlow3D(int MaxElements):FeaturesFlow(MaxElements){
	speed_3D.reserve(MaxElements);
	next_3D.reserve(MaxElements);
}*/


void  FeaturesFlow3D::GetPrev3D(vector<SpacePoint_t >& prev_3D ){
	for(unsigned int i=0;i<speed_3D.size();i++)
		prev_3D.push_back(speed_3D[i]+next_3D[i]);
}



//############################################################################################################
//################################---------StreetFeaturesFlow Methods---------################################
//############################################################################################################

/*StreetFeaturesFlow::StreetFeaturesFlow(int MaxElements):FeaturesFlow3D(MaxElements){
	reliability.reserve(MaxElements);
}
*/
void StreetFeaturesFlow::CalZparam(edge_t  left_edge,edge_t  right_edge,int rows,int cols){//function with the geometrical info of the street

	//change of coordinates and wall adjust
	  int a0left=1/(1/left_edge[0] - x_wall/(y_floor*f_camara) ) +frame_rows/2-left_edge[1]*frame_cols/2;  
	  int a0right=1/(1/right_edge[0] + x_wall/(y_floor*f_camara) ) +frame_rows/2-right_edge[1]*frame_cols/2;

  for (unsigned int i = 0; i < Prev.size(); ++i){
		//check where is the point and calc velocity
		int x=Prev[i].x;
		if (x <cols/2) {  //left side

			if (Prev[i].y<left_edge[1]*x + a0left ){//on the wall

				reliability.push_back(1);
				next_3D.push_back(SpacePoint_t(0,0,1/(left_edge[0]+(Next[i].x-cols/2)*left_edge[1])));
				speed_3D.push_back(SpacePoint_t(0,0,1/(left_edge[0]+(x-cols/2)*left_edge[1])-next_3D.back().z ));

			}else{
				reliability.push_back(10);
				next_3D.push_back(SpacePoint_t(0,0,1/(Next[i].y-rows/2)));
				speed_3D.push_back(SpacePoint_t(0,0,1/(Prev[i].y-rows/2)-next_3D.back().z));
			}

		}else{//right side

			if (Prev[i].y<right_edge[1]*x + a0right ){//on the wall
				reliability.push_back(1);
				next_3D.push_back(SpacePoint_t(0,0,1/(right_edge[0]+(Next[i].x-cols/2)*right_edge[1])));
				speed_3D.push_back(SpacePoint_t(0,0,1/(right_edge[0]+(x-cols/2)*right_edge[1])-next_3D.back().z));

			}else{
				reliability.push_back(10);
				next_3D.push_back(SpacePoint_t(0,0,1/(Next[i].y-rows/2)));
				speed_3D.push_back(SpacePoint_t(0,0,1/(Prev[i].y-rows/2)-next_3D.back().z));
			}
		}
  }
}


void StreetFeaturesFlow::CalZparam(edge_t  left_edge,edge_t  right_edge){//function with the geometrical info of the street
	// wall adjust
	 float aux=left_edge[0];
	 left_edge[0]=1/(1/left_edge[0] - x_wall/(y_floor*f_camara) );
	 left_edge[1]=left_edge[0]*left_edge[1]/aux;
	 aux=right_edge[0];
	 right_edge[0]=1/(1/right_edge[0] + x_wall/(y_floor*f_camara) );
	 right_edge[1]=right_edge[0]*right_edge[1]/aux;
	//change of coordinates
	  int a0left=left_edge[0] +frame_rows/2-left_edge[1]*frame_cols/2;
	  int a0right=right_edge[0] +frame_rows/2-right_edge[1]*frame_cols/2;

  for (unsigned int i = 0; i < Prev.size(); ++i){
		//check where is the point and calc velocity
		int x=Prev[i].x;
		if (x <frame_cols/2) {  //left side

			if (Prev[i].y<left_edge[1]*x + a0left ){//on the wall

				reliability.push_back(1);
				next_3D.push_back(SpacePoint_t(0,0,1/(left_edge[0]+(Next[i].x-frame_cols/2)*left_edge[1])));
				speed_3D.push_back(SpacePoint_t(0,0,1/(left_edge[0]+(x-frame_cols/2)*left_edge[1])-next_3D.back().z ));

			}else{
				reliability.push_back(10);
				next_3D.push_back(SpacePoint_t(0,0,1/(Next[i].y-frame_rows/2)));
				speed_3D.push_back(SpacePoint_t(0,0,1/(Prev[i].y-frame_rows/2)-next_3D.back().z));
			}

		}else{//right side

			if (Prev[i].y<right_edge[1]*x + a0right ){//on the wall
				reliability.push_back(1);
				next_3D.push_back(SpacePoint_t(0,0,1/(right_edge[0]+(Next[i].x-frame_cols/2)*right_edge[1])));
				speed_3D.push_back(SpacePoint_t(0,0,1/(right_edge[0]+(x-frame_cols/2)*right_edge[1])-next_3D.back().z));

			}else{
				reliability.push_back(10);
				next_3D.push_back(SpacePoint_t(0,0,1/(Next[i].y-frame_rows/2)));
				speed_3D.push_back(SpacePoint_t(0,0,1/(Prev[i].y-frame_rows/2)-next_3D.back().z));
			}
		}
  }


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
	//vectorSpacePoint_t > features, speed;

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




//por si nos dice que cambiemos y no usemos pint3f sino vector de float solo para z
/*
CalZparam(edge_t  leftEdge,edge_t  rightEdge,int rows,int cols){//function with the geometrical info of the street

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
