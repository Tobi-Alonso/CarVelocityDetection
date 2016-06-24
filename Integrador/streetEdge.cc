/*
 * streetEdge.cc
 *
 *  Created on: 02/06/2016
 *      Author: tobi
 */

#include "streetEdge.h"


//default constructor
streetEdge::streetEdge(void):
thresholdH(300),
thresholdS(100),
street_side(LEFT_SIDE),
KF(4,2)
{
	kernel=(Mat_<float>(5,5)<< 	0,-1,-1,-1,-1,
								1, 0,-1,-1,-1,
								1, 1, 0,-1,-1,
								1, 1, 1, 0,-1,
								1, 1, 1, 1, 0);

    last_measure=(Mat_<float>(2,1)<< 0,CV_PI*3/4);
    kalmanConfig();

}

//full constructor
streetEdge::streetEdge(int TH, unsigned char TS, bool side):
gray_img(1,1,CV_8U),
thresholdH(TH),
thresholdS(TS),
street_side(side),
KF(4,2)
{
	kernel=(Mat_<float>(5,5)<< 	0,-1,-1,-1,-1,
								1, 0,-1,-1,-1,
								1, 1, 0,-1,-1,
								1, 1, 1, 0,-1,
								1, 1, 1, 1, 0);

    last_measure=(Mat_<float>(2,1)<< 0,CV_PI*3/4);
    kalmanConfig();
}


void streetEdge::SetThresholdHS(int TH,int TS)
{
	thresholdH = TH;
	thresholdS = TS;
}


Vec2f streetEdge::GetEdge(const Mat &frame)
{
	vector<Vec2f> lines;
	vector<int> accum;
    Vec2f param;

	if (street_side){//Right side
		gray_img=frame(Range(frame.rows/2,frame.rows),Range(frame.cols/2,frame.cols)).clone();
	}else{ //left side
        gray_img=frame(Range(frame.rows/2,frame.rows),Range(0,frame.cols/2)).clone();
		flip(gray_img,gray_img,1);
	}


    DetectEdges();

    //resize(step2,step2,Size(),reduction,reduction,INTER_LINEAR);

    myHoughLines( gray_img, lines, accum,RHO_RESULUTION, THETA_RESULUTION, thresholdH,NUM_LINES );
    criteriaFilter(lines,accum);

    KF.predict();
    Mat bestGuess=KF.correct(last_measure);
    param=coordinateConv(bestGuess);
	return param;
}

void streetEdge::DetectEdges()
{
	equalizeHist(gray_img,gray_img);
	GaussianBlur(gray_img, gray_img, Size(5,5), 3.5, 3.5);
	filter2D(gray_img,gray_img,-1,kernel,Point(-1,-1));
	threshold(gray_img, gray_img,thresholdS, 255,THRESH_BINARY);
}

void streetEdge::myHoughLines( const Mat &img,vector<Vec2f> &lines,vector<int> &weights,
    float rho, float theta, int threshold,int linesMax )
{
	cv::AutoBuffer<int> _accum, _sort_buf;
    cv::AutoBuffer<float> _tabSin, _tabCos;

    const uchar* image;
    int step, width, height;
	int numangle, numrho;
	int total = 0;
	int i, j;
	float irho = 1 / rho;
    double scale;

	//CV_Assert( CV_IS_MAT(img) && CV_MAT_TYPE(img.type) == CV_8UC1 );
    image = img.data;
    step = img.step;
    width = img.cols;
    height = img.rows;

    numangle = cvRound(CV_PI / theta);
    numrho = cvRound(((width + height) * 2 + 1) / rho);
    _accum.allocate((numangle+2) * (numrho+2));
    _sort_buf.allocate(numangle * numrho);
    _tabSin.allocate(numangle);
    _tabCos.allocate(numangle);
    int *accum = _accum, *sort_buf = _sort_buf;
    float *tabSin = _tabSin, *tabCos = _tabCos;

    memset( accum, 0, sizeof(accum[0]) * (numangle+2) * (numrho+2) );

	float ang = 0;
    for(int n = 0; n < numangle; ang += theta, n++ )
    {
        tabSin[n] = (float)(sin((double)ang) * irho);
        tabCos[n] = (float)(cos((double)ang) * irho);
    }

    // stage 1. fill accumulator	//modificar para adaptar a criterio
    for( i = 0; i < height; i++ )
        for( j = 0; j < width; j++ )
        {
            if( image[i * step + j] != 0 )
                for(int n = 0; n < numangle; n++ )
                {
                    int r = cvRound( j * tabCos[n] + i * tabSin[n] );
                    r += (numrho - 1) / 2;
                    accum[(n+1) * (numrho+2) + r+1]++;
                }
        }
    // stage 2. find local maximums
    for(int r = 0; r < numrho; r++ )
        for(int n = 0; n < numangle; n++ )
        {
            int base = (n+1) * (numrho+2) + r+1;
            if( accum[base] > threshold &&
                accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
                accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2] )
                sort_buf[total++] = base;
        }
    // stage 3. sort the detected lines by accumulator value
   // icvHoughSortDescent32s( sort_buf, total, accum );

    // stage 4. store the first min(total,linesMax) lines to the output buffer
    linesMax = MIN(linesMax, total);
    scale = 1./(numrho+2);
    for( i = 0; i < linesMax; i++ )
    {
    	Vec2f line;
        int idx = sort_buf[i];
        int n = cvFloor(idx*scale) - 1;
        int r = idx - (n+1)*(numrho+2) - 1;
        line[0] = (r - (numrho - 1)*0.5f) * rho;
        line[1]= n * theta;
        lines.push_back( line );
        weights.push_back(accum[(n+1) * (numrho+2) + r+1]);
    }
}

void streetEdge::kalmanConfig(void){
    //KF.init(4,2);
    //Vec2f state;
    // intialization of KF...
    KF.transitionMatrix = (Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,.2,0,  0,0,0,.5);
    //Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));

    KF.statePre.at<float>(0) = last_measure.at<float>(0);   //rho
    KF.statePre.at<float>(1) = last_measure.at<float>(1);    //theta

    setIdentity(KF.measurementMatrix,Scalar::all(1));
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    //setIdentity(KF.measurementNoiseCov, Scalar::all(10));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
}

void streetEdge::criteriaFilter(vector<Vec2f> &lines,vector<int> &accum){

    float theta_max_detected=0;

        vector<Vec2f>::iterator ll=lines.begin();
        vector<int>::iterator   aa=accum.begin();

        size_t num_lines=lines.size();

        for( size_t i = 0; i < num_lines && ll!=lines.end(); i++ ) {  //eliminar lineas que no cumplen con especificaciones
            float rho =(*ll)[0] , theta = (*ll)[1];

            if(theta> THETA_MIN && theta < THETA_MAX && abs(rho)< RHO_MAX){//no tiene que ser muy horizontal, ni estara en x<1(muy vertical) y tiene que tender a pasar por el centro de la imagen
                ll++;
                aa++;
                if(theta>theta_max_detected) theta_max_detected=theta;
            }else{	//bad line
            	ll=lines.erase(ll);
            	aa=accum.erase(aa);
            }
        }


    switch(lines.size()){
        case 0:
            break;
        case 1:
        	last_measure=(Mat_<float>(2,1)<<	lines[0][0],lines[0][1]);
            break;
        default:
            //average of the closest lines to the car in a range of 50cm
            float ac_rho=0,ac_theta=0;
            int acc_acc=0;
            for( size_t i = 0; i < lines.size(); i++ ) {
                if(lines[i][1]>(.80925*theta_max_detected+.32435)){  //aprox. of theta(x+.5)| x(thetamax_detected)
                        ac_rho+=lines[i][0]*accum[i];
                        ac_theta+=lines[i][1]*accum[i];
                        acc_acc+=accum[i];
                }

            }

            last_measure=(Mat_<float>(2,1)<<ac_rho/acc_acc,ac_theta/acc_acc);
    }

}


Vec2f streetEdge::coordinateConv(Mat &bestGuess){
	Vec2f param;
	int k=street_side?-1:1;
    param[1]=k*cos(bestGuess.at<float>(1))/sin(bestGuess.at<float>(1));
    param[0]= bestGuess.at<float>(0)/sin(bestGuess.at<float>(1));
    return param;
}

