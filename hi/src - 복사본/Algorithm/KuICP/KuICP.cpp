#include "stdafx.h"
#include <omp.h>
#include "KuICP.h"
KuICP::KuICP()
{
	srand((unsigned)time(NULL)); //���� ���带 �׻� �ٲ��ֱ� ���ؼ� �����ؾ��Ѵ�.
	m_dDeviationforTrans=0.2;
	m_dDeviationforRotate=0.1;
	m_dDeviationforTransRotate=0.02;

}

KuICP::~KuICP()
{

}
/**
@brief Korean: �ҿ� �ð��� �����ϱ� ���ؼ� �ʱ�ȭ�ϴ� �Լ�
@brief English: Initializes to count the duration
*/
void KuICP::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}
/**
@brief Korean: ������ �ҿ� �ð��� �޾ƿ��� �Լ�
@brief English: Gets the estimated duration
*/
float KuICP::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}
void KuICP::setDeviation(double dDeviationforTrans, double dDeviationforRotate,double dDeviationforTransRotate )
{
	m_dDeviationforTrans=dDeviationforTrans;
	m_dDeviationforRotate=dDeviationforRotate;
	m_dDeviationforTransRotate=dDeviationforTransRotate;
}
inline double KuICP::calRMSError(int* nMatchingPointIndex,double* PreData,int NoPreData, double*TransformedData, int NoNewData,int cnt)
{
	int i=0;
	double dRMSError = 0;

	// --------------------------- compute RMS error ----------------------------------- //
	for (i=0; i<NoNewData; i++) {
		if (nMatchingPointIndex[i] != -1) {
			dRMSError += sqrt((TransformedData[i*2+0]-PreData[nMatchingPointIndex[i]*2+0])*(TransformedData[i*2+0]-PreData[nMatchingPointIndex[i]*2+0])
				+ (TransformedData[i*2+1]-PreData[nMatchingPointIndex[i]*2+1])*(TransformedData[i*2+1]-PreData[nMatchingPointIndex[i]*2+1]));
		}
	}
	dRMSError = (dRMSError)/(double)cnt;

	return dRMSError;
}

inline void KuICP::calTransformedData(double* PreData,int NoPreData, double*TransformedData, int NoNewData,double DeltaX,double DeltaY,double DeltaT,double *accDeltaX,double *accDeltaY,double *accDeltaT) 
{
	int i=0;
	double  dCos=cos(DeltaT);
	double  dSin=sin(DeltaT);
	double dTempX=(*accDeltaX);
	double dTempY=(*accDeltaY);

	// --------------------- accumulate translation and rotation ---------------------- //

	(*accDeltaX) = DeltaX + dTempX*dCos - dTempY*dSin;
	(*accDeltaY) = DeltaY + dTempX*dSin + dTempY*dCos;
	(*accDeltaT) += DeltaT;

	// ------------------ transform new data using R & T matrix ------------------------- //
	double dPreDataX=0.0;
	double dPreDataY=0.0;

	for (i=0; i<NoNewData; i++) {

		dPreDataX=TransformedData[i*2+0];
		dPreDataY=TransformedData[i*2+1];
		TransformedData[i*2+0] = DeltaX + dPreDataX*dCos - dPreDataY*dSin;
		TransformedData[i*2+1] = DeltaY + dPreDataX*dSin + dPreDataY*dCos;
	}

}


inline void KuICP::calDeltaPos(int*nMatchingPointIndex,double* PreData,int NoPreData, double* NewData, int NoNewData,int cnt, double *DeltaX,double *DeltaY,double *DeltaT)
{
	// ------------------------------------ Singular value decomposition ----------------------- //

	double meanPreX, meanPreY, meanNewX, meanNewY;
	int i=0;
	double w1, w2, w3, w4;
	double dCos=0.0,dSin=0.0;

	CvMat* W = cvCreateMat(3,3,CV_64FC1);
	CvMat* S = cvCreateMat(3,3,CV_64FC1);
	CvMat* U = cvCreateMat(3,3,CV_64FC1);
	CvMat* V = cvCreateMat(3,3,CV_64FC1);
	CvMat* Rotation = cvCreateMat(3,3,CV_64FC1);


	meanPreX=0;meanPreY=0;meanNewX=0;meanNewY=0;
	for (i=0; i<NoNewData; i++) {
		if (nMatchingPointIndex[i] != -1) {
			meanPreX += PreData[nMatchingPointIndex[i]*2];
			meanPreY += PreData[nMatchingPointIndex[i]*2+1];
			meanNewX += NewData[i*2];
			meanNewY += NewData[i*2+1];
		}
	}
	meanPreX /= (double)cnt; meanPreY /= (double)cnt; meanNewX /= (double)cnt; meanNewY /= (double)cnt;
	w1 = 0; w2 = 0; w3 = 0; w4 = 0;

	for (i=0; i<NoNewData; i++) {
		if (nMatchingPointIndex[i] != -1) {
			w1 += (PreData[nMatchingPointIndex[i]*2] - meanPreX)*(NewData[i*2] - meanNewX);
			w2 += (PreData[nMatchingPointIndex[i]*2] - meanPreX)*(NewData[i*2+1] - meanNewY);
			w3 += (PreData[nMatchingPointIndex[i]*2+1] - meanPreY)*(NewData[i*2] - meanNewX);
			w4 += (PreData[nMatchingPointIndex[i]*2+1] - meanPreY)*(NewData[i*2+1] - meanNewY);
		}
	}		


	cvmSet(W, 0, 0, w1);cvmSet(W, 0, 1, w2);cvmSet(W, 0, 2, 0.0);
	cvmSet(W, 1, 0, w3);cvmSet(W, 1, 1, w4);cvmSet(W, 1, 2, 0.0);
	cvmSet(W, 2, 0, 0.0);cvmSet(W, 2, 1, 0.0);cvmSet(W, 2, 2, 0.0);

	cvSVD(W, S, U, V, CV_SVD_V_T);

	cvMatMul(U, V, Rotation);	

	// ----------------------- compute R & T matrix ----------------------------------- //
	*DeltaT = atan2(cvmGet(Rotation,1,0), cvmGet(Rotation,0,0));
	dCos=cos(*DeltaT);	dSin=sin(*DeltaT);
	*DeltaX = meanPreX - (meanNewX*dCos - meanNewY*dSin);
	*DeltaY = meanPreY - (meanNewX*dSin + meanNewY*dCos);

	cvReleaseMat(&W);
	cvReleaseMat(&S);
	cvReleaseMat(&U);
	cvReleaseMat(&V);
	cvReleaseMat(&Rotation);
}

inline void KuICP::findClosestPoint(int* nMatchingPointIndex, double* PreData,int NoPreData, double* NewData, int NoNewData,double dDIstTh ,int *nCnt)
{
	*nCnt = 0;
	int i,j;
	double MinDist, Dist;
	bool* bAlreadyMatched;
	double* dDistMatched;
	int MinIndex;

	dDistMatched = new double[NoNewData];
	bAlreadyMatched = new bool[NoPreData];

	for (i=0; i<NoNewData; i++){
		nMatchingPointIndex[i] = -1;
		dDistMatched[i]=dDIstTh;
	}
	for (i=0; i<NoPreData; i++)
		bAlreadyMatched[i] = false;

	for (i=0; i<NoNewData; i++) 
	{
		MinDist = dDIstTh;

		MinIndex = -1;

		for (j=0; j<NoPreData; j++) 
		{
			Dist = sqrt( (NewData[i*2]-PreData[j*2])*(NewData[i*2]-PreData[j*2]) 
				+ (NewData[i*2+1]-PreData[j*2+1])*(NewData[i*2+1]-PreData[j*2+1]) );

			if (Dist < MinDist) {
				MinDist = Dist;
				MinIndex = j;
				dDistMatched[i] = Dist;
			}
		}

		if (MinIndex == -1)	continue;

		if (bAlreadyMatched[MinIndex]== false) {
			bAlreadyMatched[MinIndex] = true;
			nMatchingPointIndex[i] = MinIndex;
			(*nCnt)++;
		}
		else if(i>0 ) {
			for(int n= 0; n<i;n++)
			{
				if(nMatchingPointIndex[n] == MinIndex)
				{
					if(dDistMatched[n]>dDistMatched[i] )
					{
						nMatchingPointIndex[n] = -1;
						nMatchingPointIndex[i] = MinIndex;
					}
					else
					{
						nMatchingPointIndex[i]=-1;
					}
					break;
				}
			}				
		}
	}

	delete dDistMatched;
	delete bAlreadyMatched;

}
inline double KuICP::GaussRand()
{
	static double u, v;
	static int phase = 0;
	double z;

	if (phase == 0) {
		u = (rand() + 1.) / (RAND_MAX + 2.);
		v = rand() / (RAND_MAX + 1.);
		z = sqrt(-2 * log(u)) * sin(2 * 3.14159265358979323846 * v);
	}
	else {
		z = sqrt(-2 * log(u)) * cos(2 * 3.14159265358979323846 * v);
	}
	phase = 1 - phase;
	return z;
}

/**
@brief Korean: 2*sigma ~ 2*sigma ���� ������ ������ ���� �����ϴ� �Լ�. (�ŷڵ� 95% �̳�)
@brief English: return random value from -2*sigma to 2*sigma
*/
inline double KuICP::GetRandValue(double sigma)
{
	return sigma*2.0*( (double)rand()/(double)RAND_MAX - 0.5);
}
/**
@brief Korean: ���� �Ÿ�����(t-1)�� ���� �Ÿ�����(t)�� ���Ͽ� ��ġ�� ��ȭ���� ���ϴ� �Լ�
@brief English: 
*/
double KuICP::icp(double R[3][3], double T[3], double* PreData, 
	int NoPreData, double* NewData, int NoNewData, 
	int MinIter, int MaxIter, double MinOverrap, double threshold, double margin,
	double dEXm,double dEYm,double dETm)
{  	
	// PreData: previous data or model. Type: x, y, z, x, y, z, x, y, z, .....
	// NewData: new data or sensor data. Type: x, y, z, x, y, z, x, y, z, .....
	// NoPreData: number of PreData
	// MinIter: number of minimum iteration
	// MaxIter: number of maximum iteration
	// MinOverrap: minimun number of matched points should be over MinOverrap*NoNewData
	// threshold: if RMS error is lower than threshold, iteration stops
	// margin: distance margin for matching decision.
	// margin = 2.0: fast rotation or small environment
	// margin = 3.0: normal, complex and large environment

	int i(0),j(0);
	double RMSError = 1.0;
	int nCnt(0);
	double selaccDeltaX(0.0), selaccDeltaY(0.0), selaccDeltaT(0.0), dselError(RMSError * margin);
	double** TransformedData;
	int** nMatchingPointIndex;
	double dPreRMSError(0.0);
	int nselInDX=0;
	double dDist=0;
	double dXm_t=0.0;
	double dYm_t=0.0;
	double dTm_t=0.0;
	int NoIteration=0;

	double* DeltaX; double*  DeltaY; double* DeltaT;
	double* accDeltaX ; double* accDeltaY; double* accDeltaT;

	const int max_iteration = 1; 

	double d1 = (double)0.1;				// �����̵��� ���� ��ġ������ ǥ������
	double d3 = (double)0.3;				// ȸ���� ���� ���������� ǥ������
	double d2 = (double)d1*d3;				// �����̵��� ���� ���������� ǥ������

	// ���հ������� ���Ǵ� ��ġ�� ���� ������ ����
	IcpPos *fi = new IcpPos [max_iteration];

	DeltaX = new double [max_iteration];
	DeltaY = new double [max_iteration];
	DeltaT = new double [max_iteration];

	accDeltaX = new double [max_iteration];
	accDeltaY = new double [max_iteration];
	accDeltaT = new double [max_iteration];

	TransformedData = new double*[max_iteration];
	nMatchingPointIndex = new int*[max_iteration];
	if(TransformedData){
		for(int i=0; i <max_iteration; i++){
			TransformedData[i] = new double[NoNewData*2];
			nMatchingPointIndex[i] = new int[NoNewData*2];
		}
	}	

	for (int i=0; i<max_iteration; ++i) {
		int m = i ? 1 : 0;
		double dTransdistance = sqrt(pow(dEXm,2)+pow(dEYm,2));			// �����̵��Ÿ�
		double dNoiseTrans = GetRandValue(dTransdistance*d1);			// ������� ���� ����. �̵��Ÿ��� ����Ͽ� �����Ѵ�.
		double dNoiseTransRotate = GetRandValue(dTransdistance*d2);	// ������� ���� ��������. �̵��Ÿ��� ����Ͽ� �����Ѵ�.
		double dNoiseRotate = GetRandValue(dETm*d3);

		dXm_t=m*dNoiseTrans;
		dYm_t=m*GetRandValue(dEYm*d2);
		dTm_t=m*(dNoiseTransRotate+dNoiseRotate);
		fi[i].th = dTm_t;
		fi[i].x  = dXm_t*cos(fi[i].th)-dYm_t*sin(fi[i].th);
		fi[i].y  = dXm_t*sin(fi[i].th)+dYm_t*cos(fi[i].th);
		fi[i].error = 1000000000;
	}


	for(int j=0; j<max_iteration;j++)
	{
		for (int i=0; i<NoNewData; i++) {
			TransformedData[j][i*2+0] =fi[j].x+ NewData[i*2+0]*cos(fi[j].th)-NewData[i*2+1]*sin(fi[j].th);
			TransformedData[j][i*2+1] =fi[j].y+ NewData[i*2+0]*sin(fi[j].th)+NewData[i*2+1]*cos(fi[j].th);
		}

		accDeltaX[j] = fi[j].x , accDeltaY[j] = fi[j].y , accDeltaT[j] =fi[j].th ;		
		DeltaX[j] = 0.0, DeltaY[j] = 0.0, DeltaT[j] =0.0;
		RMSError = 0.1;
		dPreRMSError=0.0;				
		NoIteration=0;

		while ( NoIteration < MaxIter)
		{
			NoIteration++;
			// ----------------------- find closest point -------------------------------------- //
			findClosestPoint(nMatchingPointIndex[j], PreData, NoPreData, TransformedData[j],  NoNewData, RMSError*margin ,&nCnt);

			// exception
			if (NoIteration > MinIter && nCnt<(int)(MinOverrap*(double)NoNewData))  {
				RMSError = -1;
				break;
			}

			calDeltaPos(nMatchingPointIndex[j],PreData, NoPreData, TransformedData[j],  NoNewData,nCnt, &DeltaX[j],&DeltaY[j],&DeltaT[j]); 

			calTransformedData(PreData, NoPreData, TransformedData[j],  NoNewData,DeltaX[j],DeltaY[j],DeltaT[j],&accDeltaX[j],&accDeltaY[j],&accDeltaT[j]); 

			RMSError =  calRMSError(nMatchingPointIndex[j], PreData, NoPreData, TransformedData[j],NoNewData,nCnt); 


			if(fi[j].error>RMSError)
			{
				fi[j].error=RMSError;
				fi[j].x=accDeltaX[j];
				fi[j].y=accDeltaY[j];
				fi[j].th=accDeltaT[j];
			}
			if ((RMSError < threshold && NoIteration > MinIter)||dPreRMSError==RMSError)
			{
				break;
			}
			dPreRMSError=RMSError;

		}
	}


	// ----------------------- delete variables ---------------------------------------------- //

	double derror=100000;
	for (int i=0; i<max_iteration; ++i) {
		if(derror>fi[i].error&&fi[i].error!=-1)
		{
			derror=fi[i].error;
			selaccDeltaX=fi[i].x ;
			selaccDeltaY=fi[i].y ;
			selaccDeltaT=fi[i].th;
			dselError=derror;
		}
	}	

	R[0][0] = selaccDeltaT;
	T[0] = selaccDeltaX;
	T[1] = selaccDeltaY;

	delete DeltaX ;
	delete DeltaY ;
	delete DeltaT ;

	delete accDeltaX ;
	delete accDeltaY ;
	delete accDeltaT ;

	if(TransformedData){
		for(int i = 0 ; i < max_iteration ; i++){
			delete[] TransformedData[i];
			delete[] nMatchingPointIndex[i];			
		}
		delete[] TransformedData;
		delete[] nMatchingPointIndex;

	}
	delete fi;

	return dselError;
}

double KuICP::icp(double R[3][3], double T[3], double* PreData, 
	int NoPreData, double* NewData, int NoNewData, 
	int MinIter, int MaxIter, double MinOverrap, double threshold, double margin)
{  	
	// PreData: previous data or model. Type: x, y, z, x, y, z, x, y, z, .....
	// NewData: new data or sensor data. Type: x, y, z, x, y, z, x, y, z, .....
	// NoPreData: number of PreData
	// MinIter: number of minimum iteration
	// MaxIter: number of maximum iteration
	// MinOverrap: minimun number of matched points should be over MinOverrap*NoNewData
	// threshold: if RMS error is lower than threshold, iteration stops
	// margin: distance margin for matching decision.
	// margin = 2.0: fast rotation or small environment
	// margin = 3.0: normal, complex and large environment


	int i(0),j(0);
	double RMSError = 10.0;
	int nCnt(0);
	double selaccDeltaX(0.0), selaccDeltaY(0.0), selaccDeltaT(0.0), dselError(RMSError * margin);
	double** TransformedData;
	int** nMatchingPointIndex;
	double dPreRMSError(0.0);
	int nselInDX=0;
	double dDist=0;
	double dXm_t=0.0;
	double dYm_t=0.0;
	double dTm_t=0.0;
	int NoIteration=0;

	double* DeltaX; double*  DeltaY; double* DeltaT;
	double* accDeltaX ; double* accDeltaY; double* accDeltaT;

	const int max_iteration = 16; 

	// ���հ������� ���Ǵ� ��ġ�� ���� ������ ����
	IcpPos *fi = new IcpPos [max_iteration];

	DeltaX = new double [max_iteration];
	DeltaY = new double [max_iteration];
	DeltaT = new double [max_iteration];

	accDeltaX = new double [max_iteration];
	accDeltaY = new double [max_iteration];
	accDeltaT = new double [max_iteration];

	TransformedData = new double*[max_iteration];
	nMatchingPointIndex = new int*[max_iteration];
	if(TransformedData){
		for(int i=0; i <max_iteration; i++){
			TransformedData[i] = new double[NoNewData*2];
			nMatchingPointIndex[i] = new int[NoNewData*2];
		}
	}

	for (int i=0; i<max_iteration; ++i) {
		int m = i ? 1 : 0;	
		dXm_t=0.5*m*GaussRand();
		dYm_t=0.1*m*GaussRand();
		dTm_t=0.628*m*GaussRand();
		fi[i].th = dTm_t;
		fi[i].x  = dXm_t*cos(fi[i].th)-dYm_t*sin(fi[i].th);
		fi[i].y  = dXm_t*sin(fi[i].th)+dYm_t*cos(fi[i].th);
		fi[i].error = 1000000000;
	}


	LARGE_INTEGER t;
	startTimeCheck(t);


	for(int j=0; j<max_iteration;j++)
	{
		for (int i=0; i<NoNewData; i++) {
			TransformedData[j][i*2+0] =fi[j].x+ NewData[i*2+0]*cos(fi[j].th)-NewData[i*2+1]*sin(fi[j].th);
			TransformedData[j][i*2+1] =fi[j].y+ NewData[i*2+0]*sin(fi[j].th)+NewData[i*2+1]*cos(fi[j].th);
		}

		accDeltaX[j] = fi[j].x , accDeltaY[j] = fi[j].y , accDeltaT[j] =fi[j].th ;		
		DeltaX[j] = 0.0, DeltaY[j] = 0.0, DeltaT[j] =0.0;
		RMSError = 10.0*margin;
		dPreRMSError=0.0;				
		NoIteration=0;

		while ( NoIteration < MaxIter)
		{
			NoIteration++;
			// ----------------------- find closest point -------------------------------------- //
			findClosestPoint(nMatchingPointIndex[j], PreData, NoPreData, TransformedData[j],  NoNewData, RMSError*margin ,&nCnt);

			// exception
			if (NoIteration > MinIter && nCnt<(int)(MinOverrap*(double)NoNewData))  {
				RMSError = -1;
				break;
			}

			calDeltaPos(nMatchingPointIndex[j],PreData, NoPreData, TransformedData[j],  NoNewData,nCnt, &DeltaX[j],&DeltaY[j],&DeltaT[j]); 

			calTransformedData(PreData, NoPreData, TransformedData[j],  NoNewData,DeltaX[j],DeltaY[j],DeltaT[j],&accDeltaX[j],&accDeltaY[j],&accDeltaT[j]); 

			RMSError =  calRMSError(nMatchingPointIndex[j], PreData, NoPreData, TransformedData[j],NoNewData,nCnt); 


			if(fi[j].error>RMSError/(double)nCnt)
			{
				fi[j].error=RMSError/(double)nCnt;
				fi[j].x=accDeltaX[j];
				fi[j].y=accDeltaY[j];
				fi[j].th=accDeltaT[j];
			}
			if ((RMSError < threshold && NoIteration > MinIter)||dPreRMSError==RMSError)
			{
				break;
			}
			dPreRMSError=RMSError;

		}
	}

	double dtime = finishTimeCheck(t);

	// ----------------------- delete variables ---------------------------------------------- //


	double derror=100000;
	for (int i=0; i<max_iteration; ++i) {
		if(derror>fi[i].error)
		{
			derror=fi[i].error;
			selaccDeltaX=fi[i].x ;
			selaccDeltaY=fi[i].y ;
			selaccDeltaT=fi[i].th;
			dselError=derror;
		}
	}

	R[0][0] = selaccDeltaT;
	T[0] = selaccDeltaX;
	T[1] = selaccDeltaY;

	delete DeltaX ;
	delete DeltaY ;
	delete DeltaT ;

	delete accDeltaX ;
	delete accDeltaY ;
	delete accDeltaT ;

	if(TransformedData){
		for(int i = 0 ; i < max_iteration ; i++){
			delete[] TransformedData[i];
			delete[] nMatchingPointIndex[i];			
		}
		delete[] TransformedData;
		delete[] nMatchingPointIndex;

	}	
	delete fi;

	return dselError;
}
