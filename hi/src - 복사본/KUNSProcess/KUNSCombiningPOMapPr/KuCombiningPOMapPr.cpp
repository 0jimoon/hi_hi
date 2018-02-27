#include "stdafx.h"
#include "KuCombiningPOMapPr.h"

KuCombiningPOMapPr::KuCombiningPOMapPr()
{
	m_bThreadFlag = false;

	//m_nCellSize = 1000;				// 1cell 1000mm;

	m_dRadiusofRobot = 400; //로봇반지름 400(mm)

	m_nMapSizeX=0, m_nMapSizeY=0;
	m_nMapTimeNum=0;
	m_nMapNum=0;
	m_dOutMap=NULL;
	cout<<"[KuCombiningProMapPr]: Instance is created!!!"<<endl;
}

KuCombiningPOMapPr::~KuCombiningPOMapPr()
{

	if(m_dOutMap){
		for(int i = 0 ; i < m_nMapSizeX ; i++){
			delete[] m_dOutMap[i];
			m_dOutMap[i] = NULL;
		}
		delete[] m_dOutMap;
	}
	m_dOutMap=NULL;

	cout<<"[KuCombiningProMapPr]: Instance is destroyed!!!"<<endl;
}

/**
@brief Korean: 초기화 작업을 수행하는 함수.
@brief English: 
*/
bool KuCombiningPOMapPr::initialize( int nMapSizeX,int nMapSizeY)
{
	m_bThreadFlag = false;

	m_nMapSizeX=nMapSizeX;
	m_nMapSizeY=nMapSizeY;

	if(m_dOutMap==NULL)
	{
		m_dOutMap = new double*[m_nMapSizeX];

		for(int i=0; i <m_nMapSizeX; i++){
			m_dOutMap[i] = new double[m_nMapSizeY];
		}

	}

	for(int i=0;i<m_nMapSizeX; i++){
		for(int j=0; j<m_nMapSizeY; j++){
			m_dOutMap[i][j] = 0;
		}
	}
	for(int i=0; i<25;i++)//시간 25로 0시간을 제외시킨다
	{
		m_nHour[i]=0;
	}
	m_nLastNum=-1;

	return true;	
}

bool KuCombiningPOMapPr::loadProMap(string strMapFilePath)
{
	char cMapPathName[200];

	if(m_vecProbabilityMap.size()>0)
	{
		for (int i=0;i<m_vecProbabilityMap.size();i++)
		{
			double ** dProMap = m_vecProbabilityMap[i];
			if(dProMap){
				for(int i = 0 ; i < m_nMapSizeX ; i++){
					delete[] dProMap[i];
					dProMap[i] = NULL;
				}
				delete[] dProMap;
			}
			dProMap=NULL;
		}

		m_vecProbabilityMap.clear();
	}

	string strMapFile;

	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"%s/ProbablityMapData.txt", strMapFilePath.c_str());


	KuINIReadWriter* pINIReaderWriter;
	pINIReaderWriter = new KuINIReadWriter(cMapPathName); //설정파일 읽기


	if (pINIReaderWriter->ParseError() < 0) { //파일을 읽지 못한 경우 프로그램을 종료시킨다.
		cout << "Can't load "<<strMapFilePath<<endl;;
		//_getch();
		return false;
	}
	int nMapSizeX=m_nMapSizeX;
	int nMapSizeY=m_nMapSizeY;
	
	m_nMapSizeX = pINIReaderWriter->getIntValue("MAP","MAP_SIZE_X",nMapSizeX);
	m_nMapSizeY = pINIReaderWriter->getIntValue("MAP","MAP_SIZE_Y",nMapSizeY);
	//m_nCellSize = pINIReaderWriter->getIntValue("MAP","CELL_SIZE",100);
	//m_math.setCellSizeMM(m_nCellSize);	

	delete pINIReaderWriter;
///---------------------------------------------------------------------------------------------------------///

	SYSTEMTIME stTime;
	GetLocalTime(&stTime);
	int nDay = stTime.wDay;
	int nHour =stTime.wHour;
	
	int nDayIDX = 0;
	int nHourIDX =0;


	int nMinDay=nDay-1;
	int nMaxDay=nDay+1;


	int nMinHour=nHour-1;
	int nMaxHour=nHour+1;

	for(int j=nMinDay;j<=nMaxDay;j++ )// 시간에 따라 따로 저장해야한다
	{
		for(int i=nMinHour; i<=nMaxHour; i++)
		{
			if(j<0){nDayIDX=j+31;} 
			else if(j>31){nDayIDX=j-31;}
			else{nDayIDX=j;}
			if(i<0){nHourIDX=i+24;} 
			else if(i>24){nHourIDX=i-24;}
			else{nHourIDX=i;}
					

			memset(cMapPathName,0,sizeof(cMapPathName));
			sprintf_s(cMapPathName,"%s/Day%d/Hour%d/ProbablityMapData.txt", strMapFilePath.c_str(),nDayIDX,nHourIDX);

			///---------------------------------------------------------------------------------------------------------///
			KuINIReadWriter* pINIReaderWriter;
			pINIReaderWriter = new KuINIReadWriter(cMapPathName); //설정파일 읽기

			if (pINIReaderWriter->ParseError() < 0) { //파일을 읽지 못한 경우 프로그램을 종료시킨다.
				cout << "Can't load "<<strMapFilePath<<endl;;
				//_getch();
				delete pINIReaderWriter;
				continue;
			}

			int nDataNum = pINIReaderWriter->getIntValue("PATH","DATANUMBER",0);

			if(nDay==nDayIDX){m_nHour[nHourIDX]=nDataNum;}
			int nMinNum=1;
			if(nDataNum>10) nMinNum=nDataNum-10;

			for(int nNum=nMinNum;nNum<nDataNum+1;nNum++)
			{
				memset(cMapPathName,0,sizeof(cMapPathName));
				sprintf_s(cMapPathName,"PATH&NAME%d",nNum);

				strMapFile= pINIReaderWriter->getStringValue("PATH", cMapPathName, "no");

				double ** dProbabilityMap = new double*[m_nMapSizeX];

				if(dProbabilityMap){
					for(int ni=0; ni <m_nMapSizeX; ni++){
						dProbabilityMap[ni] = new double[m_nMapSizeY];
					}
				}

				loadProMap(strMapFile,m_nMapSizeX,m_nMapSizeY,dProbabilityMap);
				m_vecProbabilityMap.push_back(dProbabilityMap);	

				if(nDay==nDayIDX&&nHour==nHourIDX&&nDataNum==nNum)
				{
					m_nLastNum=m_vecProbabilityMap.size()-1;
				}

			}

			delete pINIReaderWriter;
			///---------------------------------------------------------------------------------------------------------///

		}
	}

	return true;
}

void KuCombiningPOMapPr::saveProMap(string strMapFilePath,int nMapSizeX,int nMapSizeY, double** dMap)
{
	int i = 0;
	int j = 0;

	ofstream data(strMapFilePath);

	for(i=0; i<nMapSizeX; i++){

		for(j=0; j<nMapSizeY; j++){

			data<<dMap[i][j]<<" ";
		}
		data<<endl;
	}

	data.close();

}

bool KuCombiningPOMapPr::loadProMap(string strMapFilePath,int nMapSizeX,int nMapSizeY, double** dMap)
{	
	char cData;

	double t;

	ifstream file_in;
	file_in.open(strMapFilePath);

	if(!file_in.is_open())
	{
		return false;
	}

	for(int i = 0 ; i< nMapSizeX; i++)
	{
		for(int j = 0; j< nMapSizeY ; j++) 
		{
			file_in >> t;
			dMap[i][j] = t;
		}
	}

	file_in.close();
}

bool KuCombiningPOMapPr::saveProMap(string strMapFilePath,double** dProMap)
{
	SYSTEMTIME stTime;
	GetLocalTime(&stTime);
	int nDay = stTime.wDay;
	int nHour =stTime.wHour;

	CString strTemp;
	CString strPath;
	char cMapPathName[200];
	CString strMapName;

	strPath=strMapFilePath.c_str();

	m_nHour[nHour]=m_nHour[nHour]+1;

	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"%s/Day%d/Hour%d/ProbablityMapData.txt", strMapFilePath.c_str(),nDay,nHour);
	strPath=cMapPathName;

	strTemp.Format(L"%d",m_nHour[nHour]);
	WritePrivateProfileString(_T("PATH"), _T("DATANUMBER"), strTemp,strPath);

	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"PATH&NAME%d",m_nHour[nHour]);

	strMapName=cMapPathName;

	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"%s/Day%d/Hour%d/Map%d.txt",strMapFilePath.c_str(),nDay,nHour, m_nHour[nHour]);
	strTemp=cMapPathName;

	WritePrivateProfileString(_T("PATH"), strMapName, strTemp,strPath);

	saveProMap(cMapPathName,m_nMapSizeX,m_nMapSizeY, dProMap);

	return true;
}


void KuCombiningPOMapPr::calGradientMask(double** dProMap,int nX,int nY,double** dGradientMask,int nMapSizeX,int nMapSizeY)
{
	for(int i=-1; i<2;i++)
	{
		for(int j=-1; j<2;j++)
		{
			dGradientMask[i+1][j+1]=fabs(dProMap[nX][nY]-dProMap[nX+i][nY+j]);
		}
	}
}

void KuCombiningPOMapPr::calNormalization(double** dMultiplicationPro,double** dOutNormalPro)
{
	int nMaskSize=3;
	double dSum=0.0 ;

	for(int i=0;i<nMaskSize; i++){
		for(int j=0; j<nMaskSize; j++){
			dSum+=dMultiplicationPro[i][j] ;
		}
	}

	if(dSum==0.0)dSum=1.0;

	for(int i=0;i<nMaskSize; i++){
		for(int j=0; j<nMaskSize; j++){
			dOutNormalPro[i][j]=dMultiplicationPro[i][j]/dSum;
		}
	}
}


double KuCombiningPOMapPr::calConditionalPro(double**dCurTimeMap,double** dOutNormalPro,double dXPro,int nX,int nY )
{
	double dPro=0;
	double dDenominator=1.0;//분모
	double dNumerator=1.0; //분자
	int nMaskSize=3;
	double dSum=0;

	dNumerator=dXPro*dXPro;
	vector<double> vecCurMapPro;

	for(int i=0;i<nMaskSize; i++){
		for(int j=0; j<nMaskSize; j++){		
			if(dOutNormalPro[i][j]!=0.0)
			{
				dNumerator*=dOutNormalPro[i][j];
				double dThData;
				dThData=dCurTimeMap[nX+i-1][nY+j-1];
				if(dThData>0.9)dThData=0.9;
				else if(dThData<0.1)dThData=0.1;
				vecCurMapPro.push_back(dThData);
				dSum+=dThData;
			}
		}
	}

	for(int i=0; i<vecCurMapPro.size();i++)
	{
		if(dXPro<0.5)
		{
			printf("%d, %d\n",nX,nY);
		}
		double dTemp=vecCurMapPro[i]/dSum;
		dDenominator*=dTemp;
	}

	dPro=dNumerator/dDenominator;

	return dPro;
}

void KuCombiningPOMapPr::countPro(double**dGradPro,double**dNextMap, int nX, int nY,double dTotalNum )
{
	int nMaskSize=3;	

	for(int i=0;i<nMaskSize; i++){
		for(int j=0; j<nMaskSize; j++){
			if(dNextMap[nX][nY]>0.5)
			{
				if(dNextMap[nX+i-1][nY+j-1]>0.5)
				{
					dGradPro[i][j]+=1/dTotalNum;
				}
			}

		}
	}
}


void KuCombiningPOMapPr::calPostProNum(double**dPosterioriNum,double**dDatabaseMap, int nX, int nY )
{
	int nMaskSize=3;	

	for(int i=0;i<nMaskSize; i++){
		for(int j=0; j<nMaskSize; j++){

			if(dDatabaseMap[nX+i-1][nY+j-1]>0.5)
			{
				if(dDatabaseMap[nX][nY]>0.5)
				{
					dPosterioriNum[i][j]+=1.0;
				}
			}

			if(dDatabaseMap[nX+i-1][nY+j-1]<0.5)
			{
				if(dDatabaseMap[nX][nY]>0.5)
				{
					dPosterioriNum[i][j]+=1.0;
				}
			}



		}
	}
}
void KuCombiningPOMapPr::calOccLikelihoodNum(double**dLikelihoodNum,double**dDatabasetMap, int nX, int nY,double**dCurStateMap)
{
	int nMaskSize=3;	

	for(int i=0;i<nMaskSize; i++){
		for(int j=0; j<nMaskSize; j++){
			if(dDatabasetMap[nX][nY]>0.5)
			{
				if(dCurStateMap[nX+i-1][nY+j-1]==1)
				{
					if(dDatabasetMap[nX+i-1][nY+j-1]>0.5)
					{
						dLikelihoodNum[i][j]+=1.0;
					}
				}
				else if(dCurStateMap[nX+i-1][nY+j-1]==-1)
				{
					if(dDatabasetMap[nX+i-1][nY+j-1]<0.5)
					{
						dLikelihoodNum[i][j]+=1.0;
					}
				}
			}

		}
	}
}


void KuCombiningPOMapPr::calEmptyLikelihoodNum(double**dLikelihoodNum,double**dDatabasetMap, int nX, int nY,double**dCurStateMap )
{
	int nMaskSize=3;	

	for(int i=0;i<nMaskSize; i++){
		for(int j=0; j<nMaskSize; j++){
			if(dDatabasetMap[nX][nY]<0.5)
			{
				if(dCurStateMap[nX+i-1][nY+j-1]==1)
				{
					if(dDatabasetMap[nX+i-1][nY+j-1]>0.5)
					{
						dLikelihoodNum[i][j]+=1.0;
					}
				}
				else if(dCurStateMap[nX+i-1][nY+j-1]==-1)
				{
					if(dDatabasetMap[nX+i-1][nY+j-1]<0.5)
					{
						dLikelihoodNum[i][j]+=1.0;
					}
				}
			}

		}
	}
}

//P(X|C); 사후확률 C가 점유되었을때 X가 점유되었을 확률
void KuCombiningPOMapPr::calPostPro(double**dPostPro,double**dDatabaseMap, int nX, int nY,double** dNum )
{
	int nMaskSize=3;	

	for(int i=0;i<nMaskSize; i++){
		for(int j=0; j<nMaskSize; j++){

			if(dDatabaseMap[nX+i-1][nY+j-1]>0.5)
			{
				if(dDatabaseMap[nX][nY]>0.5)
				{
					dPostPro[i][j]+=1/dNum[i][j];
				}
			}
		}
	}
}

void KuCombiningPOMapPr::calOccLikelihoodPro(double**dLikelihood,double**dDatabasetMap, int nX, int nY,double** dNum,double**dCurStateMap)
{
	int nMaskSize=3;	

	for(int i=0;i<nMaskSize; i++){
		for(int j=0; j<nMaskSize; j++){
			if(dDatabasetMap[nX][nY]>0.5)
			{	
				if(dCurStateMap[nX+i-1][nY+j-1]==1)
				{
					if(dDatabasetMap[nX+i-1][nY+j-1]>0.5)
					{
						dLikelihood[i][j]+=1/dNum[i][j];
					}
				}
				else if(dCurStateMap[nX+i-1][nY+j-1]==-1)
				{
					if(dDatabasetMap[nX+i-1][nY+j-1]<0.5)
					{
						dLikelihood[i][j]+=1/dNum[i][j];
					}
				}
			}

		}
	}
}

void KuCombiningPOMapPr::calEmptyLikelihoodPro(double**dLikelihood,double**dDatabasetMap, int nX, int nY,double** dNum,double**dCurStateMap )
{
	int nMaskSize=3;	

	for(int i=0;i<nMaskSize; i++){
		for(int j=0; j<nMaskSize; j++){
			if(dDatabasetMap[nX][nY]<0.5)
			{
				if(dCurStateMap[nX+i-1][nY+j-1]==1)
				{
					if(dDatabasetMap[nX+i-1][nY+j-1]>0.5)
					{
						dLikelihood[i][j]+=1/dNum[i][j];
					}
				}
				else if(dCurStateMap[nX+i-1][nY+j-1]==-1)
				{
					if(dDatabasetMap[nX+i-1][nY+j-1]<0.5)
					{
						dLikelihood[i][j]+=1/dNum[i][j];
					}
				}
			}

		}
	}
}
double  KuCombiningPOMapPr::calPriProValue(int nMaskSize,double** dPosterioriPro,double** dOccLikelihood,double** dEmptyLikelihood,double dXPro )
{
	double dPostPro=1;
	double dOccLH=1;
	double dEmptyLH=1;
	double dPriorPro=0.5;
	bool bPostflag=false;
	bool bOccLHflag=false;
	bool bEmptyLHflag=false;

	for(int i=0;i<nMaskSize; i++){
		for(int j=0; j<nMaskSize; j++){

			if(dPosterioriPro[i][j]!=0)
			{
				dPostPro*=dPosterioriPro[i][j];//P(X|C)C
				bPostflag=true;
			}

			if(dOccLikelihood[i][j]!=0)
			{
				dOccLH*=dOccLikelihood[i][j];//P(C|X1)A
				bOccLHflag=true;
			}

			if(dEmptyLikelihood[i][j]!=0)
			{
				dEmptyLH*=dEmptyLikelihood[i][j];//P(C|X2)B
				bEmptyLHflag=true;
			}
		}
	}


	if((dPriorPro==1||dPriorPro==0)) dPriorPro=0.5;

	double dNumerator=0.0;     
	double dDenominator=0.0; 

	if(bPostflag==false){dPostPro=0.0;}
	if(bOccLHflag==false){dOccLH=0.0;}
	if(bEmptyLHflag==false){dEmptyLH=0.0;}
	if(bPostflag==true&&bEmptyLHflag==false){dEmptyLH=1.0;}


	dNumerator=dXPro*dOccLH;
	dDenominator=dXPro*dOccLH+(1-dXPro)*dEmptyLH;

	if(dDenominator==0)
	{
		dPriorPro=dNumerator;
	}
	else 
		dPriorPro=dNumerator/dDenominator;


	return dPriorPro;
}

void KuCombiningPOMapPr::generateProMap( double**dCurTimeMap,double**dOutMap)
{
	int nMaskSize=3;

	double** dPosterioriPro;
	double** dOccLikelihood;
	double** dEmptyLikelihood;

	double** dPosterioriNum;
	double** dOccLikelihoodNum;
	double** dEmptyLikelihoodNum;

	dPosterioriPro = new double*[nMaskSize];
	dOccLikelihood= new double*[nMaskSize];
	dEmptyLikelihood= new double*[nMaskSize];

	dPosterioriNum = new double*[nMaskSize];
	dOccLikelihoodNum= new double*[nMaskSize];
	dEmptyLikelihoodNum= new double*[nMaskSize];

	for(int i=0; i <nMaskSize; i++){
		dPosterioriPro[i] = new double[nMaskSize];
		dOccLikelihood[i] = new double[nMaskSize];
		dEmptyLikelihood[i] = new double[nMaskSize];

		dPosterioriNum[i] = new double[nMaskSize];
		dOccLikelihoodNum[i] = new double[nMaskSize];
		dEmptyLikelihoodNum[i] = new double[nMaskSize];
	}

	for(int i=0;i<nMaskSize; i++){
		for(int j=0; j<nMaskSize; j++){
			dPosterioriPro[i][j] = 0;
			dOccLikelihood[i][j] = 0;
			dEmptyLikelihood[i][j] = 0;

			dPosterioriNum[i][j] = 0;
			dOccLikelihoodNum[i][j] = 0;
			dEmptyLikelihoodNum[i][j] = 0;
		}
	}


	double** dCurStateMap;

	dCurStateMap = new double*[m_nMapSizeX];

	for(int i=0; i <m_nMapSizeX; i++){
		dCurStateMap[i] = new double[m_nMapSizeY];
	}

	for(int i=0;i<m_nMapSizeX; i++){
		for(int j=0; j<m_nMapSizeY; j++){
			if(dCurTimeMap[i][j]>0.5){dCurStateMap[i][j] = 1;}
			else if(dCurTimeMap[i][j]<0.5){	dCurStateMap[i][j] = -1;}		
			else{dCurStateMap[i][j] =0;	}
		}
	}


	int nProMapNum=m_vecProbabilityMap.size();

	for(int nX=1;nX<m_nMapSizeX-1; nX++){
		for(int nY=1; nY<m_nMapSizeY-1; nY++){
			//---------------------------------------------------------------------------
			for(int i=0;i<nMaskSize; i++){
				for(int j=0; j<nMaskSize; j++){
					dPosterioriPro[i][j] = 0;
					dOccLikelihood[i][j] = 0;
					dEmptyLikelihood[i][j] = 0;

					dPosterioriNum[i][j] = 0;
					dOccLikelihoodNum[i][j] = 0;
					dEmptyLikelihoodNum[i][j] = 0;
				}
			}

			for(int k=0; k<nProMapNum;k++)
			{	//점유 개수 계산
				double** dDatabaseMap=m_vecProbabilityMap[k];
				calPostProNum(dPosterioriNum,dDatabaseMap, nX, nY);//P(X|C); 사후확률 C가 점유되었을때 X가 점유되었을 개수
				calOccLikelihoodNum(dOccLikelihoodNum,dDatabaseMap, nX, nY,dCurStateMap);//P(C|X); X가 점유되었을때 C가 점유될 개수
				calEmptyLikelihoodNum(dEmptyLikelihoodNum,dDatabaseMap, nX, nY,dCurStateMap);//P(C|X); X가 비점유되었을때 C가 점유될 개수 	
			}

			for(int k=0; k<nProMapNum;k++)
			{
				double** dDatabaseMap=m_vecProbabilityMap[k];
				calPostPro(dPosterioriPro,dDatabaseMap, nX, nY,dPosterioriNum );//P(X|C); 사후확률 C가 점유되었을때 X가 점유되었을 확률
				calOccLikelihoodPro(dOccLikelihood,dDatabaseMap, nX, nY,dOccLikelihoodNum ,dCurStateMap);//P(C|X); X가 점유되었을때 C가 점유될 확률
				calEmptyLikelihoodPro(dEmptyLikelihood,dDatabaseMap, nX, nY,dEmptyLikelihoodNum,dCurStateMap );//P(C|X); X가 비점유되었을때 C가 점유될 확률				
			}

			double dXPro=0.0;

			for(int k=0; k<nProMapNum;k++)
			{
				dXPro+=m_vecProbabilityMap[k][nX][nY];
			}
			dXPro=dXPro/(double)nProMapNum;//P(X);

			double dPro=calPriProValue(nMaskSize, dPosterioriPro, dOccLikelihood, dEmptyLikelihood,dXPro );
			dOutMap[nX][nY]=dPro;
			//---------------------------------------------------------------------------		
		}
	}

	for(int i = 0 ; i < nMaskSize ; i++){
		delete[] dPosterioriPro[i];
		delete[] dOccLikelihood[i];
		delete[] dEmptyLikelihood[i];
	}
	delete[] dPosterioriPro;
	delete[] dOccLikelihood;
	delete[] dEmptyLikelihood;


	for(int i = 0 ; i < nMaskSize ; i++){
		delete[] dPosterioriNum[i];
		delete[] dOccLikelihoodNum[i];
		delete[] dEmptyLikelihoodNum[i];
	}
	delete[] dPosterioriNum;
	delete[] dOccLikelihoodNum;
	delete[] dEmptyLikelihoodNum;


	for(int i = 0 ; i < m_nMapSizeX ; i++){
		delete[] dCurStateMap[i];
	}
	delete[] dCurStateMap;	

}

void KuCombiningPOMapPr::loadLastMap(double**dCurTimeMap)
{
	int nSize=m_vecProbabilityMap.size()-1;

	if(m_vecProbabilityMap.size()>0&&m_nLastNum!=-1)
	{
		for(int i = 0 ; i< m_nMapSizeX; i++)
		{
			for(int j = 0; j< m_nMapSizeY ; j++) 
			{
				dCurTimeMap[i][j] = m_vecProbabilityMap[m_nLastNum][i][j];
			}
		}
	}

}

void KuCombiningPOMapPr::execute(string strProMapNameNPath )
{

	loadProMap(strProMapNameNPath);

	double** dCurTimeMap;
	double** dOutMap;

	dCurTimeMap = new double*[m_nMapSizeX];
	dOutMap = new double*[m_nMapSizeX];

	for(int i=0; i <m_nMapSizeX; i++){
		dCurTimeMap[i] = new double[m_nMapSizeY];
		dOutMap[i] = new double[m_nMapSizeY];
	}

	for(int i=0;i<m_nMapSizeX; i++){
		for(int j=0; j<m_nMapSizeY; j++){
			dCurTimeMap[i][j] = 0;
			dOutMap[i][j] = 0;
		}
	}

	loadLastMap(dCurTimeMap );

	generateProMap(dCurTimeMap,dOutMap);

	for(int i=0;i<m_nMapSizeX; i++){
		for(int j=0; j<m_nMapSizeY; j++){
			m_dOutMap[i][j] = dOutMap[i][j];
		}
	}

	for(int i = 0 ; i < m_nMapSizeX ; i++){
		delete[] dCurTimeMap[i];
		delete[] dOutMap[i];
	}
	delete[] dCurTimeMap;
	delete[] dOutMap;

	KuDrawingInfo::getInstance()->setProObBuildingMap(m_dOutMap);

}

double** KuCombiningPOMapPr::getMap( )
{
	return m_dOutMap;
}

