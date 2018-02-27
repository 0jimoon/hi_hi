#include "stdafx.h"
#include "kuMapRepository.h"

/**
 @brief Korean: ���࿡ �ʿ��� ���������� �����ϰ� �ִ� Ŭ�����̴�.
 @brief         �ϳ��� ��ü���� �����ؾ� �ϱ� ������ �̱��� Ÿ������ �����ȴ�. 
 @brief English: write in English
*/
kuMapRepository::kuMapRepository()
{
	cout<<"[kuMapRepository]: Singletone type instance is created!!!"<<endl;

	m_smtpMap = NULL;
	m_pVelocityMap=NULL;
	m_nRefMap=NULL;
	m_bUsingRefMapflag=false;
	m_bCompensationflag=false;
}

kuMapRepository::~kuMapRepository()
{
	if(m_smtpMap!=NULL)
	delete m_smtpMap;
	if(m_pVelocityMap!=NULL)
	delete m_pVelocityMap;


	
	cout<<"[kuMapRepository]: Singletone type instance is destroyed!!!"<<endl;	
}


/**
 @brief Korean: bmp������ ���������� �о��� ���������� �����ϴ� ������ �����Ѵ�. 
 @brief English: write in English
*/
bool kuMapRepository::loadMap(string strMapFilePath)
{
	FILE	*infile;
 	infile = fopen(strMapFilePath.c_str() ,"rb");
 	if(infile==NULL) {
 		cout<< "[CMapRepository] : Error - Could not open map from "<<strMapFilePath<<endl;
		return false;
 	}
	fclose(infile);
	m_strMapFilePath =strMapFilePath;
	m_IplMapImage = cvLoadImage(strMapFilePath.c_str(),1);

	int nMapSizeX = m_IplMapImage->width;
	int nMapSizeY = m_IplMapImage->height;


	if(NULL==m_smtpMap){
		m_smtpMap = new KuMap(nMapSizeX, nMapSizeY); //map�� �������� ���� �� map ����.
	}
	else{//map�� ����� �ٸ� ���. 
		if(nMapSizeX != m_smtpMap->getX() || nMapSizeY !=m_smtpMap->getY())
		{
			delete m_smtpMap;
			m_smtpMap = new KuMap(nMapSizeX, nMapSizeY); 			
		}
	}	
	

	int **nMap=m_smtpMap->getMap();

	for(int nX=0; nX< m_IplMapImage->width; nX++){
		for(int nY=0; nY<m_IplMapImage->height; nY++){
			if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] <= (char)255 && //Blue
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] <= (char)255 && //Green
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] <= (char)255 &&
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] >= (char)180 && //Blue
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] >= (char)180 && //Green
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] >= (char)180){ //Red
			   	
			   	nMap[nX][m_IplMapImage->height-1-nY] = KuMap::EMPTY_AREA;				
			   		
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
			   		m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)0 &&
			   		m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)0){
			   		
					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::OCCUPIED_AREA;
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)255 &&
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)0){ 

				nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::LUGGAGE_AREA;
			
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)0 &&
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)255){ 

					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::SAFE_AREA;					

			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)100 &&
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)0){ 

					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::DYNAMIC_ENVIRONMENT_AREA;
	
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)255 && 
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)0 &&
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)255){ 

					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::FIXED_CAD_AREA;

			}
			else{
				nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::UNKNOWN_AREA;				

			}
		}
	} 

 	cout<< "[CMapRepository] : Map loading success!!!" << endl;	
 	cvReleaseImage(&m_IplMapImage);

	if(true==m_bCompensationflag)
	{
		for(int i=0;i<1;i++){
			compensateMap(m_smtpMap->getX(), m_smtpMap->getY(),m_smtpMap->getMap());
			doMorphologyForOccupiedGrid(m_smtpMap);
		}

		filterMapForOccupiedGrid(m_smtpMap);
		filterMap(m_smtpMap);
		compensateMap(m_smtpMap->getX(), m_smtpMap->getY(),m_smtpMap->getMap());
	}
	KuDrawingInfo::getInstance()->setMap(m_smtpMap); //���� ���� ����

	return true;
}

/**
 @brief Korean: bmp������ ���������� �о��� ���������� �����ϴ� ������ �����Ѵ�. 
 @brief English: write in English
*/
KuMap* kuMapRepository::getMap()
{
	return m_smtpMap;
}
 void kuMapRepository::saveMap(KuMap*pMap,bool bLocalMppingflag)
{
	m_smtpMap=pMap;

	if(bLocalMppingflag==false)
	{
		filterMap(m_smtpMap);
		filterMapForOccupiedGrid(m_smtpMap);
		compensateMap(m_smtpMap->getX(), m_smtpMap->getY(),m_smtpMap->getMap());		
	}
	generateMap(KuRobotParameter::getInstance()->getMapNameNPath(),m_smtpMap );

	KuDrawingInfo::getInstance()->setMap(m_smtpMap);
}
 

void kuMapRepository::compensateMap(int nSizeX, int nSizeY, int** nMap)
{
	//������� �޲ٴ� ��ƾ ����..
	for(int i=1;i<nSizeX-1;i++){
		for(int j=1;j<nSizeY-1;j++){
			if(nMap[i][j] ==KuMap::EMPTY_AREA) { //����� �߿��� �����¿� �� unknown:(0.5) ������ ������
				//1�� �޿�� ���� ����..
				if(nMap[i-1][j] == KuMap::UNKNOWN_AREA ){ //�� �˻�...
					if(m_bUsingRefMapflag==true)
						nMap[i][j] = m_nRefMap[i][j];
				else
						nMap[i][j] = KuMap::OCCUPIED_AREA;
				}
				else if(nMap[i+1][j] == KuMap::UNKNOWN_AREA){ //�� �˻�.
					if(m_bUsingRefMapflag==true)
						nMap[i][j] = m_nRefMap[i][j];
					else
						nMap[i][j] = KuMap::OCCUPIED_AREA;
				}
				else if(nMap[i][j-1]== KuMap::UNKNOWN_AREA){ //�� �˻�.
					if(m_bUsingRefMapflag==true)
						nMap[i][j] = m_nRefMap[i][j];
					else
						nMap[i][j] = KuMap::OCCUPIED_AREA;
				}
				else if(nMap[i][j+1]== KuMap::UNKNOWN_AREA){  //�� �˻�.
					if(m_bUsingRefMapflag==true)
						nMap[i][j] = m_nRefMap[i][j];
					else
						nMap[i][j] = KuMap::OCCUPIED_AREA;
				}
			}
		}
	}

}

void kuMapRepository::filterMap(KuMap* pMap)
{
	//�������� ������ ���� ������ �������ִ� �۾��� �ϴ� �Լ�
	for(int i=0;i<4;i++){
		doMorphologyClose(pMap);
		}

	for(int i=0;i<4;i++){
		doMorphologyOpen(pMap);
	}
}


void kuMapRepository::filterMapForOccupiedGrid(KuMap* pMap)
{
	//�������� ������ ���� ������ �������ִ� �۾��� �ϴ� �Լ�
	for(int i=0;i<3;i++){
		doMorphologyCloseForOccupiedGrid(pMap);
	}

	for(int i=0;i<2;i++){
		doMorphologyOpenForOccupiedGrid(pMap);
	}
}


void kuMapRepository::doMorphologyClose(KuMap* pMap)
{

	int nTmpVal=12345678;
	int **  nMap = pMap->getMap();
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(i<1 || i > pMap->getX()-2 || j<1 || j> pMap->getY()-2){
				continue;
			}

			if(nMap[i][j] ==KuMap::EMPTY_AREA) {
				if(nMap[i-1][j] == KuMap::UNKNOWN_AREA ){ //�� �˻�...
					nMap[i-1][j] = nTmpVal;
				}
				else if(nMap[i+1][j] == KuMap::UNKNOWN_AREA){ //�� �˻�.
					nMap[i+1][j] = nTmpVal;
				}
				else if(nMap[i][j-1]== KuMap::UNKNOWN_AREA){ //�� �˻�.
					nMap[i][j-1] = nTmpVal;
				}
				else if(nMap[i][j+1]== KuMap::UNKNOWN_AREA){  //�� �˻�.
					nMap[i][j+1] = nTmpVal;
				}
			}
		}
	}

	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(nMap[i][j] == nTmpVal ){
				nMap[i][j] = KuMap::EMPTY_AREA;
			}
		}
	}
	
}


void kuMapRepository::doMorphologyOpen(KuMap* pMap)
{
	int nTmpVal=12345678;
	int ** nMap = pMap->getMap();
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(i<1 || i > pMap->getX()-2 || j<1 || j> pMap->getY()-2){
				continue;
			}

			if(nMap[i][j] ==KuMap::EMPTY_AREA) {
				if(nMap[i-1][j] == KuMap::UNKNOWN_AREA ){ //�� �˻�...
					nMap[i][j] = nTmpVal;
				}
				else if(nMap[i+1][j] == KuMap::UNKNOWN_AREA){ //�� �˻�.
					nMap[i][j] = nTmpVal;
				}
				else if(nMap[i][j-1]== KuMap::UNKNOWN_AREA){ //�� �˻�.
					nMap[i][j] = nTmpVal;
				}
				else if(nMap[i][j+1]== KuMap::UNKNOWN_AREA){  //�� �˻�.
					nMap[i][j] = nTmpVal;
				}
			}
		}
	}
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(nMap[i][j] == nTmpVal ){
				nMap[i][j] = KuMap::UNKNOWN_AREA;
			}
		}
	}

}


void kuMapRepository::doMorphologyCloseForOccupiedGrid(KuMap* pMap)
{

	int nTmpVal=12345678;
	int ** nMap = pMap->getMap();
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(i<1 || i > pMap->getX()-2 || j<1 || j> pMap->getY()-2){
				continue;
			}

			if(nMap[i][j] ==KuMap::OCCUPIED_AREA) {
				if( nMap[i-1][j] == KuMap::EMPTY_AREA ){ //�� �˻�...
					nMap[i-1][j] = nTmpVal;
				}
				else if( nMap[i+1][j] == KuMap::EMPTY_AREA){ //�� �˻�.
					nMap[i+1][j] = nTmpVal;
				}
				else if( nMap[i][j-1]== KuMap::EMPTY_AREA){ //�� �˻�.
					nMap[i][j-1] = nTmpVal;
				}
				else if( nMap[i][j+1]== KuMap::EMPTY_AREA ){  //�� �˻�.
					nMap[i][j+1] = nTmpVal;
				}
			}
		}
	}

	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(nMap[i][j] == nTmpVal ){
				nMap[i][j] = KuMap::OCCUPIED_AREA;
			}
		}
	}


}


void kuMapRepository::doMorphologyOpenForOccupiedGrid(KuMap* pMap)
{
	int nTmpVal=12345678;
	int ** nMap = pMap->getMap();
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(i<1 || i > pMap->getX()-2 || j<1 || j> pMap->getY()-2){
				continue;
			}

			if(nMap[i][j] ==KuMap::OCCUPIED_AREA) {
				if( nMap[i-1][j] == KuMap::EMPTY_AREA){ //�� �˻�...
					nMap[i][j] = nTmpVal;
				}
				else if( nMap[i+1][j] == KuMap::EMPTY_AREA){ //�� �˻�.
					nMap[i][j] = nTmpVal;
				}
				else if( nMap[i][j-1]== KuMap::EMPTY_AREA){ //�� �˻�.
					nMap[i][j] = nTmpVal;
				}
				else if( nMap[i][j+1]== KuMap::EMPTY_AREA ){  //�� �˻�.
					nMap[i][j] = nTmpVal;
				}
			}
		}
	}
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(nMap[i][j] == nTmpVal ){
				nMap[i][j] = KuMap::EMPTY_AREA;
			}
		}
	}


}


void kuMapRepository::doMorphologyForOccupiedGrid(KuMap* pMap)
{
	int nTmpVal=12345678;
	int ** nMap = pMap->getMap();
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(i<1 || i > pMap->getX()-2 || j<1 || j> pMap->getY()-2){
				continue;
			}

			if(nMap[i][j] ==KuMap::OCCUPIED_AREA) {
				if( nMap[i-1][j] == KuMap::UNKNOWN_AREA){ nMap[i-1][j] = nTmpVal;}				
				if( nMap[i+1][j] == KuMap::UNKNOWN_AREA){ nMap[i+1][j] = nTmpVal;}				
				if( nMap[i][j-1]== KuMap::UNKNOWN_AREA){ nMap[i][j-1] = nTmpVal;}				
				if( nMap[i][j+1]== KuMap::UNKNOWN_AREA ){ nMap[i][j+1] = nTmpVal;}

// 				if( nMap[i-1][j-1] == KuMap::UNKNOWN_AREA){ nMap[i-1][j] = nTmpVal;}
// 				if( nMap[i-1][j+1] == KuMap::UNKNOWN_AREA){ nMap[i-1][j] = nTmpVal;}
// 				if( nMap[i+1][j-1] == KuMap::UNKNOWN_AREA){ nMap[i+1][j] = nTmpVal;}
// 				if( nMap[i+1][j+1] == KuMap::UNKNOWN_AREA){ nMap[i+1][j] = nTmpVal;}
			}
		}
	}
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(nMap[i][j] == nTmpVal ){
				nMap[i][j] = KuMap::OCCUPIED_AREA;
			}
		}
	}


}
void kuMapRepository::generateMap(string strMapFilePath, KuMap* pMap)
{

	int ** nMap = pMap->getMap();	
	int nMapSizeX = pMap->getX();
	int nMapSizeY = pMap->getY();
	IplImage* IplMapImage = cvCreateImage(cvSize (nMapSizeX, nMapSizeY),IPL_DEPTH_8U, 3);

	for(int LoopX=0 ; LoopX<pMap->getX() ; LoopX++){
		for(int LoopY=0 ; LoopY<pMap->getY(); LoopY++){
			if(	nMap[LoopX][pMap->getY()-1-LoopY] == KuMap::EMPTY_AREA ){ //���
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3]=(char)255; //b
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+1]=(char)255; //g
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+2]=(char)255; //r
			}
			else if(nMap[LoopX][pMap->getY()-1-LoopY]== KuMap::UNKNOWN_AREA ){ //ȸ��
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3]=(char)150; //b
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+1]=(char)150; //g
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+2]=(char)150; //r
			}
			else if(nMap[LoopX][pMap->getY()-1-LoopY] == KuMap::OCCUPIED_AREA ){ //������
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3]=(char)0; //b
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+1]=(char)0; //g
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+2]=(char)0; //r
			}
			else if(nMap[LoopX][pMap->getY()-1-LoopY] == KuMap::FIXED_CAD_AREA ){ //������
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3]=(char)255; //b
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+1]=(char)0; //g
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+2]=(char)255; //r
			}
			else if(nMap[LoopX][pMap->getY()-1-LoopY] == KuMap::LUGGAGE_AREA ){ 
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3]=(char)0; //b
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+1]=(char)255; //g
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+2]=(char)0; //r
			}
			else if(nMap[LoopX][pMap->getY()-1-LoopY] == KuMap::DYNAMIC_ENVIRONMENT_AREA ){ //������
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3]=(char)0; //b
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+1]=(char)100; //g
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+2]=(char)0; //r
			}

			
		}
	}
	int nCurFloor=KuDrawingInfo::getInstance()->getCurFloor();
	char cMapPathName[200];
	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"%s/IH_%dF.png", strMapFilePath.c_str(),nCurFloor);	
	cvSaveImage(cMapPathName,IplMapImage);
	cvReleaseImage(&IplMapImage);
}
/**
 @brief Korean: bmp������ ���������� �о��� ���������� �����ϴ� ������ �����Ѵ�. 
 @brief English: write in English
*/
bool kuMapRepository::loadVelocityMap(string strMapFilePath)
{
	FILE	*infile;
 	infile = fopen(strMapFilePath.c_str() ,"rb");
 	if(infile==NULL) {
 		cout<< "[CMapRepository] : Error - Could not open map from "<<strMapFilePath<<endl;
		return false;
 	}
	fclose(infile);

	m_IplMapImage = cvLoadImage(strMapFilePath.c_str(),1);

	int nMapSizeX = m_IplMapImage->width;
	int nMapSizeY = m_IplMapImage->height;


	if(NULL==m_pVelocityMap){
		m_pVelocityMap = new KuMap(nMapSizeX, nMapSizeY); //map�� �������� ���� �� map ����.
	}
	else{//map�� ����� �ٸ� ���. 
		if(nMapSizeX != m_pVelocityMap->getX() || nMapSizeY !=m_pVelocityMap->getY())
		{
			delete m_pVelocityMap;
			m_pVelocityMap = new KuMap(nMapSizeX, nMapSizeY); 			
		}
	}	
	

	int **nMap=m_pVelocityMap->getMap();

	for(int nX=0; nX< m_IplMapImage->width; nX++){
		for(int nY=0; nY<m_IplMapImage->height; nY++){//���
			if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)255 && //Blue
			   m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)255 && //Green
			   m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)255	){ //Red 
			   	
			   	nMap[nX][m_IplMapImage->height-1-nY] = KuMap::NOMALVELOCITY;				
			   		
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
			   		m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)0 &&
			   		m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)255){//����
			   		
					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::FIRDECEL;
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)255 &&
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)255){//���

					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::SECDECEL ;
					
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)255 &&
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)0){//�ʷ�

					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::THIDECEL ;					
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)255 && 
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)0 &&
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)0){//�Ķ�

					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::FOUDECEL ;					
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)255 && 
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)0 &&
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)255){//����

					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::FIFDECEL ;					
			}
			else{
				nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::UNKNOWN_AREA;				

			}
		}
	} 

 	cout<< "[CMapRepository] : Map loading success!!!" << endl;	
 	cvReleaseImage(&m_IplMapImage);
 
	return true;
}
int **kuMapRepository::getVelocityMap()
{
	return m_pVelocityMap->getMap();
}


bool kuMapRepository::loadRefMap(string strMapFilePath,int nMapSizeX,int nMapSizeY)
{
	FILE	*infile;
	infile = fopen(strMapFilePath.c_str() ,"rb");
	if(infile==NULL) {
		cout<< "[CMapRepository] : Error - Could not open map from "<<strMapFilePath<<endl;
		return false;
	}
	fclose(infile);

	Mat sourceImage=imread(strMapFilePath.c_str(),CV_LOAD_IMAGE_COLOR);
	if(!sourceImage.data) return -1;
	

	int nCADMapSizeX = sourceImage.rows;
	int nCADMapSizeY = sourceImage.cols;
	int nCntX=0;
	int nCntY=0;
	int nTotalCnt=1;


	if(nCADMapSizeX>=nMapSizeX)
	{
		int ntempX=nCADMapSizeX/nMapSizeX;
		int ntempsubX=nCADMapSizeX%nMapSizeX;
		if(ntempsubX>0)
		{
			nCntX=ntempX+1;
		}
		else
		{
			nCntX=ntempX;
		}
	}

	if(nCADMapSizeY>=nMapSizeY)
	{
		int ntempY=nCADMapSizeY/nMapSizeY;
		int ntempsubY=nCADMapSizeY%nMapSizeY;
		if(ntempsubY>0)
		{
			nCntY=ntempY+1;
		}
		else
		{
			nCntY=ntempY;
		}
	}
	nTotalCnt=nCntY*nCntX;

	CCADMapData CADMapData;
	Mat subimage;
	for(int i=0; i<nCntX;i++)
	{
		for(int j=0; j<nCntY;j++)
		{
			Rect rect(i*nMapSizeX,j*nMapSizeY,nMapSizeX,nMapSizeY);
			subimage=sourceImage(rect);
			CADMapData.x=i*nMapSizeX;
			CADMapData.y=j*nMapSizeY;
			CADMapData.imageSizeX=nMapSizeX;
			CADMapData.imageSizeY=nMapSizeY;
			CADMapData.sourceImage=subimage;
			m_vecCADMapData.push_back(CADMapData);
		}
	}

	for(int i=0; i<m_vecCADMapData.size();i++)
	{
		imshow("sistarROI",m_vecCADMapData[i].sourceImage); //Show image
		waitKey(0); //Wait for keystroke
	}

	if(m_nRefMap==NULL)
	{
		m_nRefMap = new int*[nMapSizeX];
		if(m_nRefMap){
			for(int i = 0 ; i < nMapSizeX ; i++){
				m_nRefMap[i] = new int[nMapSizeY];
			}
		}
	}
	int nIDX=1;
	for(int nX=0; nX< m_vecCADMapData[nIDX].imageSizeX; nX++){
		for(int nY=0; nY<m_vecCADMapData[nIDX].imageSizeY; nY++){

		
			if(m_vecCADMapData[nIDX].sourceImage.data[(nX+nY*m_vecCADMapData[nIDX].imageSizeX)*3] == (char)255 && //Blue
				m_vecCADMapData[nIDX].sourceImage.data[(nX+nY*m_vecCADMapData[nIDX].imageSizeX)*3 +1]== (char)255 && //Green
				m_vecCADMapData[nIDX].sourceImage.data[(nX+nY*m_vecCADMapData[nIDX].imageSizeX)*3 +2] == (char)255	){ //Red 

					m_nRefMap[nX][nMapSizeY-1-nY] = KuMap::EMPTY_AREA;		

			}
			else if(m_vecCADMapData[nIDX].sourceImage.data[(nX+nY*m_vecCADMapData[nIDX].imageSizeX)*3]== (char)0 && 
				m_vecCADMapData[nIDX].sourceImage.data[(nX+nY*m_vecCADMapData[nIDX].imageSizeX)*3 +1]== (char)0 &&
				m_vecCADMapData[nIDX].sourceImage.data[(nX+nY*m_vecCADMapData[nIDX].imageSizeX)*3 +2] == (char)0){

					m_nRefMap[nX][nMapSizeY-1-nY] =  KuMap::FIXED_CAD_AREA;
			}
			else if(m_vecCADMapData[nIDX].sourceImage.data[(nX+nY*m_vecCADMapData[nIDX].imageSizeX)*3] == (char)0 && 
				m_vecCADMapData[nIDX].sourceImage.data[(nX+nY*m_vecCADMapData[nIDX].imageSizeX)*3 +1]== (char)0 &&
				m_vecCADMapData[nIDX].sourceImage.data[(nX+nY*m_vecCADMapData[nIDX].imageSizeX)*3 +2]== (char)255){

					m_nRefMap[nX][nMapSizeY-1-nY] =  KuMap::VIRTUAL_WALL_AREA;

			}
			else{
				m_nRefMap[nX][nMapSizeY-1-nY] =  KuMap::UNKNOWN_AREA;				
			}

		}
	}
		KuDrawingInfo::getInstance()->setCADMap(m_nRefMap,nMapSizeX,nMapSizeY);

	return true;
}

int** kuMapRepository::getRefMap()
{
	return m_nRefMap;
}