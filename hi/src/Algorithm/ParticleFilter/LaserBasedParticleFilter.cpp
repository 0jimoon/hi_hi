#include "stdafx.h"
#include "LaserBasedParticleFilter.h"

/**
 @사용법.
 @ 1. SetSampleRegion() 함수를 이용하여 샘플이 뿌려질 영역 설정. 
 @ 2. ResetSamples() 함수를 이용하여 샘플을 초기화시킴.
 @ 1,2는 생성자에서 초기설정값으로 실행. 위치추정 실패나 키드냅 등으로 인하여 다시 전역위치추정을 수행할 때 1,2 순서로 호출하면 OK.
 @ 3. SetRangeSensorParameter() 함수를 이용하여 거리센서 파라미터를 설정.
 @ 4. SetMapData() 함수를 이용하여 지도정보 전달. 중간에 지도를 수정하는 것은 안만들었음.
 @ 5. MCLStart(), MCLStop() 함수를 이용하여 MCL 수행여부를 설정. 실행 중에 MCLStop()을 하면 센서를 이용한 업데이트는 수행 안하고 모션모델만 적용.
 @ 6. SetEncoderData() 함수를 이용하여 엔코더의 변화량 입력.
 @ 7. SetRangeData() 함수를 이용하여 거리센서값 입력. 거리센서값은 최대값 이상의 값은 최대값으로 변경하여 저장되어 있어야 함.
 @ 8. GetSamplePos() 함수를 이용하여 결과값 리턴. 입력 파라미터는 1. 추정된 위치, 2. 추정된 위치예서 예상되는 거리센서값 리턴을 위한 변수.
 @ 9. 6,7,8을 반복하면 됨.
 @ 다른 센서를 사용할 경우, 센서값을 입력받고, 센서모델을 이용한 확률 갱신을 추가하면 됨.
 @ m_nMaxSampleNum은 전역위치추정 시 뿌릴 샘플의 수, m_nMinSampleNum은 위치트래킹시 뿌릴 최소한의 샘플 수.
 @ m_nMaxSampleNum은 필요하다면 아무때나 변경하면 됨.
 @ m_nMinSampleNum은 100이상으로 추천함.
*/

LaserBasedParticleFilter::LaserBasedParticleFilter()
{
	m_nResamplingCnt =0;
	
	m_nMaxSampleNum =500;		// 샘플의 갯수의 최대값. set2DMode()에서도 바꿔줘야 함.
	m_nMinSampleNum =500;			// 샘플의 갯수의 최소값. set2DMode()에서도 바꿔줘야 함.
	m_nSampleNum = m_nMaxSampleNum;
	m_dRangeSensorOffset = 0.0;
	m_bDoRangeUpdate = false;
    m_bResetSamples = false;
	m_bMCLWorkingNow = false;
    m_bMCLWork = true;
    m_nMCLReturnValue = 1;
	m_dMaxProForUpdate = 0.8;
	m_dMinProForUpdate = 0.02;
    m_dAccumulatedRotation = 0.0;
    m_dAccumulatedDistance = 0.0;
    m_stSample = new Sample[m_nMaxSampleNum];

	m_nMapSizeX = 0;
	m_nMapSizeY = 0;
	srand((unsigned)time(NULL)); //랜덤 씨드를 항상 바꿔주기 위해서 설정해야한다.

	m_fRangeMCLPro = NULL;
	m_nGridMap=NULL;
	  m_dDeviationforTransConverced = 0.20;
	 m_dDeviationforRotateConverced = 0.10;
	 m_dDeviationforTransRotateConverced= M_PI/180.0*0.02;
	 m_nMinSampleStandardDeviation= 100;
	 m_nSampleDensity=200;
	 m_dSampleSD = m_nMinSampleStandardDeviation;

}

LaserBasedParticleFilter::~LaserBasedParticleFilter()
{
	delete [] m_stSample;
	if(m_fRangeMCLPro!=NULL){
		for(int i=0;i<(int)(m_dMaxDistance/10.0)+1;i++){
			free(m_fRangeMCLPro[i]);
		}
		free (m_fRangeMCLPro);
	}

}
/**
 @brief Korean: 거리 센서 모델의 거리 따른 확률 값
 @brief English: 
*/
void LaserBasedParticleFilter::initProbability()
{

	// 모션모델에서 계산되는 확률의 범위. 0~1.0사이. 센서모델에서 계산되는 범위를 정할 때 이 범위를 고려해야 함.
	m_dMaxProForUpdate = 0.8;
	m_dMinProForUpdate = 0.2;

	// range-based MCL에서 거리차에 따른 확률을 미리 계산
	m_fRangeMCLPro = (float**)calloc((int)(m_dMaxDistance/10.0)+1,sizeof(float*));
	for(int i=0;i<(int)(m_dMaxDistance/10.0)+1;i++) {
		m_fRangeMCLPro[i] = (float*)calloc((int)(m_dMaxDistance/10.0)+1,sizeof(float));
	}
	float s;
	for(int i=0;i<(int)(m_dMaxDistance/10.0)+1;i++) {	// expected distance
		s = 0.006*(float)i/100.0 + 0.150;
		for(int j=0;j<(int)(m_dMaxDistance/10.0)+1;j++) {	// measured distance
			m_fRangeMCLPro[i][j] = 0.05*exp(-((float)i*0.01-(float)j*0.01)*((float)i*0.01-(float)j*0.01)/2/s/s)
									+ 0.005*((float)m_dMaxDistance-(float)j*10.0)/(float)m_dMaxDistance;
			if(((float)j*10.0)>((float)m_dMaxDistance-300.0))
				m_fRangeMCLPro[i][j] = (float)0.10;
			// 모션모델의 확률범위를 고려하여 범위를 맞춰주는 부분. (0~1.0사이 조절)*(가중치)
			m_fRangeMCLPro[i][j] *= (10.0 * 1.5);
		}
	}

}


/**
 @brief Korean: 지도정보를 저장함
 @brief English:  set grid map date
*/
void LaserBasedParticleFilter::setMap(int sizeX, int sizeY, int** nGridMap)
{
	
	if(m_nGridMap==NULL)
	{
		m_nGridMap = new int*[sizeX];
		if(m_nGridMap){
			for(int i = 0 ; i < sizeX ; i++){
				m_nGridMap[i] = new int[sizeY];

			}
		}
	}


	for(int i = 0; i<sizeX; i++){
		for(int j = 0; j< sizeY;j++){

			if( nGridMap[i][j]==KuMap::FIXED_CAD_AREA)
			{
				m_nGridMap[i][j] = KuMap::OCCUPIED_AREA;
			}
			else	m_nGridMap[i][j] = nGridMap[i][j];

		}
	}


	m_nMapSizeX = sizeX;
	m_nMapSizeY = sizeY;

	// 샘플이 뿌려지는 영역에 대한 좌표 초기화. 지도 전체에 뿌림. 전역위치추정.
	SetSampleRegion(100, m_nMapSizeX*100-100, 100, m_nMapSizeY*100-100);
	resetSamples();
}

/**
 @brief Korean:  갱신 된 지도정보를 저장함
 @brief English:  set  updated grid map date
*/
void LaserBasedParticleFilter::updateMapData(int sizeX, int sizeY, int **nGridMap)
{

	for(int i = 0; i<sizeX; i++){
		for(int j = 0; j< sizeY;j++){
			m_nGridMap[i][j] = nGridMap[i][j];
		}
	}
	//printf("[CLaserBasedParticleFilter]: map updated\n");

	m_nMapSizeX = sizeX;
	m_nMapSizeY = sizeY;

	// 샘플이 뿌려지는 영역에 대한 좌표 초기화. 지도 전체에 뿌림. 전역위치추정.
	SetSampleRegion(100, m_nMapSizeX*100-100, 100, m_nMapSizeY*100-100);
	resetSamples();
}

/**
 @brief Korean: MCL를 종료 시킴
 @brief English:  activate or deactivate MCL
*/
void LaserBasedParticleFilter::MCLStop()
{

	// 동적배열 사용하는 부분이 다 끝나면 MCL을 끝내도록.... 메인문의 exit()직전에 수행.
    m_nSampleNum = 0;
	m_bMCLWork = false;
	m_bCalculationStop = true;
	while(m_bMCLWorkingNow)
	{
		////TRACE("MCL is stopping....\n");
	}

}

/**
 @brief Korean: 각 샘플의 위치와 확률값을 평균하여 로봇의 위치 추정
 @brief English:  estimate robot pose averaging all poses of samples with probabilities
*/
int LaserBasedParticleFilter::getSamplePos(double dPos[3], double dEstimatedRangeData[181])
{
  if (m_bMCLWork==false) return 3;
	m_bMCLWorkingNow = true;


	double dSum = 0.0;
   	if(false==DoMCL()&& m_nMCLReturnValue == MCL_ERROR_ALLSAMPLES_DISAPPEARED){ // MCL 수행.
   		//샘플이 지도밖으로 모두 사라진 경우
   		// 샘플이 뿌려지는 영역에 대한 좌표 초기화. 지도 전체에 뿌림. 전역위치추정.
		SetSampleRegion(STARTPOS_MARGIN, m_math.AI2MM(m_nMapSizeX)-ENDPOS_MARGIN,
						STARTPOS_MARGIN, m_math.AI2MM(m_nMapSizeY)-ENDPOS_MARGIN);
		resetSamples();
		m_bMCLWorkingNow = false;
		return m_nLocalizationState=10;
   	}

    // MCL이 수행되는 중간에 샘플 갯수를 임의로 변경하면 안됨.
    //if (m_bResetSamples) ResetSamples();
   	if(m_bResetSamples) resetSamplesNearRobot(m_nRobotPosForLocalTracking[0],m_nRobotPosForLocalTracking[1],
   											  m_nRobotPosForLocalTracking[2],m_nRobotPosForLocalTracking[3]);


	// 180도 지점에서 각도가 179에서 -179로 변하는 문제를 처리하기 위해.
// 	if (m_dSamplePos[2]>120*D2R || m_dSamplePos[2]<-120*D2R) bAngleCalculationMode = true;
// 	else bAngleCalculationMode = false;
	// 새로운 위치를 계산하기 위해 변수 초기화.
	m_dSamplePos[0] = 0.0;
	m_dSamplePos[1] = 0.0;
	m_dSamplePos[2] = 0.0;


	// 확률값과 위치값을 이용한 평균.
	double dx, dy;
	dx = 0; dy = 0;
	for(int i=0; i<m_nSampleNum; i++)  {
		dSum += m_stSample[i].dPro;
		m_dSamplePos[0] += m_stSample[i].x*m_stSample[i].dPro;
		m_dSamplePos[1] += m_stSample[i].y*m_stSample[i].dPro;
	//if(bAngleCalculationMode && m_stSample[i].t<0) m_dSamplePos[2] += (m_stSample[i].t+2.0*M_PI)*m_stSample[i].dPro;
	//else m_dSamplePos[2] += m_stSample[i].t*m_stSample[i].dPro;
		dx += cos(m_stSample[i].t)*m_stSample[i].dPro;
		dy += sin(m_stSample[i].t)*m_stSample[i].dPro;
	}

    if (dSum!=0.0) {
    	m_dSamplePos[0] = m_dSamplePos[0]/dSum;
	    m_dSamplePos[1] = m_dSamplePos[1]/dSum;
		m_dSamplePos[2] = atan2(dy/dSum, dx/dSum);
	    if (m_dSamplePos[2]>M_PI) m_dSamplePos[2] -= 2.0*M_PI;
    	else if (m_dSamplePos[2]<-M_PI) m_dSamplePos[2] += 2.0*M_PI;
    }

	// 값 전달.
	dPos[0] = m_dSamplePos[0];
	dPos[1] = m_dSamplePos[1];
	dPos[2] = m_dSamplePos[2];


	// 추정한 위치에서 예상되는 거리센서값 저장.
	predictRangeData(dPos[0], dPos[1], dPos[2], m_dRangeSensorAngleInterval);
	memcpy(dEstimatedRangeData, m_dEstimatedRangeData, sizeof(m_dEstimatedRangeData));

	m_bMCLWorkingNow = false;
	return m_nLocalizationState;

}

/**
 @brief Korean: 
 @brief English:  
*/
void LaserBasedParticleFilter::CheckLocalizationState()
{
//	cout << "SD = "<<m_dSampleSD <<" " <<"MME = "<<m_dMinMatchingError<<endl;

	if(fabs(m_dSampleSD) > 350) //원래 250;
		m_nLocalizationState = 1;

//	else if(m_dMinMatchingError > 0.60)
//		m_nLocalizationState = 2;
	else
		m_nLocalizationState = 0;

}

/**
 @brief Korean: MCL을 시작한다
 @brief English:  do MCL
*/
bool LaserBasedParticleFilter::DoMCL()
{
	int nCnt1 = 0;
    bool bMotionModelResult;

    m_bCalculationStop = false;

	//-----------------------------------------------------------------------------------//
	// 모션모델 적용
	bMotionModelResult = MotionModel();			// 엔코더 정보를 이용하여 각 샘플에 모션모델 적용.

	// 모션모델 적용 후 샘플이 모두 사라졌다면(장애물로 돌진) 에라처리.
	if (bMotionModelResult==false) {

		m_nMCLReturnValue = MCL_ERROR_ALLSAMPLES_DISAPPEARED;
		return false;
	}
    //-----------------------------------------------------------------------------------//

	//-----------------------------------------------------------------------------------//
	// 로봇이 움직이지 않는 경우는 업데이트 안함.

	if ( m_nLocalizationState==10) {
		if (CheckMovementForUpdateOrNot(0)==false) {
			return true;
		}
	}
	else {
		if ( CheckMovementForUpdateOrNot(1)==false) {
			return true;
		}
	}



	//-----------------------------------------------------------------------------------//
	// 거리센서를 사용한 업데이트.
	if (m_bDoRangeUpdate ) {
		//cout<<"update"<<endl;
		if (!UpdateUsingRangeSensor()) {
			m_bDoRangeUpdate = false;
			return false;
		}

		for(int i = 0; i<m_nSampleNum; i++)
		{
			if (m_bCalculationStop) return false;
			//cout<<"sample probability: "<< m_stSample[i].dPro <<endl;
			m_stSample[i].dPro = m_stSample[i].dPro * m_stSample[i].dRangeUpdatePro;
		}

	}

	//-----------------------------------------------------------------------------------//
	// 확률 정규화
	Normalizing();
	//-----------------------------------------------------------------------------------//

	//added CYK
	if(m_nLocalizationState != 10)
	{
		CheckLocalizationState();
	}
	// 모션모델만 수행되어 샘플이 퍼지는 경우.
	m_nMCLReturnValue = MCL_DIVERGING_NOW;
	if ( m_bDoRangeUpdate  ){
		m_nResamplingCnt++;
		Resampling();
	}

	//if (m_nSampleNum<=m_nMinSampleNum && m_nLocalizationState==10) m_nLocalizationState= 0;
	// 전역위치추정 시 샘플이 수렴하면 local tracking 모드로 변환
	// 위치 표준편차가 MINSAMPLESTANDARDDEVIATION의 3배보다 작고, 각도 표준편차가 10도 보다 작으면 수렴했다고 판단.
	// 수렴을 제대로 했는지 판단하는 상태로 전환
	if ( m_nLocalizationState==10 && m_nSampleNum<=m_nMinSampleNum
		&& fabs(m_dSampleSD) < m_nMinSampleStandardDeviation*3.0
		&& fabs(m_dSampleSDForRotation) < 10.*M_PI/180.){
		m_nLocalizationState = 0;
	}

	m_bDoRangeUpdate = false;
	return true;
}


/**
 @brief Korean:  모션 모델을 기반으로 하는 샘플들의 이동
 @brief English:   move each sample based on motion model
*/
bool LaserBasedParticleFilter::MotionModel()
{
	double dEncoderDelta2[3];
	int nCnt = 0;
	double s1,c1;
	double dPro;
	double d1, d2, d3;
	double dNoise;
	d1 = (double)m_dDeviationforTransConverced;
	d2 = (double)m_dDeviationforTransRotateConverced;
	d3 = (double)m_dDeviationforRotateConverced;

	if (m_dEncoderData[0]==0 && m_dEncoderData[1]==0 && m_dEncoderData[2]==0) return true;

	// 노이즈 추가. Konolige가 유도한 모션모델 사용.
	for (int i=0; i<m_nSampleNum; i++) {
		dNoise = GetRandValue(m_dEncoderData[0]*d1);	// x 방향 오차는 x 방향 이동량에만 비례.
		if (dNoise>0) dNoise /= 2.0;
		dEncoderDelta2[0] = m_dEncoderData[0] + dNoise;
		dEncoderDelta2[1] = m_dEncoderData[1];
		dEncoderDelta2[2] = m_dEncoderData[2]									// 각도 오차는,
											+ GetRandValue(m_dEncoderData[0]*d2)	// x 방향 이동량과
											+ GetRandValue(m_dEncoderData[2]*d3);

		s1 = sin(m_stSample[i].t+dEncoderDelta2[2]/2.0);	// yaw
		c1 = cos(m_stSample[i].t+dEncoderDelta2[2]/2.0);


		// 엔코더 정보를 이용하여 샘플의 절대좌표계 상에서의 이동을 계산.
		m_stSample[i].x += (c1*dEncoderDelta2[0] + (-s1)*dEncoderDelta2[1]);
		m_stSample[i].y += (s1*dEncoderDelta2[0] + (c1)*dEncoderDelta2[1]);
		m_stSample[i].t += dEncoderDelta2[2];
		if (m_stSample[i].t>M_PI) m_stSample[i].t -=2*M_PI;
		else if (m_stSample[i].t<-M_PI) m_stSample[i].t +=2*M_PI;

		// 이동량에 따른 확률 갱신.
		// 엔코더로 예측한 값이 정확하다면 엔코더 변화량과 노이즈가 추가된 변화량이 일치해야 하므로,
		// 엔코더 변화량과 노이즈가 추가된 변화량 사이의 차이가 작을수록 높은 확률값 부여.
		//dPro = GetGaussianValue(hypot(m_dEncoderData[0], m_dEncoderData[1])*d1*2.0
		//	,fabs(hypot(m_dEncoderData[0], m_dEncoderData[1]) - hypot(dEncoderDelta2[0], dEncoderDelta2[1])) );
		dPro = GetGaussianValue(m_dEncoderData[0]*d1*2.0, dNoise);
		if (dPro<m_dMinProForUpdate)
			dPro = m_dMinProForUpdate;
		if (dPro>m_dMaxProForUpdate)
			dPro = m_dMaxProForUpdate;

		m_stSample[i].dPro *= dPro;

		// --------------------------------------- 제거할 샘플을 제거하는 부분 --------------------- //
		// 지도 범위 바깥에 추출된 샘플은 무시.
		if (m_stSample[i].x<STARTPOS_MARGIN*3 || m_stSample[i].x >= m_math.AI2MM(m_nMapSizeX-3)
			|| m_stSample[i].y< STARTPOS_MARGIN*3  || m_stSample[i].y >= m_math.AI2MM(m_nMapSizeY-3)){

			m_stSample[i].dPro = 0.0;
		}

		else if ( m_nGridMap[(int)(0.5+m_stSample[i].x/100.0)][(int)(0.5+m_stSample[i].y/100.0)] != 0)
				m_stSample[i].dPro = 0.0;
		// 온전한 것이 몇개인지 셈.
		else nCnt++;
		// ----------------------------------------------------------------------------------------- //
	}

	if (nCnt==0){
		return false;	// 살아남은 샘플이 없을 때
	}
	else{
		return true;
	}
}


/**
 @brief Korean: 거리기반한 샘플들의 확률 업데이트
 @brief English:  update probabilities of samples using range valuesl
*/
bool LaserBasedParticleFilter::UpdateUsingRangeSensor()
{
	int i,j;
	int nCnt;

	nCnt = 0;
	for (j=0;j<m_nNoOfRangeSensorPoint;j=j+m_dRangeSensorAngleInterval) {
		if (m_dRangeData[j] >= m_dMaxDistance) {
			m_dRangeData[j] = m_dMaxDistance;
		}
		else if (m_dRangeData[j] == 0){
			m_dRangeData[j] = m_dMaxDistance;
		}
	}

	for (i=0; i<m_nSampleNum; i++) {
		if (m_bCalculationStop){
			return false;
		}

		predictRangeData(m_stSample[i].x, m_stSample[i].y , m_stSample[i].t, m_dRangeSensorAngleInterval);

		nCnt = 0;
		m_stSample[i].dRangeUpdatePro = 0.0;
		m_stSample[i].dMatchingError = 0.0; //added CYK

		for (j=0;j<m_nNoOfRangeSensorPoint;j=j+m_dRangeSensorAngleInterval) {
			if(m_dRangeData[j] >= m_dMaxDistance){ //added CYK 센싱안되는것(25m이상)은 확률계산에 사용하지 않음
				continue;
			}
			if (m_dEstimatedRangeData[j]==0){
				m_dEstimatedRangeData[j] = m_dMaxDistance;
			}
			if (m_dEstimatedRangeData[j] >= m_dMaxDistance){
				m_dEstimatedRangeData[j] = m_dMaxDistance;
			}

			m_stSample[i].dRangeUpdatePro += (double)m_fRangeMCLPro[(int)(0.5 + m_dEstimatedRangeData[j]/10.0)][(int)(0.5 + m_dRangeData[j]/10.0)];
			nCnt++;
//		m_stSample[i].dRangeUpdatePro *=pow((double)m_fRangeMCLPro[(int)(0.5 + m_dEstimatedRangeData[j]/10.0)][(int)(0.5 + m_dRangeData[j]/10.0)],0.02); // 0.2
		}

		if (nCnt>0){
			m_stSample[i].dRangeUpdatePro /= (double)nCnt;
			m_stSample[i].dMatchingError /= (double)nCnt;	//added CYK
			m_dMinMatchingError += m_stSample[i].dMatchingError;
		}

	}
	//cout<<"[CLaserBasedParticleFilter]: Update using range data"<<endl;
	return true;

}

/**
 @brief Korean: 확률값을 이용하여 샘플 복사 및 확률값 정규화
 @brief English:   re-distribute samples according to prior probabilities with uniform probability
*/
void LaserBasedParticleFilter::Resampling()
{
	Accumulation *pAccumulation;

	double dSum = 0.0;
	int nNoOfAccumulated;
	int nCnt;
	double dRandPro;
	int i;

	pAccumulation = NULL;
	pAccumulation = (Accumulation*)calloc(m_nSampleNum,sizeof(Accumulation));

	//TRACE("SD: %.3f\n", m_dSampleSD);
	if (m_dSampleSD<m_nMinSampleStandardDeviation)
	{
		CalculateNoOfSamples(false);
		//cout<<"[CLaserBasedParticleFilter] : No sensor update\n"<<endl;
	}
	else
	{
		// 확률에 따른 resampling을 위한 확률누적.
		nNoOfAccumulated=0;
		for (i =0; i<m_nSampleNum; i++) {
			if (m_stSample[i].dPro==0.0) continue;
			dSum += m_stSample[i].dPro;
			pAccumulation[nNoOfAccumulated].dSum = dSum;
			pAccumulation[nNoOfAccumulated].x = m_stSample[i].x;
			pAccumulation[nNoOfAccumulated].y = m_stSample[i].y;
			pAccumulation[nNoOfAccumulated].t = m_stSample[i].t;
			nNoOfAccumulated++;
		}

		// 모든 샘플이 벽속으로 들어가 버리면 resampling 단계에서 복사할 수 있는 샘플이 없음.
		if (nNoOfAccumulated==0) {
			//TRACE("\n\nError: All sampels get into the obstacles. Any new sample cannot be generated.\n\n");
			while(1){
				//CWheelActuatorInterface::getInstance()->stop();
			}

			m_nMCLReturnValue = MCL_ERROR_ALLSAMPLES_DISAPPEARED;
			//exit(0);
		}

		// 샘플의 위치의 표준편차를 이용하여 다음 단계 샘플의 갯수를 계산.
		CalculateNoOfSamples(true);

		// 샘플 갯수에 따른 정규화된 확률값.
		m_dBasePro = 1.0/(double)m_nSampleNum;
		int nTemp=0;

		// 확률값이 높은 샘플은 많이 복사하고, 확률값이 낮은 샘플은 적게 복사하는 부분.
		// 지정한 갯수(m_nSampleNum)만큼 새로운 샘플 복사.
		while(nTemp<m_nSampleNum) {
			nCnt = 0;
			dRandPro = (double)rand()/(double)RAND_MAX;
			while (pAccumulation[nCnt].dSum<dRandPro && nCnt<nNoOfAccumulated-1){
				nCnt++;
			}
			m_stSample[nTemp].x = pAccumulation[nCnt].x;
			m_stSample[nTemp].y = pAccumulation[nCnt].y;
			m_stSample[nTemp].t = pAccumulation[nCnt].t;
			m_stSample[nTemp].dPro = m_dBasePro;
			nTemp++;

		}

	}
	// 작업용 동적배열 삭제.
	free (pAccumulation);
	pAccumulation = NULL;
}

int LaserBasedParticleFilter::getResamplingCnt()
{
	return m_nResamplingCnt;
}


/**
 @brief Korean: 지정된 영역에 샘플을 골고루 뿌리고 확률값을 초기화시킴.
 @brief English: distribute samples uniformly on predefined region with normalized probabilities.
*/
void LaserBasedParticleFilter::resetSamples()
{
	m_nResamplingCnt = 0;
	
	////TRACE("MCL: before reset samples\n");
	int nCnt = 0;

	m_bResetSamples = false;
	m_nLocalizationState = 10;
	m_dSampleSD = m_nMinSampleStandardDeviation;

	// MCL 결과 저장 변수 초기화.
	m_nMCLReturnValue = MCL_CONVERGING_YET;

	// 샘플이 뿌려질 영역 및 SAMPLE_DENSITY값에 따른 샘플 갯수 계산.
	m_nSampleNum = (int)((double)(m_nSampleRegion[3]-m_nSampleRegion[2])/3.0/1000.0*(double)(m_nSampleRegion[1]-m_nSampleRegion[0])/3.0/1000.0*m_nSampleDensity);

	// 최대값 이상, 최소값 이하의 샘플 갯수가 계산되면 최대값, 최소값으로 대치.
	if (m_nSampleNum > m_nMaxSampleNum) m_nSampleNum = m_nMaxSampleNum;
	if (m_nSampleNum < m_nMinSampleNum) m_nSampleNum = m_nMinSampleNum;

	// 정규화된 샘플의 확률값.
	m_dBasePro = 1.0/(double)m_nSampleNum;

	//TRACE("%d, %d, %d, %d\n", m_nSampleRegion[0], m_nSampleRegion[1], m_nSampleRegion[2], m_nSampleRegion[3]);
	while(nCnt<m_nSampleNum)
	{
		// 샘플이 뿌려지는 영역을 고려하여 임의의 x, y, theta 값 입력.
		m_stSample[nCnt].x = (double)m_nSampleRegion[0] + (float)rand()/(float)RAND_MAX*(double)(m_nSampleRegion[1]-m_nSampleRegion[0]);
		m_stSample[nCnt].y = (double)m_nSampleRegion[2] + (float)rand()/(float)RAND_MAX*(double)(m_nSampleRegion[3]-m_nSampleRegion[2]);
		m_stSample[nCnt].t = (0.5 - (float)rand()/(float)RAND_MAX) *2.0*M_PI;

		// 지도 범위 바깥에 추출된 샘플은 무시.
		if ((int)(0.5+m_stSample[nCnt].x/100.)<0 || (int)(0.5+m_stSample[nCnt].x/100.) >= m_nMapSizeX
			|| (int)(0.5+m_stSample[nCnt].y/100.) < 0 || (int)(0.5+m_stSample[nCnt].y/100.) >= m_nMapSizeY ){
				continue;
		}


		if ( (m_nGridMap[(int)(0.5+m_stSample[nCnt].x/100.0)][(int)(0.5+m_stSample[nCnt].y/100.0)] == UNKNOWN)
			|| (m_nGridMap[(int)(0.5+m_stSample[nCnt].x/100.0)][(int)(0.5+m_stSample[nCnt].y/100.0)] == WALL) ){
			continue;
		 }

		// 확률값 초기화.
		m_stSample[nCnt].dPro = m_dBasePro;
		nCnt++;
	}


	// 그림그리기 위해 샘플정보 한 번 넘겨주기
	m_nOldSampleNum = m_nSampleNum;
	m_vecOldSample.clear();
	Sample stSample;
	for (int i=0; i<m_nSampleNum; i++) {
	  	if (m_stSample[i].dPro==0.0) continue;
		stSample.x = m_stSample[i].x;
		stSample.y = m_stSample[i].y;
		stSample.t = m_stSample[i].t;
		stSample.dPro = m_stSample[i].dPro;
		stSample.dRangeUpdatePro = m_stSample[i].dRangeUpdatePro;
		m_vecOldSample.push_back(stSample);
	}
	m_vecOldSampleCopy = m_vecOldSample;
}
/**
 @brief Korean: 
 @brief English: 
*/
void LaserBasedParticleFilter::setSamplesLocalTracking(double x, double y, double t, double dSize)
{
	m_nRobotPosForLocalTracking[0] = x;
	m_nRobotPosForLocalTracking[1] = y;
	m_nRobotPosForLocalTracking[2] = t;
	m_nRobotPosForLocalTracking[3] = dSize;
}
/**
 @brief Korean: 로봇 주변 반경으로 샘플들을 뿌려준다.
 @brief English: reset samples near Robot region.
*/
void LaserBasedParticleFilter::resetSamplesNearRobot(double x, double y, double t, double dSize)
{
	////TRACE("MCL: before reset samples\n");
	int nCnt = 0;
	int nSampleRegion[4];

	nSampleRegion[0] = (int)(x-(dSize*1000));
	nSampleRegion[1] = (int)(x+(dSize*1000));
	nSampleRegion[2] = (int)(y-(dSize*1000));
	nSampleRegion[3] = (int)(y+(dSize*1000));

	m_bResetSamples = false;
	m_nLocalizationState = 10;
	m_dSampleSD = m_nMinSampleStandardDeviation;

	// MCL 결과 저장 변수 초기화.
	m_nMCLReturnValue = MCL_CONVERGING_YET;

	// 샘플이 뿌려질 영역 및 SAMPLE_DENSITY값에 따른 샘플 갯수 계산.
	//m_nSampleNum = (int)((double)(nSampleRegion[3]-nSampleRegion[2])/3.0/1000.0*(double)(nSampleRegion[1]-nSampleRegion[0])/3.0/1000.0*nSampleDensity);
	m_nSampleNum = m_nMinSampleNum;

	// 최대값 이상, 최소값 이하의 샘플 갯수가 계산되면 최대값, 최소값으로 대치.
// 	if (m_nSampleNum > m_nMaxSampleNum) m_nSampleNum = m_nMaxSampleNum;
// 	if (m_nSampleNum < m_nMinSampleNum) m_nSampleNum = m_nMinSampleNum;

	// 정규화된 샘플의 확률값.
	m_dBasePro = 1.0/(double)m_nSampleNum;

	//TRACE("%d, %d, %d, %d\n", nSampleRegion[0], nSampleRegion[1], nSampleRegion[2], nSampleRegion[3]);
	while(nCnt<m_nSampleNum)
	{
		// 샘플이 뿌려지는 영역을 고려하여 임의의 x, y, theta 값 입력.
		m_stSample[nCnt].x = (double)nSampleRegion[0] + (float)rand()/(float)RAND_MAX*(double)(nSampleRegion[1]-nSampleRegion[0]);
		m_stSample[nCnt].y = (double)nSampleRegion[2] + (float)rand()/(float)RAND_MAX*(double)(nSampleRegion[3]-nSampleRegion[2]);
		m_stSample[nCnt].t = t;// + 2.0 * (0.5 - (float)rand()/(float)RAND_MAX) * 10.0/180.0 * M_PI;

		// 지도 범위 바깥에 추출된 샘플은 무시.
		if ((int)(0.5+m_stSample[nCnt].x/100.)<0 || (int)(0.5+m_stSample[nCnt].x/100.) >= m_nMapSizeX
			|| (int)(0.5+m_stSample[nCnt].y/100.)<0 || (int)(0.5+m_stSample[nCnt].y/100.) >= m_nMapSizeY)
			continue;

		// 빈 공간이 아닌 곳에 추출된 샘플은 무시.
		if ( (m_nGridMap[(int)(0.5+m_stSample[nCnt].x/100.0)][(int)(0.5+m_stSample[nCnt].y/100.0)] == UNKNOWN)
			|| (m_nGridMap[(int)(0.5+m_stSample[nCnt].x/100.0)][(int)(0.5+m_stSample[nCnt].y/100.0)] == WALL) )
		continue;

		// 확률값 초기화.
		m_stSample[nCnt].dPro = m_dBasePro;
		nCnt++;
	}
	////TRACE("MCL: after reset samples\n");
	// 그림그리기 위해 샘플정보 한 번 넘겨주기
	m_nOldSampleNum = m_nSampleNum;
	m_vecOldSample.clear();
	Sample stSample;
	for (int i=0; i<m_nSampleNum; i++) {
	  	if (m_stSample[i].dPro==0.0) continue;
		stSample.x = m_stSample[i].x;
		stSample.y = m_stSample[i].y;
		stSample.t = m_stSample[i].t;

		stSample.dPro = m_stSample[i].dPro;
		stSample.dRangeUpdatePro = m_stSample[i].dRangeUpdatePro;
		m_vecOldSample.push_back(stSample);
	}
	m_vecOldSampleCopy = m_vecOldSample;
}


/**
 @brief Korean:  샘플의 위치 분산을 이용하여 다음 단계에서 추출할 샘플의 갯수를 계산.
 @brief English: calculate number of samples which will be extracted based on standard deviation of sample positions.
*/
void LaserBasedParticleFilter::CalculateNoOfSamples(bool bCalSampleCnt)
{
	double dAvgPose[3], dSD[3];		// 샘플의 평균 및 표준편차를 구하기 위한 변수.

	if (bCalSampleCnt) {
		// 확률을 고려하여 분산과 다음 단계 샘플의 갯수를 계산함.
		dAvgPose[0] = 0.0;
		dAvgPose[1] = 0.0;
		dAvgPose[2] = 0.0;
		dSD[0] = 0.0;
		dSD[1] = 0.0;
		dSD[2] = 0.0;

		for(int i = 0; i<m_nSampleNum; i++) {
			dAvgPose[0] += m_stSample[i].x*m_stSample[i].dPro;
			dAvgPose[1] += m_stSample[i].y*m_stSample[i].dPro;
			dAvgPose[2] += m_stSample[i].t*m_stSample[i].dPro;
		}
		for(int i = 0; i<m_nSampleNum; i++) {
			dSD[0] += (m_stSample[i].x-dAvgPose[0])*(m_stSample[i].x-dAvgPose[0])*m_stSample[i].dPro;
			dSD[1] += (m_stSample[i].y-dAvgPose[1])*(m_stSample[i].y-dAvgPose[1])*m_stSample[i].dPro;
			dSD[2] += (m_stSample[i].t-dAvgPose[2])*(m_stSample[i].t-dAvgPose[2])*m_stSample[i].dPro;
		}
		m_dSampleSD = sqrt(dSD[0] + dSD[1]);
		m_dSampleSDForRotation = sqrt(dSD[2]);

		dSD[0] = sqrt(dSD[0]);
		dSD[1] = sqrt(dSD[1]);

		// 샘플의 표준편차를 이용하여 다음 단계에 추출될 샘플의 갯수를 계산.
		m_nSampleNum = (int)( 1.0*(dSD[0]*dSD[1]/1000.0/1000.0)*m_nSampleDensity );

		// 최대값 이상, 최소값 이하의 샘플 갯수가 계산되면 최대값, 최소값으로 대치.
		if (m_nSampleNum > m_nMaxSampleNum) m_nSampleNum = m_nMaxSampleNum;
		if (m_nSampleNum < m_nMinSampleNum) m_nSampleNum = m_nMinSampleNum;
	}
	else {
		// 확률을 고려하지 않고 샘플 각각의 위치만 고려하여 분산을 구함.
		dAvgPose[0] = 0.0;
		dAvgPose[1] = 0.0;
		dAvgPose[2] = 0.0;
		dSD[0] = 0.0;
		dSD[1] = 0.0;
		dSD[2] = 0.0;

		for(int i = 0; i<m_nSampleNum; i++) {
			dAvgPose[0] += m_stSample[i].x;
			dAvgPose[1] += m_stSample[i].y;
		}
		dAvgPose[0] /= (double)m_nSampleNum;
		dAvgPose[1] /= (double)m_nSampleNum;
		for(int i = 0; i<m_nSampleNum; i++) {
			dSD[0] += (m_stSample[i].x-dAvgPose[0])*(m_stSample[i].x-dAvgPose[0]);
			dSD[1] += (m_stSample[i].y-dAvgPose[1])*(m_stSample[i].y-dAvgPose[1]);
		}
		m_dSampleSD = sqrt( (dSD[0]+dSD[1])/(double)m_nSampleNum);
	}
}


/**
 @brief Korean:  샘플들의 확률을 평균화한다.
 @brief English: normalize probabilities of samples
*/
void LaserBasedParticleFilter::Normalizing()
{
	double dSum = 0.0;
    int nCnt = 0;
    double dMax=0.0;

	for(int i=0; i<m_nSampleNum; i++)
		dSum += m_stSample[i].dPro;
/*
	if (dSum==0.0) {
		for(int i=0; i<m_nSampleNum; i++)
			m_stSample[i].dPro = 1.0/(double)m_nSampleNum;
	} else {
		for(int i=0; i<m_nSampleNum; i++)
			m_stSample[i].dPro = m_stSample[i].dPro / dSum;
	}
*/
	if(dSum !=0.0){
		for(int i=0; i<m_nSampleNum; i++)
			m_stSample[i].dPro = m_stSample[i].dPro / dSum;
	}
}

/**
 @brief Korean:  거리센서의 데이터를 받아온다.
 @brief English: set range values of range sensor
*/
void LaserBasedParticleFilter::setRangeData(int nRangeData[Sensor::URG04LX_DATA_NUM181])
{
	//memcpy(m_dRangeData, nRangeData, sizeof(m_dRangeData));
	for(int i=0; i<181; i++){
		m_dRangeData[i] = (double)(nRangeData[i]);
	}
	m_bDoRangeUpdate = true;

}


/**
 @brief Korean:  엔코더의 데이터를 받아온다.
 @brief English: set encoder values
*/
void LaserBasedParticleFilter::setEncoderData(double dEncoderDelta[3])
{
	memcpy(m_dEncoderData, dEncoderDelta, sizeof(m_dEncoderData));

	m_dAccumulatedDistance += sqrt(m_dEncoderData[0]*m_dEncoderData[0]+m_dEncoderData[1]*m_dEncoderData[1]);
	m_dAccumulatedRotation += fabs(m_dEncoderData[2]);


}

/**
 @brief Korean: 거리센서의 파라미터 정보를 받아온다
 @brief English: set parameters of range sensor
*/
void LaserBasedParticleFilter::setRangeSensorParameter(int nNo, int nMinAngle, double dInterval, double dMaxDistance,double dRangeSensorOffset)
{

	m_nNoOfRangeSensorPoint = nNo;
	m_dRangeSensorAngleInterval = dInterval;
	m_nRangeSensorMinAngle = nMinAngle;
	m_dMaxDistance = dMaxDistance;
	m_dRangeSensorOffset = dRangeSensorOffset;
	initProbability();

}

/**
 @brief Korean: 모션에 대한 불확실성 정보를 받아온다
 @brief English: set uncertainty of wheel motion
 of range sensor
*/
void LaserBasedParticleFilter::setDeviation(double dDevationforTrans, double dDevationforRotae,double dDevationforTransRotae )
{
	m_dDeviationforTransConverced = 0.20;
	m_dDeviationforRotateConverced = 0.10;
	m_dDeviationforTransRotateConverced= M_PI/180.0*dDevationforTransRotae;

}


/**
 @brief Korean: 샘플의 최대 최소 개수를 받아온다
 @brief English: set  max and min sample nubers
 of range sensor
*/
void LaserBasedParticleFilter::setSampleNum(int nMaxSample,int nMinSample)
{
	m_nMaxSampleNum =nMaxSample;		
	m_nMinSampleNum =nMinSample;			
	m_nSampleNum = m_nMaxSampleNum;

	if(m_stSample!=NULL)	delete [] m_stSample;
	m_stSample = new Sample[m_nMaxSampleNum];

}

/**
 @brief Korean: 2*sigma ~ 2*sigma 범위 내에서 임의의 값을 리턴하는 함수.
 @brief English: return random value from -2*sigma to 2*sigma
 of range sensor
*/
double LaserBasedParticleFilter::GetRandValue(double sigma)
{
	return sigma*2.0*( (double)rand()/(double)RAND_MAX - 0.5);
}

/**
 @brief Korean: 평균 0, sigma인 가우시안 분포를 따르는 값 출력.
 @brief English: return value according to Gausian distribution with sigma and x
 of range sensor
*/

double LaserBasedParticleFilter::GetGaussianValue(double sigma, double x)
{
        if (sigma==0) return 0;

		//return 0.3989/sigma*exp(-x*x/sigma/sigma);
		return exp(-x*x/2.0/sigma/sigma);
}

/**
 @brief Korean: 샘플들을 가져간다
 @brief English: get Sample
 of range sensor
*/
vector<Sample> LaserBasedParticleFilter::getParticle()
{
	vector<Sample> vecDummy;
	Sample stSample;
	vecDummy.clear();
	
	for (int i=0; i<m_nSampleNum; i++) {
		stSample.x = m_stSample[i].x;
		stSample.y = m_stSample[i].y;
		stSample.t = m_stSample[i].t;

		stSample.dPro = m_stSample[i].dPro;
		stSample.dRangeUpdatePro = m_stSample[i].dRangeUpdatePro;
		vecDummy.push_back(stSample);
	}

    return vecDummy;
}

/**
 @brief Korean: 
 @brief English: 
 of range sensor
*/
vector<Sample> LaserBasedParticleFilter::GetOldParticle()
{
	return m_vecOldSampleCopy;
}

/**
 @brief Korean: 샘플이 뿌려질 영역을 설정한다
 @brief English: set sample region
 of range sensor
*/
void LaserBasedParticleFilter::SetSampleRegion(int minX, int maxX, int minY, int maxY)
{
	if (minX>100)
		m_nSampleRegion[0] = minX;
	else m_nSampleRegion[0] = 100;

	if (maxX<m_nMapSizeX*100-100)
		m_nSampleRegion[1] = maxX;
	else m_nSampleRegion[1] = m_nMapSizeX*100-100;

	if (minY>100)
		m_nSampleRegion[2] = minY;
	else m_nSampleRegion[2] = 100;

	if (maxY<m_nMapSizeY*100-100)
		m_nSampleRegion[3] = maxY;
	else m_nSampleRegion[3] = m_nMapSizeY*100-100;

}

/**
 @brief Korean: 샘플들의  위치  업데이트를 위해 일정이상 로봇이 움직였는지 확인한다
 @brief English: 
 of range sensor
*/
bool LaserBasedParticleFilter::CheckMovementForUpdateOrNot(int nMode)
{
	// for global localization: nMode = 0
	// for local tracking: nMode = 1
	////TRACE("%.3f\n", m_dAccumulatedDistance);
	if (nMode == 0) {
		if (m_dAccumulatedDistance > 300.0 || m_dAccumulatedRotation > 10.0*M_PI/180.0) {
			m_dAccumulatedDistance = 0.0;
			m_dAccumulatedRotation = 0.0;
			return true;
		}
		return false;
	}

	if (nMode == 1) {
		if (m_dAccumulatedDistance > 100.0 || m_dAccumulatedRotation > 3.0*M_PI/180.0) {
			m_dAccumulatedDistance = 0.0;
			m_dAccumulatedRotation = 0.0;
			return true;
		}
		return false;
	}

	return false;
}

/**
 @brief Korean: 지도 상의 (x,y,t)위치에서 예상되는 거리센서 값을 구함.
 @brief English:  predict range values on (x,y,t) based on map data
 of range sensor
*/
void LaserBasedParticleFilter::predictRangeData(double x, double y, double t, int nGap)
{
	int nPosX1=0,nPosY1=0;
	int nPosX2=0,nPosY2=0;
	int nHitX=0, nHitY=0;

	double dAngle=0.;
	double dPosTh=0.;

	nPosX1 = (int)((x + m_dRangeSensorOffset * cos(t)) / 100.0);
	nPosY1 = (int)((y + m_dRangeSensorOffset * sin(t))/ 100.0);
	dPosTh = t*R2D + (float)+m_nRangeSensorMinAngle;
	dPosTh = dPosTh*D2R;

	for (int i=0; i<m_nNoOfRangeSensorPoint; i=i+nGap, dAngle+=m_dRangeSensorAngleInterval){
	//for (int i=0; i<m_nNoOfRangeSensorPoint; i=i+nGap, dAngle+=m_dRangeSensorAngleInterval){

		nPosX2 = nPosX1 +(int)(m_dMaxDistance*cos(dPosTh)/100.);
		nPosY2 = nPosY1 + (int)(m_dMaxDistance*sin(dPosTh)/100.);
		RayTracer(nPosX1, nPosX2, nPosY1, nPosY2, &nHitX, &nHitY);

		// 지도의 경계 근처에 있는 경우를 따지는 부분
		if (nHitX<=10 || nHitX >= (m_nMapSizeX-10) || nHitY<=10 || nHitY >=(m_nMapSizeY-10)) {
			m_dEstimatedRangeData[i]=0.;
		}
		else{
			m_dEstimatedRangeData[i] = sqrt((double)(nHitX*100-nPosX1*100)*(nHitX*100-nPosX1*100)+
											(double)(nHitY*100-nPosY1*100)*(nHitY*100-nPosY1*100)
										  );
		}

		dPosTh= dPosTh + (1*D2R);

	}

}

/**
 @brief Korean: 
 @brief English: 
 of range sensor
*/
void LaserBasedParticleFilter::RayTracer(int x_1, int x_2, int y_1, int y_2, int *hit_x, int *hit_y)
{

	int eps = 0;
	int x_thres=x_2, y_thres=y_2;

	if(x_1 < 0) x_1 = 0;
	else if(x_1 >= m_nMapSizeX) x_1 = m_nMapSizeX-1;
	if(y_1 < 0) y_1 = 0;
	else if(y_1 >= m_nMapSizeY) y_1 = m_nMapSizeY-1;

	if(x_thres < 0) x_thres = 0;
	else if(x_thres >= m_nMapSizeX) x_thres = m_nMapSizeX-1;
	if(y_thres < 0) y_thres = 0;
	else if(y_thres >= m_nMapSizeY) y_thres = m_nMapSizeY-1;

		int x = x_1,y = y_1;

		int delx = x_thres - x_1;
		int dely = y_thres - y_1;


	if(delx > 0) {
		if(dely >=0) {
			if(delx > dely) {
				for(int x=x_1; x<=x_thres; x++) {
  					if(m_nGridMap[x][y] == 2 || m_nGridMap[x][y] == 1) {
    					// 거리값을 넣는다.
						*hit_x = x;
					    	*hit_y = y;
    						return;
  					}

  					eps += dely;
  					if((eps<<1) >= delx) {
  						y++;
    					eps -= delx;
  					}
				}

			}
			else { //delx <= dely인 경우

				for(y=y_1; y<=y_thres; y++) {
					if(m_nGridMap[x][y] == 2 || m_nGridMap[x][y] == 1) {
    					// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
  					}

  					eps += delx;
  					if((eps<<1) >= dely) {
    						x++;
    						eps -= dely;
  					}
				}

			}

		}
		else { // dely < 0인경유
			if(delx > -dely) {
				for(x=x_1; x<=x_thres; x++) {
  					if(m_nGridMap[x][y] == 2 || m_nGridMap[x][y] == 1) {
    					// 거리값을 넣는다.
					       *hit_x = x;
					       *hit_y = y;
    					       return;
  					}

					eps += dely;
					if((eps<<1) <= -delx) {
						y--;
						eps += delx;
  					}
				}

			}
			else { //delx <= -dely인 경우

				for(y=y_1; y>=y_thres; y--) {
  					if(m_nGridMap[x][y] == 2 || m_nGridMap[x][y] == 1) {
    					// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

  					eps += delx;
  					if((eps<<1) >= -dely) {
    						x++;
    						eps += dely;
  					}
				}

			}
		}
	}

	else { //delx <=0인경우
		if(dely >= 0) {
			if(-delx > dely) {

				for(x=x_1; x>=x_thres; x--) {
					if(m_nGridMap[x][y] == 2 || m_nGridMap[x][y] == 1) {
					// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += dely;
					if((eps<<1) >= -delx) {
						y++;
						eps += delx;
					}
				}

			}
			else { //-delx <= dely인경우

				for(y=y_1; y<=y_thres; y++) {
					if(m_nGridMap[x][y] == 2 || m_nGridMap[x][y] == 1) {
					// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += delx;
					if((eps<<1) <= -dely) {
						x--;
						eps += dely;
					}
				}

			}
		}
		else { //dely < 0인경우
			if(-delx > -dely) {
				for(x=x_1; x>=x_thres; x--) {
					if(m_nGridMap[x][y] == 2 || m_nGridMap[x][y] == 1) {
						// 거리값을 넣는다.
							*hit_x = x;
							*hit_y = y;
							return;
					}

					eps -= dely;
					if((eps<<1) > -delx) {
						y--;
						eps += delx;
					}
				}

			}
			else { //-delx <= -dely인경우
				for(y=y_1; y>=y_thres; y--) {
					if(m_nGridMap[x][y] == 2 || m_nGridMap[x][y] == 1) {
					// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps -= delx;
					if((eps<<1) >= -dely) {
						x--;
						eps += dely;
					}
				}

			}
		}
	}

}


