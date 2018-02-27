#ifndef __RPRIDAR_DATA_H__
#define __RPRIDAR_DATA_H__

class RPLidarData {

private:
	float* angle;
	float* distance;

public:
	RPLidarData() {angle = new float[360]; distance = new float[360];};
	~RPLidarData() {delete[] angle; delete[] distance;};

	float* getAngle() {return angle;};
	float* getDistance() {return distance;};

};




#endif /* __RPRIDAR_DATA_H__ */