#ifndef __RPRIDAR_PROXY_H__
#define __RPRIDAR_PROXY_H__

#include "RPLidarData.h"
#include "rplidar.h"
#include <string>

using namespace std;
using namespace rp::standalone::rplidar;

class RPLidarProxy {

private:
	_u32         opt_com_baudrate;
	u_result     op_result;
	RPlidarDriver* drv;
	int result;

public:
	RPLidarProxy();
	~RPLidarProxy();


	bool checkRPLIDARHealth(RPlidarDriver * drv);
	int open(const string& opt_com_path);
	int reset();
	void readData(RPLidarData& data);
	int close();

};




#endif /* __RPRIDAR_PROXY_H__ */