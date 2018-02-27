#include <stdafx.h>
#include "RPLidarProxy.h"
#include <iostream>

using namespace std;


#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


RPLidarProxy::RPLidarProxy() {
	
	opt_com_baudrate = 115200;
	drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
	
	result = 0;
}

RPLidarProxy::~RPLidarProxy() {

}

bool RPLidarProxy::checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

int RPLidarProxy::open(const string& opt_com_path) {
	char* this_opt_com_path = (char*)opt_com_path.c_str();
	int result = 0;
    
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
	cout << "this_opt_com_path2 : " << this_opt_com_path << endl; 

    // make connection...
    if (IS_FAIL(drv->connect(this_opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , this_opt_com_path);
        //goto on_finished;
		result = -1;
	}



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        //goto on_finished;
		result = -1;
    }

    // start scan...
	if(result != -1) {
		u_result re = drv->startScan();
		cout << "scan result : " << re << endl;
	}

	return result;
}

int RPLidarProxy::reset() {
	drv->reset();
	return 0;
}



void RPLidarProxy::readData(RPLidarData& data) {
	
	rplidar_response_measurement_node_t nodes[360*2];
	size_t   count = _countof(nodes);

	op_result = drv->grabScanData(nodes, count);

	//cout << "grabScanData : " << op_result << " , " << count << endl;

	if (IS_OK(op_result)) {
		drv->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count ; ++pos) {
			//printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
				//(nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
				//(nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
				//nodes[pos].distance_q2/4.0f,
				//nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

			data.getAngle()[pos] = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
			data.getDistance()[pos] = nodes[pos].distance_q2/4.0f;
		}
	}
}

int RPLidarProxy::close(){ 
	RPlidarDriver::DisposeDriver(drv);
	return 0;
}