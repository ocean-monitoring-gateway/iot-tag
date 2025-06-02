/*
 * manager_imu.h
 *
 *  Created on: 2 mars 2020
 *      Author: jfezande
 */

#ifndef SRC_MANAGER_MANAGER_IMU_H_
#define SRC_MANAGER_MANAGER_IMU_H_

typedef enum {
	imuM_moving,
	imuM_stopped


}imuMove_t;

class manager_imu {

public :
	manager_imu(void);
	void init(void);
	imuMove_t getCurrentMove(void);


private :
	uint32_t _lastStopTime;
	void (*_accelero_int1_callback)(void);
	void (*_accelero_int2_callback)(void);
	void (*_mangeto_callback)(void);



	void _handler_accelero1(void);
	void _handler_accelero2(void);
	void _handler_magneto(void);
};



#endif /* SRC_MANAGER_MANAGER_IMU_H_ */
