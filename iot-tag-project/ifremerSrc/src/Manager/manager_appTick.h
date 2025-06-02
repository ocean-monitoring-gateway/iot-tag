/*
 * manager_apppTick.h
 *
 *  Created on: 10 août 2020
 *      Author: jfezande
 */

#ifndef SRC_MANAGER_MANAGER_APPTICK_H_
#define SRC_MANAGER_MANAGER_APPTICK_H_

#include "Arduino.h"

void manager_appTick_init(void);
void manager_appTick_tick(void);
uint32_t manager_appTick_get(void);


#endif /* SRC_MANAGER_MANAGER_APPTICK_H_ */
