/*
 * LoRaConnection.h
 *
 *  Created on: 23 janv. 2020
 *      Author: jfezande
 */

#ifndef LORAAPPLICATION_LORACONNECTION_H_
#define LORAAPPLICATION_LORACONNECTION_H_

#include "Arduino.h"

typedef struct {



}LoRaConnection_t;


//call LoRaInit
bool LoRaConnection_init(void);
bool LoRaConnection_send(uint8_t port, uint8_t* buff, uint16_t msgSize);

#endif /* LORAAPPLICATION_LORACONNECTION_H_ */
