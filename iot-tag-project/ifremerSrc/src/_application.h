/*
 * _application.h
 *
 *  Created on: 2 avr. 2020
 *      Author: jfezande
 */

#ifndef SRC__APPLICATION_H_
#define SRC__APPLICATION_H_




// --- IoT Tag Application --- //
#include "TurtleTagApplication/tagApplication.h"
#define application_init() 		tagApplication_init()
#define application_process() 	tagApplication_process()

// --- Jellyfish Application --- //
//#include "JellyfishApplication/jellyfishApplication.h"
//#define application_init() 		jellyfishApplication_init()
//#define application_process() 	jellyfishApplication_process()


#endif /* SRC__APPLICATION_H_ */
