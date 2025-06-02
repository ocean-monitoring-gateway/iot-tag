/*
 * _manager_gnss_config.h
 *
 *  Created on: 17 juil. 2020
 *      Author: jfezande
 */

#ifndef SRC__MANAGER_GNSS_CONFIG_H_
#define SRC__MANAGER_GNSS_CONFIG_H_

//decommenter les algos à activer
#define GNSS_ZEROSATTIMEOUT
#define GNSS_ADAPTATIVEEHPE
#define GNSS_FIXDELAYLOCK



#define GNSS_TIMEOUT_S		(60)
#define GNSS_SNR_LEVEL		(30)


///////////////////////////
// ZEROSAT TIMEOUT ALGO  //
///////////////////////////
#ifdef GNSS_ZEROSATTIMEOUT

#define GNSS_ZEROSATTIMEOUT_S	(8) //second
#define GNSS_ZEROSATTIMEOUT_

#endif


///////////////////////////
// ADAPTATIVE EHPE ALGO  //
///////////////////////////
#ifdef GNSS_ADAPTATIVEEHPE

#define EHPE_LIMIT(t)	(16.6*t)

#else
//fixed EHPE
#define EHPE_LIMIT(t)	(25.0)


#endif


///////////////////////////
// FIX DELAY LOCK        //
///////////////////////////
#ifdef GNSS_FIXDELAYLOCK
#define FIXDELAYLOCK_DURATION_S	(60*60)
#define FIXDELAYLOCK_EHPE		(500)
#endif


#endif /* SRC__MANAGER_GNSS_CONFIG_H_ */
