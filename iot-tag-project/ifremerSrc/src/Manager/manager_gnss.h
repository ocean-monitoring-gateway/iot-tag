/*
 * manager_gnss.h
 *
 *  Created on: 4 févr. 2020
 *      Author: jfezande
 */

#ifndef SRC_MANAGER_MANAGER_GNSS_H_
#define SRC_MANAGER_MANAGER_GNSS_H_

#include "GNSS.h"

#define GNSS_ON		true
#define GNSS_OFF 	false

void manager_gnss_init(void);
void manager_gnss_logSattelites(void);

//fonction utilisée car les fonctions suspend et resume ne fonctionne pas toujours à cause du tx.busy
//donc on définit l'état souhaité et on continue à vérifier si l'état a bien été appliqué
bool manager_gnss_getState(void);
void manager_gnss_setState(bool state);
void manager_gnss_processState(void);

void manager_gnss_Vdd_En(bool state);
void manager_gnss_Backup_En(bool state);


#endif /* SRC_MANAGER_MANAGER_GNSS_H_ */
