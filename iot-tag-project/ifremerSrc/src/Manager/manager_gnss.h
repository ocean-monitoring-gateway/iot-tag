/*
 * manager_gnss.h
 *
 *  Created on: 4 f�vr. 2020
 *      Author: jfezande
 */

#ifndef SRC_MANAGER_MANAGER_GNSS_H_
#define SRC_MANAGER_MANAGER_GNSS_H_

#include "GNSS.h"

#define GNSS_ON		true
#define GNSS_OFF 	false

void manager_gnss_init(void);
void manager_gnss_logSattelites(void);

//fonction utilis�e car les fonctions suspend et resume ne fonctionne pas toujours � cause du tx.busy
//donc on d�finit l'�tat souhait� et on continue � v�rifier si l'�tat a bien �t� appliqu�
bool manager_gnss_getState(void);
void manager_gnss_setState(bool state);
void manager_gnss_processState(void);

void manager_gnss_Vdd_En(bool state);
void manager_gnss_Backup_En(bool state);


#endif /* SRC_MANAGER_MANAGER_GNSS_H_ */
