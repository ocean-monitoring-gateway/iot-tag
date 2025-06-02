/*
 * tagApplication.h
 *
 *  Created on: 22 janv. 2020
 *      Author: jfezande
 */

#ifndef APPLICATION_TAGAPPLICATION_H_
#define APPLICATION_TAGAPPLICATION_H_


// ********************************************************
//		[Surfacing] --------------> [Surface]
//    			  \                   \  /
//				   \				[SubSurface]
//					\	            /
//					 \			   /
//                     [   Dive   ]
//
//***************************************************//

typedef enum {

	tagApp_boot = 0,
	tagApp_surface,
	tagApp_subsurface,
	tagApp_surfacing,
	tagApp_dive,
	tagApp_preDive,



	//last
	tagApp_size

}tagApp_mode_t;


void tagApplication_init(void);
void tagApplication_process(void);




#endif /* APPLICATION_TAGAPPLICATION_H_ */
