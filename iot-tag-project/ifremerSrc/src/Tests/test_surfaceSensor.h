/*
 * test_surfaceSensor.h
 *
 *  Created on: 6 mars 2020
 *      Author: agoharza
 */

#ifndef SRC_TESTS_TEST_SURFACESENSOR_H_
#define SRC_TESTS_TEST_SURFACESENSOR_H_


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



	//last
	tagApp_size

}tagApp_mode_t;


void test_ss_tagApplication_init(void);
void test_ss_tagApplication_process(void);
void test_surfaceSensor(void);



#endif /* SRC_TESTS_TEST_SURFACESENSOR_H_ */
