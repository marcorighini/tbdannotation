/*
 * Debug.h
 *
 *  Created on: 29/apr/2013
 *      Author: marco
 */

#ifndef DEBUG_H_
#define DEBUG_H_

//#define DEBUG_IMAGE_PROCESSOR_GET_IMAGE_FEATURE_POINTS
//#define DEBUG_FEATURES_MANAGER_KNN

//#define DEBUG_FEATURES_MANAGER_DISCRETIZATION_STATISTICS

//#define DEBUG_UTILS_SORT_POINT_CLOCKWISE

//#define DEBUG_REGISTER_FEATURE_PRINT_M_COMBINATION
//#define DEBUG_REGISTER_FEATURE_PRINT_F_COMBINATION
//#define DEBUG_REGISTER_FEATURE_PRINT_FINAL_STATISTICS

//#define DEBUG_RETRIEVE_PAGE_PRINT_M_COMBINATION
//#define DEBUG_RETRIEVE_PAGE_PRINT_CYCLIC_M_COMBINATION
//#define DEBUG_RETRIEVE_PAGE_PRINT_F_COMBINATION
//#define DEBUG_RETRIEVE_PAGE_PRINT_PAGE_MATCH_CONDITIONS
#define DEBUG_RETRIEVE_PAGE_PRINT_FINAL_STATISTICS

//#define DEBUG_TRACKING_COLOR_COMPONENTS
//#define DEBUG_TRACKING_SHOW_ANNOTATIONS

/*
 * Timer macros
 */
#define START_TIMER(n)	double main_timer_n = (double) getTickCount(); \
						double previous_timer_n = main_timer_n;

#define ELAPSED_TIME(n, what)	std::cout << what << ":\t" << ((double) getTickCount() - previous_timer_n) / getTickFrequency() << \
											"\t(" << ((double) getTickCount() - main_timer_n) / getTickFrequency() << ")\n"; \
								previous_timer_n = getTickCount();

#define END_TIME(n, what)	std::cout << what << ":\t" << ((double) getTickCount() - previous_timer_n) / getTickFrequency() << \
											"\t\t(" << ((double) getTickCount() - main_timer_n) / getTickFrequency() << ")\n\n";

/*
 * Semaphore dead-lock check macros
 */
#define WAIT(who)		std::cout << who << "\tW" << std::endl;
#define LOCK(who)		std::cout << who << "\t\tL" << std::endl;
#define RELEASE(who)	std::cout << who << "\t\t\tR" << std::endl;

#endif /* DEBUG_H_ */
