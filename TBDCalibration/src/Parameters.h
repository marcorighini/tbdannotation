/*
 * Parameters.h
 *
 *  Created on: 08/jul/2013
 *      Author: alessandro
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "opencv2/core/core.hpp"

/*
 * Timer macros
 * ############################################################
 */
#define START_TIMER(n)	double main_timer_n = (double) getTickCount(); \
						double previous_timer_n = main_timer_n;

#define ELAPSED_TIME(n, what)	std::cout << what << ":\t" << ((double) getTickCount() - previous_timer_n) / getTickFrequency() << \
											"\t(" << ((double) getTickCount() - main_timer_n) / getTickFrequency() << ")\n"; \
								previous_timer_n = getTickCount();

#define END_TIME(n, what)	std::cout << what << ":\t" << ((double) getTickCount() - previous_timer_n) / getTickFrequency() << \
											"\t\t(" << ((double) getTickCount() - main_timer_n) / getTickFrequency() << ")\n\n";

namespace Parameters {

/*
 * Parameters
 * ############################################################
 */
// Bottom Hat
const int MORPH_SE_SHAPE = 0;
const int MORPH_SE_SIZE = 5;

// Adaptive threshold
const int ADAPTIVE_THRESHOLD_BLOCK_SIZE = 41;
const int ADAPTIVE_THRESHOLD_CONSTANT = 8;
const int THRESHOLD_MAX_VALUE = 255;

// Adjust Web Cam Image Orientation
static const int WEB_CAM_FROM_RIGHT = 0;
static const int WEB_CAM_FROM_LEFT = 1;
static const int SELECTED_WEB_CAM_ORIENTATION = WEB_CAM_FROM_LEFT;

// Preprocessing parameters
const bool CHANNEL_PREPROCESSING = true;
const cv::Size GAUSSIAN_BLUR_SIZE(3, 3);
const int MEDIAN_BLUR_SIZE = 5;
const int BOTTOM_HAT_MORPH_SE_SIZE = 7;

// Search types
const int BRUTE_FORCE_SEARCH = 0;
const int GREEDY_SEARCH = 1;
const int SELECTED_SEARCH = GREEDY_SEARCH;

// Other parameters
const int MAX_SUCCESSIVE_EQUAL_VALUE_ACCEPTED_MOVE = 30;
const int BOUNDARY_MARGIN = 15;
const int THRESH_DISTANCE = 1;
const int MAX_COMBINATION = 3;
const double ASPIRATION_CRITERIA_FACTOR = 11. / 10;
const std::string ANNOTATIONS_THRESHOLD_FILENAME = "thresholds.yml";
const int ESC_KEY = 27;

// Enum
enum CHANNELS_ID {
	BGR_B, BGR_G, BGR_R, HSV_H, HSV_S, HSV_V, LAB_L, LAB_A, LAB_B, YCRCB_Y, YCRCB_CR, YCRCB_CB
};

enum DIRECTIONS {
	LOWER_SX_DIRECTION, LOWER_DX_DIRECTION, UPPER_SX_DIRECTION, UPPER_DX_DIRECTION
};

// Windows displacement
const int WINDOW_FIRST_COL = 20;
const int WINDOW_SECOND_COL = 520;
const int WINDOW_THIRD_COL = 1020;
const int WINDOW_FIRST_ROW = 30;
const int WINDOW_SECOND_ROW = 720;

}

#endif /* PARAMETERS_H_ */
