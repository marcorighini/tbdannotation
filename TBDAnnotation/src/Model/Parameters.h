/*
 * Parameters.h
 *
 *  Created on: 22/apr/2013
 *      Author: marco
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "RegisteredPdfMap.hpp"
#include "ConvertedPdfMap.hpp"
#include "Debug.h"
#include "opencv2/core/core.hpp"
#include <sqlite3.h>

/*************** DEBUG ****************/

namespace Parameters {

/*
 * ImageProcessor.hpp constants
 * ############################################################
 */
// Custom preprocessing size
const cv::Size PREPROCESSING_SIZE(960, 1280);

// Bottom Hat
const int MORPH_SE_SHAPE = 0;
const int MORPH_SE_SIZE = 5;

// Adaptive threshold
const int ADAPTIVE_THRESHOLD_BLOCK_SIZE = 41;
const int ADAPTIVE_THRESHOLD_CONSTANT = 8;

// Otsu
const int THRESHOLD_MAX_VALUE = 255;

// GaussianBlur;
const cv::Size BLUR_SIZE(3, 3);

// Hough
const int HOUGH_RHO = 1;
const double HOUGH_THETA = CV_PI / 180;
const int HOUGH_THRESHOLD = 100;
const int HOUGH_MIN_LINE_LENGTH = 50;
const int HOUGH_MAX_LINE_GAP = 3;

// Contours area mode
const int CONTOURS_AREA_THRESHOLD = 50;
const int CONTOURS_AREA_QUANTUM = 5;
const int CONTOURS_SEMI_DOZEN_THRESHOLD = 3;
const int CONTOURS_LINE_THICKNESS = 3;

// Adjust Web Cam Image Orientation
static const int WEB_CAM_FROM_RIGHT = 0;
static const int WEB_CAM_FROM_LEFT = 1;
static const int SELECTED_WEB_CAM_ORIENTATION = WEB_CAM_FROM_LEFT;

// Skin detection
static const cv::Scalar SKIN_LOW = cv::Scalar(0, 30, 80);
static const cv::Scalar SKIN_HIGH = cv::Scalar(20, 150, 255);
static const float SKIN_PERCENTAGE_THRESHOLD = 0.1;

// Annotation color detection
const int ANNOTATION_COLOR_THRESHOLD_FILE_MODE = 0;
const int ANNOTATION_COLOR_FIXED_MODE = 1;
int ANNOTATION_COLOR_MODE = ANNOTATION_COLOR_THRESHOLD_FILE_MODE;
const bool CHANNEL_PREPROCESSING = true;
const int CB_THRESHOLD = 120;
const int CB_BOTTOM_HAT_THRESHOLD = 17;
const cv::Size ANNOTATION_COLOR_GAUSSIAN_BLUR_SIZE(3, 3);
const int ANNOTATION_COLOR_MEDIAN_BLUR_SIZE = 5;
const int ANNOTATION_COLOR_MORPH_SE_SIZE = 7;

/*
 * Features Manager
 * ############################################################
 */

const unsigned int N = 8;
const unsigned int M = 7;
const unsigned int F = 4;
const unsigned int K = 8;

const unsigned int H_SIZE = 128000000;

const double DISCRETIZATION_VECTOR[] = { 0.31, 0.6, 0.95, 1.30, 1.94, 2.93, 5.5 };

#ifdef DEBUG_FEATURES_MANAGER_DISCRETIZATION_STATISTICS
static int DISCRETIZATION_STATISTICS[] = {0, 0, 0, 0, 0, 0, 0, 0};
static vector<double> affineInvariantVector;
static double maxAffineInvariant = 0;
static int invalidAffineInvariants = 0;
#endif

/*
 * Utils.hpp
 * ############################################################
 */
const int PAPER_LINES_LOW = 70;

const double GOOD_LINE_TOLERANCE = 3 * CV_PI / 180;

/*
 * Calibration
 * ############################################################
 */

// Preprocessing parameters
const cv::Size CALIBRATION_GAUSSIAN_BLUR_SIZE(3, 3);
const int CALIBRATION_MEDIAN_BLUR_SIZE = 5;
const int CALIBRATION_BOTTOM_HAT_MORPH_SE_SIZE = 7;

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

/*
 * PageDiscover.hpp
 * ############################################################
 */
const double PAGE_DISCOVERING_THRESHOLD = 0.8;

const double PAGE_DISCOVERING_TIME_TO_RETRIEVE = 5;

const cv::Scalar PAGE_DISCOVERING_LINE_COLOR(255, 0, 0);
const int PAGE_DISCOVERING_LINE_THICKNESS = 3;

/*
 * PageTracking.hpp
 * ############################################################
 */
const int TRACKING_KNN_NORM = cv::NORM_L2;

const bool RESIZE_TRACKING = true;
const double RESIZE_FACTOR = 7. / 10;

const int SIFT_N_FEATURES = 400;
const int SIFT_N_OCTAVE_LAYERS = 10;
const double SIFT_CONTRAST_THRESHOLD = 0.06;
const double SIFT_EDGE_THRESHOLD = 10;
const double SIFT_SIGMA = 1.6;

const double LOWE_THRESHOLD = 0.8;
const double LOWE_FACTOR = 3;
const double MIN_DISTANCE_THRESHOLD = 600;

const float RANSAC_REPROJ_THRESHOLD = 3;

const unsigned int MIN_HOMOGRAPHY_POINTS = 10;

const double HOMOGRAPHY_DETERMINANT_THRESHOLD = 0.6;

const double PAGE_TRACKING_TIME_TO_FAIL = 5;
const double PAGE_TRACKING_TIME_PAGE_ANNOTATION = 3;

const int ANNOTATION_CONTOUR_MIN_AREA = 90;

const int SHARPEN_COLOR_BLUE = 0;
const int SHARPEN_COLOR_RED = 60;
const int SHARPEN_COLOR_GREEN = 20;

const int DIFF_UPPER_Y_THRESHOLD = 20;
const int DIFF_LENGTH_THRESHOLD = 2 * DIFF_UPPER_Y_THRESHOLD;

const int JPEG_COMPRESSION_FACTOR = 80;

/*
 * PageAnnotation.hpp
 * ############################################################
 */
const int ANNOTATION_MOTION_LOWER_THRESHOLD = 2;
const int ANNOTATION_MOTION_UPPER_THRESHOLD = 250;
const int ANNOTATION_TOTAL_MOTION_THRESHOLD = 10;
const int ANNOTATION_LATTICE_DISTANCE = 30;

const cv::Size PAGE_ANNOTATION_BLUR_SIZE(9, 9);
const cv::Size PAGE_ANNOTATION_WIN_SIZE(30, 30);

const cv::Scalar PAGE_ANNOTATION_OPTICAL_FLOW_COLOR(255, 0, 0);
const int PAGE_ANNOTATION_CIRCLE_RADIUS = 3;
const int PAGE_ANNOTATION_CIRCLE_THICKNESS = 1;
const int PAGE_ANNOTATION_LINE_THICKNESS = 2;

const double PAGE_ANNOTATION_TIME_TO_SAVE = 5;

/*
 * FileManager.hpp
 * ############################################################
 */
const string REGISTERED_PDF_MAP_PATH = "registeredPdfMap.dat";
const string DATABASE_PATH = "pageHashTable.db";
const string CONVERTED_PDF_MAP_PATH = "convertedPdfMap.dat";
const string IMAGES_PATH = "images/";
const string RETAINED_IMAGES_PATH = "retained/";
const string MASKS_PATH = "masks/";
const string ANNOTATIONS_PATH = "annotations/";
const string DATASET_PREFIX = "dataset_";
const string ANNOTATIONS_THRESHOLD_FILENAME = "thresholds.yml";

const int CONVERT_DENSITY = 150;
const int CONVERT_QUALITY = 100;

/*
 * WebCamCapture.hpp
 * ############################################################
 */
const bool SHOW_WEBCAM_THREAD_IMAGE = true;

/*
 * Global Variables
 * ############################################################
 */
static RegisteredPdfMap registeredPdfMap;
static ConvertedPdfMap convertedPdfMap;

/*
 * Annotation thresholds
 * ############################################################
 */
struct ThresholdStructure {
	int channel;
	pair<int, int> thresh;
	double value;

	ThresholdStructure() {
	}

	ThresholdStructure(int _channel) {
		channel = _channel;
	}
};

static vector<ThresholdStructure> annotationThresholds;

enum CHANNELS_ID {
	BGR_B, BGR_G, BGR_R, HSV_H, HSV_S, HSV_V, LAB_L, LAB_A, LAB_B, YCRCB_Y, YCRCB_CR, YCRCB_CB
};

enum DIRECTIONS {
	LOWER_SX_DIRECTION, LOWER_DX_DIRECTION, UPPER_SX_DIRECTION, UPPER_DX_DIRECTION
};

/*
 * Others
 * ############################################################
 */
const int ESC_KEY = 27;

// Delimitator between filename and page number adopted by extern convert application
const char CONVERT_DELIMITATOR = '-';

// Rand SEED
const unsigned int FIXED_RAND_SEED = 3521236; // It can be every value not equal to 1!
const bool USE_FIXED_RAND_SEED = true;

// Info size
const cv::Size COMMANDS_IMAGE_SIZE(480, 200);
const cv::Size INFO_IMAGE_SIZE(980, 200);

// Windows displacement
const int WINDOW_FIRST_COL = 20;
const int WINDOW_SECOND_COL = 520;
const int WINDOW_THIRD_COL = 1020;
const int WINDOW_FIRST_ROW = 30;
const int WINDOW_SECOND_ROW = 720;

}

#endif /* PARAMETERS_H_ */
