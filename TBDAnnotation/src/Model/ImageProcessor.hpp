/*
 * ImageProcessor.hpp
 *
 *  Created on: 20/apr/2013
 *      Author: alessandro
 */

#ifndef IMAGEPROCESSOR_HPP_
#define IMAGEPROCESSOR_HPP_

#include <vector>
#include <opencv2/core/core.hpp>
#include <iostream>
#include "Parameters.h"
#include "TShighgui.hpp"

using namespace std;
using namespace cv;

/*
 * Class ImageProcessor
 */
class ImageProcessor {
public:
	static void getImageFeaturePoints(const Mat& image, vector<Point>& featurePoints) {
		// Initial resize before processing
		Mat resizedImage;
		resize(image, resizedImage, Parameters::PREPROCESSING_SIZE, 0, 0, CV_INTER_AREA);

		// First binarization
		Mat firstAdaptiveThreshold;
		cv::adaptiveThreshold(resizedImage, firstAdaptiveThreshold, Parameters::THRESHOLD_MAX_VALUE,
				CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, Parameters::ADAPTIVE_THRESHOLD_BLOCK_SIZE,
				Parameters::ADAPTIVE_THRESHOLD_CONSTANT);

		// Estimate character size (square root of the mode of areas of connected components)
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		ImageProcessor::contours(firstAdaptiveThreshold, contours, hierarchy);

		vector<double> areas;
		ImageProcessor::contoursAreas(contours, areas);

		vector<int> notFilteredContoursIndexes;
		int mode = ImageProcessor::computeAreasMode(areas, notFilteredContoursIndexes);
		if (mode % 2 == 0) {
			mode++;
		}

		// Blur
		Mat blurred;
		GaussianBlur(firstAdaptiveThreshold, blurred, Size(mode, mode), 0, 0);

		// Second binarization
		Mat finalOtsu;
		ImageProcessor::otsu(blurred, finalOtsu);

		// Compute centroids
		connectedComponentCentroids(finalOtsu, featurePoints);

#ifdef DEBUG_IMAGE_PROCESSOR_GET_IMAGE_FEATURE_POINTS
		TShighgui::imshow("FIRST ADAPTIVE THRESHOLD", firstAdaptiveThreshold);
		TShighgui::waitKey(0);

		TShighgui::imshow("GAUSSIAN BLUR", blurred);
		TShighgui::waitKey(0);

		TShighgui::imshow("FINAL OTSU", finalOtsu);
		TShighgui::waitKey(0);

		Mat centroidsImage = finalOtsu.clone();
		for (unsigned i = 0; i < featurePoints.size(); i++) {
			circle(centroidsImage, featurePoints.at(i), 5, Scalar(255, 255, 255));
		}
		TShighgui::imshow("CENTROIDS", centroidsImage);
#endif
	}
	;

	/**
	 * Computes centroids for connected components
	 */
	static void connectedComponentCentroids(const Mat& src, vector<Point>& centroids) {
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		ImageProcessor::contours(src, contours, hierarchy);

		for (unsigned int i = 0; i < contours.size(); i++) {
			Moments mu = moments(contours[i]);

			if (mu.m00 > Parameters::CONTOURS_AREA_THRESHOLD) {
				centroids.push_back(Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00));
			}
		}
	}
	;

	/*
	 * Skin detection
	 */
	static bool skinDetection(const Mat& src) {
		Mat blurred;
		blur(src, blurred, Parameters::BLUR_SIZE);
		Mat hsv;
		cvtColor(blurred, hsv, CV_BGR2HSV);
		Mat bw;
		inRange(hsv, Parameters::SKIN_LOW, Parameters::SKIN_HIGH, bw);

		if ((double) countNonZero(bw) / (bw.rows * bw.cols) < Parameters::SKIN_PERCENTAGE_THRESHOLD) {
			return false;
		} else {
			return true;
		}
	}
	;

	/*
	 * Annotation color detection based on loaded thresholds
	 */
	static void annotationColorMask(const Mat&src, Mat& dst) {
		Mat blurred = src.clone();
		blur(src, blurred, Parameters::ANNOTATION_COLOR_GAUSSIAN_BLUR_SIZE);

		// Select thresholds from file (NEW MODE)
		if (Parameters::ANNOTATION_COLOR_MODE == Parameters::ANNOTATION_COLOR_THRESHOLD_FILE_MODE) {
			dst = Mat(src.size(), CV_8UC1, Scalar(255));
			for (uint t = 0; t < Parameters::annotationThresholds.size(); t++) {
				Parameters::ThresholdStructure ts = Parameters::annotationThresholds[t];

				Mat convertedBlurred;
				// Respecting the enum order
				switch (ts.channel / 3) {
				case 0: // BGR
					// It is already a BGR image
					convertedBlurred = blurred.clone();
					break;
				case 1: // HSV
					cvtColor(blurred, convertedBlurred, CV_BGR2HSV);
					break;
				case 2: // Lab
					cvtColor(blurred, convertedBlurred, CV_BGR2Lab);
					break;
				case 3: // yCrCb
					cvtColor(blurred, convertedBlurred, CV_BGR2YCrCb);
					break;
				default:
					;
				}

				// Splitting
				vector<Mat> channels(3);
				split(convertedBlurred, channels);

				// Selecting the right channel
				Mat channel = channels[ts.channel % 3];

				Mat channelPreprocessed;
				if (Parameters::CHANNEL_PREPROCESSING) {
					// Bottom hat of channel
					bottomHat(channel, channelPreprocessed, Parameters::ANNOTATION_COLOR_MORPH_SE_SIZE);
					medianBlur(channelPreprocessed, channelPreprocessed, Parameters::ANNOTATION_COLOR_MEDIAN_BLUR_SIZE);
				} else {
					channelPreprocessed = channel.clone();
				}

				Mat currentThresholded;
				inRange(channelPreprocessed, Scalar(ts.thresh.first), Scalar(ts.thresh.second), currentThresholded);

				bitwise_and(dst, currentThresholded, dst);
			}

		// Fixed threshold (OLD MODE)
		} else if (Parameters::ANNOTATION_COLOR_MODE == Parameters::ANNOTATION_COLOR_FIXED_MODE) {
			// Get Cb component
			Mat yCbCr;
			cvtColor(blurred, yCbCr, CV_BGR2YCrCb);
			vector<Mat> channelsYCbCr(3);
			split(yCbCr, channelsYCbCr);
			Mat Cb = channelsYCbCr.at(1);

			// Bottom hat of Cb
			Mat CbBottomHat;
			bottomHat(Cb, CbBottomHat, Parameters::ANNOTATION_COLOR_MORPH_SE_SIZE);
			medianBlur(CbBottomHat, CbBottomHat, Parameters::ANNOTATION_COLOR_MEDIAN_BLUR_SIZE);

			dst = Mat::zeros(blurred.rows, blurred.cols, CV_8UC1);
			for (int i = 0; i < dst.rows; i++) {
				for (int j = 0; j < dst.cols; j++) {
					if (Cb.at<uchar> (i, j) < Parameters::CB_THRESHOLD && CbBottomHat.at<uchar> (i, j)
							> Parameters::CB_BOTTOM_HAT_THRESHOLD) {
						dst.at<uchar> (i, j) = 255;
					}
				}
			}
		}
	}

	/*
	 * Return boolean for intersection between black and white (0,255) mask and a contour
	 */
	static bool countourIntersects(const Mat& mask, const vector<Point>& contour) {
		unsigned int i = 0;
		bool intersect = false;
		while (i < contour.size() && intersect == false) {
			Point p = contour.at(i);
			if (mask.at<uchar> (p.y, p.x) == 255) {
				intersect = true;
			}
			i++;
		}
		return intersect;
	}

	/*
	 * Computes morphological operator Bottom Hat
	 */
	static void bottomHat(const Mat& src, Mat& dst, const int radius) {
		Mat element = getStructuringElement(Parameters::MORPH_SE_SHAPE, Size(2 * radius + 1, 2 * radius + 1),
				Point(radius, radius));

		morphologyEx(src, dst, MORPH_BLACKHAT, element);
	}

	/*
	 * Computes threshold with Otsu
	 */
	static void otsu(const Mat& src, Mat& dst) {
		double level = threshold(src, dst, 0, 0, THRESH_OTSU);
		threshold(src, dst, level, Parameters::THRESHOLD_MAX_VALUE, THRESH_BINARY);
	}

	/*
	 * Computes Hough's transformation
	 */
	static void houghLines(const Mat& src, OutputArray lines) {
		HoughLinesP(src, lines, Parameters::HOUGH_RHO, Parameters::HOUGH_THETA, Parameters::HOUGH_THRESHOLD,
				Parameters::HOUGH_MIN_LINE_LENGTH, Parameters::HOUGH_MAX_LINE_GAP);
	}

	/*
	 * Computes contours
	 */
	static void contours(const Mat& src, vector<vector<Point> >& contours, vector<Vec4i>& hierarchy) {
		Mat srcClone = src.clone(); // findContours alters the matrix also if is const
		findContours(srcClone, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point());
	}

	/**
	 * Compute contours areas
	 */
	static void contoursAreas(const vector<vector<Point> >& contours, vector<double> & areas) {
		for (unsigned int i = 0; i < contours.size(); i++) {
			Moments mu = moments(contours[i]);
			areas.push_back(fabs(mu.m00));
		}
	}

	/**
	 * Compute mode of area values, filtering too low values and discretizing
	 */
	static double computeAreasMode(const vector<double>& areas, vector<int> areasNotFilteredIndexes) {
		map<int, int> occurrencesMap;

		for (unsigned i = 0; i < areas.size(); i++) {
			if (areas.at(i) > Parameters::CONTOURS_AREA_THRESHOLD) {
				areasNotFilteredIndexes.push_back(i);
				int area = areas.at(i);

				// Round to nearest semi-dozen
				int semidozens = area / Parameters::CONTOURS_AREA_QUANTUM;
				int notSemiDozen = area % Parameters::CONTOURS_AREA_QUANTUM;
				if (notSemiDozen >= Parameters::CONTOURS_SEMI_DOZEN_THRESHOLD) {
					semidozens++;
				}

				// Occurrence map increment
				map<int, int>::iterator i = occurrencesMap.find(semidozens * Parameters::CONTOURS_AREA_QUANTUM);
				if (i == occurrencesMap.end()) {
					occurrencesMap[semidozens * Parameters::CONTOURS_AREA_QUANTUM] = 1;
				} else {
					i->second++;
				}
			}
		}

		// Get mode
		int mode = 0;
		int maxOccurrences = 0;
		map<int, int>::const_iterator it;
		for (it = occurrencesMap.begin(); it != occurrencesMap.end(); it++) {
			if (it->second > maxOccurrences) {
				maxOccurrences = it->second;
				mode = it->first;
			}
		}

		return sqrt(double(mode));
	}

	/*
	 * Transpose and flip the image for a good visualization:
	 * WEB_CAM_FROM_RIGHT: webcam "looks" from the right side (for left-handed)
	 * WEB_CAM_FROM_LEFT: webcam "looks" from the left side (for right-handed)
	 */
	static void adjustWebCamImage(Mat& src, int webCamOrientation) {
		transpose(src, src);

		switch (webCamOrientation) {
		case Parameters::WEB_CAM_FROM_RIGHT:
			flip(src, src, 0);
			break;
		case Parameters::WEB_CAM_FROM_LEFT:
			flip(src, src, 1);
			break;
		default:
			break;
		}
	}
};

#endif /* IMAGEPROCESSOR_HPP_ */
