/*
 * Utils.hpp
 *
 *  Created on: 08/jul/2013
 *      Author: alessandro
 */

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include "Parameters.h"

using namespace cv;
using namespace std;

/*
 * Class Utils: a collection of util functions
 */
class Utils {
public:
	static constexpr double BETA = 1.;

	/*
	 * Computes morphological operator Bottom Hat
	 */
	static void bottomHat(const Mat& src, Mat& dst, const int radius) {
		Mat element = getStructuringElement(Parameters::MORPH_SE_SHAPE, Size(2 * radius + 1, 2 * radius + 1),
				Point(radius, radius));

		morphologyEx(src, dst, MORPH_BLACKHAT, element);
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

	/*
	 * Threshold structure
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

	/*
	 * Gets thresholded image
	 */
	static void getThresholdedImage(Mat& thresholded, const Mat* channelsVector,
			const vector<ThresholdStructure> tsVector) {
		thresholded = Mat(channelsVector[0].size(), CV_8UC1, Scalar(255));

		// For each threshold
		for (uint i = 0; i < tsVector.size(); i++) {
			ThresholdStructure ts = tsVector[i];

			Mat currentThresholded;
			inRange(channelsVector[ts.channel], Scalar(ts.thresh.first), Scalar(ts.thresh.second), currentThresholded);

			// Merge result
			bitwise_and(thresholded, currentThresholded, thresholded);
		}
	}

	/*
	 * Gets error image from thresholded image
	 */
	static void getErrorImage(Mat& result, const Mat& truth, const Mat& thresholded) {
		bitwise_xor(truth, thresholded, result);
	}

	/*
	 * F-measure: a common way to join two indexes values
	 */
	static double getFmeasure(double index1, double index2) {
		double score;

		if (index1 != 0 && index2 != 0) {
			score = (1 + BETA * BETA) * (index1 * index2) / (BETA * BETA * index1 + index2);
		} else {
			score = 0;
		}

		return score;
	}

	/*
	 * Gets precision and recall values
	 */
	static pair<double, double> getPrecisionRecall(const Mat& truth, const Mat mask) {
		// Computes needed statistics
		Mat truePositiveDefectMask = mask & truth;
		float truePositives = countNonZero(truePositiveDefectMask);

		// Maybe this mask is useless in this context
		//		Mat trueNegativeDefectMask = ~mask & ~truth;
		//		float trueNegatives = countNonZero(trueNegativeDefectMask);

		Mat falsePositiveDefectMask = mask & (~truth);
		float falsePositives = countNonZero(falsePositiveDefectMask);

		Mat falseNegativeDefectMask = (~mask) & truth;
		float falseNegatives = countNonZero(falseNegativeDefectMask);

		double precision = truePositives / (truePositives + falsePositives);
		double recall = truePositives / (truePositives + falseNegatives);

		return pair<double, double> (precision, recall);
	}

	/*
	 * Objective Function (for a combination of threshold channels)
	 */
	static double objFunction(const Mat& truth, const Mat* channelsVector,
			const vector<ThresholdStructure> tsVector) {
		Mat thresholded;
		getThresholdedImage(thresholded, channelsVector, tsVector);

		pair<double, double> precisionRecall = getPrecisionRecall(truth, thresholded);

		double fMeasure = getFmeasure(precisionRecall.first, precisionRecall.second);

		return fMeasure;
	}

	/*
	 * Objective Function (for a single thresh on a channel)
	 */
	static double objFunction(const Mat& truth, const Mat* channelsVector, ThresholdStructure ts) {
		vector<ThresholdStructure> tsVector;
		tsVector.push_back(ts);

		return objFunction(truth, channelsVector, tsVector);
	}

	/*
	 * Brute Force Search
	 * NOTE: don't use this type of search!
	 */
	static vector<ThresholdStructure> bruteForceSearch(const Mat& truth, const Mat* channelsVector,
			const vector<int>& channelToOptimize, const vector<string>& channelLabels) {
		vector<ThresholdStructure> tsVector;

		// For each channel to optimize
		for (unsigned int ch = 0; ch < channelToOptimize.size(); ch++) {
			std::cout << "Optimizing channel " << channelLabels[channelToOptimize[ch]] << "\n";
			ThresholdStructure best_ts(channelToOptimize[ch]);
			best_ts.value = numeric_limits<double>::min();

			int steps = 0;
			for (int lower = 0; lower <= 256 - Parameters::THRESH_DISTANCE; lower++) {
				for (int upper = lower + Parameters::THRESH_DISTANCE; upper <= 256; upper++) {
					ThresholdStructure ts(channelToOptimize[ch]);
					ts.thresh = pair<int, int> (lower, upper);
					ts.value = objFunction(truth, channelsVector, ts);

					if (ts.value > best_ts.value) {
						best_ts = ts;
					}

					if (steps % 1000 == 0) {
						std::cout << "Step n° " << steps << "\tF-Measure " << best_ts.value << "\n";
					}

					steps++;
				}
			}

			tsVector.push_back(best_ts);
		}

		return tsVector;
	}

	/*
	 * Greedy Search, based on truth image information and on a local search
	 */
	static vector<ThresholdStructure> greedySearch(const Mat& truth, const Mat* channelsVector,
			const vector<int>& channelToOptimize, const vector<string>& channelLabels) {
		vector<ThresholdStructure> tsVector;

		std::cout << "\n-------------------------------------------------------------------\n" << " GREEDY SEARCH"
				<< "\n-------------------------------------------------------------------\n";

		std::cout << "Channel preprocessing:\t\t\t\t" << (Parameters::CHANNEL_PREPROCESSING ? "yes" : "no") << "\n";
		std::cout << "Max successive equal value move to accept:\t"
				<< Parameters::MAX_SUCCESSIVE_EQUAL_VALUE_ACCEPTED_MOVE << "\n";
		std::cout << "Max combinations of channels:\t\t\t" << Parameters::MAX_COMBINATION << " channels\n";

		std::cout << "\nSearching...\n-------------------------------------------------\n";

		// For each channel to optimize
		for (unsigned int ch = 0; ch < channelToOptimize.size(); ch++) {
			std::cout << "Optimizing channel " << channelLabels[channelToOptimize[ch]] << "\n";

			// GOAL: select true pixels
			Mat greedy;
			bitwise_and(truth, channelsVector[channelToOptimize[ch]], greedy);

			vector<uchar> pixels;
			for (int i = 0; i < greedy.rows; i++) {
				for (int j = 0; j < greedy.cols; j++) {
					if (truth.at<uchar> (i, j) == 255) {
						pixels.push_back(channelsVector[channelToOptimize[ch]].at<uchar> (i, j));
					}
				}
			}

			// Closest thresh based on true pixels
			pair<int, int> closest_thresh;
			closest_thresh.first = (int) *(std::min_element(pixels.begin(), pixels.end()));
			closest_thresh.second = (int) *(std::max_element(pixels.begin(), pixels.end()));

			ThresholdStructure ts(channelToOptimize[ch]);
			ts.thresh = closest_thresh;
			ts.value = objFunction(truth, channelsVector, ts);
			ThresholdStructure best_ts = greedySearchCore(truth, channelsVector, ts);

			// Pushing inside vector
			tsVector.push_back(best_ts);

			// Notifing result
			cout << "\tThreshold:\t";
			cout << "[" << best_ts.thresh.first << ", " << best_ts.thresh.second << "]\n";
			cout << "\tF-Measure:\t" << best_ts.value << "\n";
		}

		return tsVector;
	}

	/*
	 * Greedy Search Core: local search in N^4 with 4 directions (increasing or decreasing threshold limits)
	 */
	static ThresholdStructure greedySearchCore(const Mat& truth, const Mat* channelsVector,
			ThresholdStructure init_ts) {
		ThresholdStructure ts = init_ts;

		// Pool of directions
		bool directions[4] = { true, true, true, true };

		// While some direction is available
		int successiveEqualValueAcceptedMoves = 0;
		while (directions[Parameters::LOWER_SX_DIRECTION] || directions[Parameters::LOWER_DX_DIRECTION]
				|| directions[Parameters::UPPER_SX_DIRECTION] || directions[Parameters::UPPER_DX_DIRECTION]) {

			// Searching for invalid moves
			if (directions[Parameters::LOWER_SX_DIRECTION] && ts.thresh.first == 0) {
				directions[Parameters::LOWER_SX_DIRECTION] = false;
			}

			if (directions[Parameters::LOWER_DX_DIRECTION] && ts.thresh.first >= ts.thresh.second - 1) {
				directions[Parameters::LOWER_DX_DIRECTION] = false;
			}

			if (directions[Parameters::UPPER_SX_DIRECTION] && ts.thresh.second <= ts.thresh.first + 1) {
				directions[Parameters::UPPER_SX_DIRECTION] = false;
			}

			if (directions[Parameters::UPPER_DX_DIRECTION] && ts.thresh.second >= 255) {
				directions[Parameters::UPPER_DX_DIRECTION] = false;
			}

			// Move!
			ThresholdStructure best_ts = ts;
			bool finalRangeIncrease = false;
			for (int m = 0; m < 4; m++) {
				ThresholdStructure current_ts = best_ts;

				bool rangeIncrease = false;
				// If the current direction is available
				if (directions[m]) {
					switch (m) {
					case Parameters::LOWER_SX_DIRECTION:
						current_ts.thresh.first--;
						rangeIncrease = true;
						break;
					case Parameters::LOWER_DX_DIRECTION:
						current_ts.thresh.first++;
						rangeIncrease = false;
						break;
					case Parameters::UPPER_SX_DIRECTION:
						current_ts.thresh.second--;
						rangeIncrease = false;
						break;
					case Parameters::UPPER_DX_DIRECTION:
						current_ts.thresh.second++;
						rangeIncrease = true;
						break;
					}

					// Evaluating current move
					current_ts.value = objFunction(truth, channelsVector, current_ts);

					// If the current value is better than accept the solution...
					if (current_ts.value > best_ts.value) {
						best_ts = current_ts;
						// ...else if it is equal accept the solution if it increases the range
					} else if (current_ts.value == best_ts.value) {
						if (rangeIncrease && successiveEqualValueAcceptedMoves
								< Parameters::MAX_SUCCESSIVE_EQUAL_VALUE_ACCEPTED_MOVE) {
							best_ts = current_ts;
							finalRangeIncrease = true;
						} else {
							directions[m] = false;
						}
						// ...else discard the direction
					} else {
						directions[m] = false;
					}
				}
			}

			/*
			 * Update if best value found in previous search is better then the current one or
			 * it increseas the range
			 */
			if (best_ts.value > ts.value || finalRangeIncrease) {
				if (best_ts.value > ts.value) {
					// Every improvement enables all directions
					for (int k = 0; k < 4; k++) {
						directions[k] = true;
					}

					successiveEqualValueAcceptedMoves = 0;
				} else if (finalRangeIncrease) {
					successiveEqualValueAcceptedMoves++;
				}

				ts = best_ts;
			}
		}

		return ts;
	}
};

#endif /* UTILS_HPP_ */
