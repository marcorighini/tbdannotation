/*
 * Utils.hpp
 *
 *  Created on: 20/apr/2013
 *      Author: alessandro
 */

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include "Parameters.h"
#include "Triangle.hpp"
#include "TShighgui.hpp"
#include <stdio.h>

using namespace cv;

/*
 * Class Utils: a collection of util functions
 */
class Utils {
public:
	static int getHashIndex(const list<unsigned char>& invariants) {
		int index = 0;
		list<unsigned char>::const_iterator it;
		int i;
		for (it = invariants.begin(), i = 0; it != invariants.end(); it++, i++) {
			;
			index += *it * (int) pow(Parameters::K, i);
			index %= Parameters::H_SIZE;

			assert(index >= 0);
		}

		return index;
	}

	/*
	 * Returns the median of a set of elements
	 */
	template<class T>
	static T getMedian(vector<T> elements) {
		T median = 0;
		size_t size = elements.size();

		sort(elements.begin(), elements.end());

		if (size != 0) {
			if (size % 2 == 0) {
				median = (elements[size / 2 - 1] + elements[size / 2]) / 2;
			} else {
				median = elements[size / 2];
			}
		}

		return median;
	}

	/*
	 * Computes the semi-quadratic score given number of lines and percentage of valid ones
	 */
	static double computeQuadraticScore(int linesNumber, double percentage) {
		// Compute score component by linesNumber (piecewise function)
		double linesNumComponent;
		if (linesNumber < Parameters::PAPER_LINES_LOW) {
			double coeff = 1 / pow(Parameters::PAPER_LINES_LOW, 2);
			linesNumComponent = coeff * pow(linesNumber, 2);
		} else {
			linesNumComponent = 1;
		}

		return linesNumComponent * percentage;
	}

	/*
	 * Computes the max score between YX and XY scores from lines given by Hough transformation
	 */
	static double computePageDiscoverScore(vector<Vec4i>& lines, vector<Vec4i>& goodLines) {
		double score = 0;

		vector<double> thetasYX, thetasXY;
		// Collecting line's orientations
		for (size_t i = 0; i < lines.size(); i++) {
			Vec4i l = lines[i];

			if (l[3] >= l[1]) {
				thetasYX.push_back(atan2(double(l[3] - l[1]), double(l[2] - l[0])));
			} else {
				thetasYX.push_back(atan2(double(l[1] - l[3]), double(l[0] - l[2])));
			}

			if (l[2] >= l[0]) {
				thetasXY.push_back(atan2(double(l[2] - l[0]), double(l[3] - l[1])));
			} else {
				thetasXY.push_back(atan2(double(l[0] - l[2]), double(l[1] - l[3])));
			}
		}

		bool worksWithMean = false;

		// Some statistics
		Mat meanYX, meanXY;
		Mat devYX, devXY;
		meanStdDev(thetasYX, meanYX, devYX);
		meanStdDev(thetasXY, meanXY, devXY);
		double medianYX = getMedian(thetasYX);
		double medianXY = getMedian(thetasXY);

		int goodYX = 0, goodXY = 0;
		size_t size = lines.size();
		double tol = Parameters::GOOD_LINE_TOLERANCE;
		// Collecting good lines
		for (size_t i = 0; i < size; i++) {
			double elementYX, elementXY;
			if (worksWithMean) {
				elementYX = meanYX.at<double>(0, 0);
				elementXY = meanXY.at<double>(0, 0);
			} else {
				elementYX = medianYX;
				elementXY = medianXY;
			}

			bool isLineGood = false;
			if (fabs(elementYX - thetasYX[i]) < tol) {
				goodYX++;
				isLineGood = true;
			}
			if (fabs(elementXY - thetasXY[i]) < tol) {
				goodXY++;
				isLineGood = true;
			}

			if (isLineGood) {
				goodLines.push_back(lines[i]);
			}
		}

		// Computing final statistics
		if (size != 0) {
			double percentualYX = double(goodYX) / size;
			double percentualXY = double(goodXY) / size;

			// Compute score
			double scoreYX = computeQuadraticScore(size, percentualYX);
			double scoreXY = computeQuadraticScore(size, percentualXY);

			score = std::max(scoreXY, scoreYX);
		}

		return score;
	}

	/*
	 * Draws given lines
	 */
	static void drawLines(Mat& image, const vector<Vec4i>& goodLines, Scalar lineColor, int lineThickness) {
		for (size_t i = 0; i < goodLines.size(); i++) {
			Vec4i l = goodLines[i];
			line(image, Point(l[0], l[1]), Point(l[2], l[3]), lineColor, lineThickness, CV_AA);
		}
	}

	/*
	 * Draw given contours
	 */
	static void drawContours(Mat& image, const vector<vector<Point> >& contours, const vector<Vec4i>& hierarchy) {
		cv::RNG rng(12345);
		int lineThickness = Parameters::CONTOURS_LINE_THICKNESS;

		for (unsigned i = 0; i < contours.size(); i++) {
			Scalar lineColor = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			cv::drawContours(image, contours, i, lineColor, lineThickness, CV_AA, hierarchy, 0, Point());
		}
	}

	/*
	 * Clockwise Comparator Class
	 */
	struct ClockwiseComparator {
		Point centroid;

		ClockwiseComparator(Point _centroid) :
				centroid(_centroid) {
		}

		bool operator()(Point i, Point j) {
			double i_atan = atan2(centroid.y - i.y, centroid.x - i.x);
			double j_atan = atan2(centroid.y - j.y, centroid.x - j.x);
			return (i_atan < j_atan);
		}
	};

	/*
	 * Given a set of points, this function sorts points in clockwise order,
	 * using ClockwiseComparator
	 */
	static void sortPointsClockwise(vector<Point>& points) {
		double x = 0;
		double y = 0;

		int pointsSize = points.size();
		for (int i = 0; i < pointsSize; i++) {
			x += points[i].x;
			y += points[i].y;
		}

		// Defines points centroids
		Point centroid(x / pointsSize, y / pointsSize);

#ifdef DEBUG_UTILS_SORT_POINT_CLOCKWISE
		int maxY = 0, maxX = 0, minY = INT_MAX, minX = INT_MAX;
		for (int i = 0; i < pointsSize; i++) {
			if (points[i].x > maxX) {
				maxX = points[i].x;
			}
			if (points[i].y > maxY) {
				maxY = points[i].y;
			}
			if (points[i].x < minX) {
				minX = points[i].x;
			}
			if (points[i].y < minY) {
				minY = points[i].y;
			}
		}

		Mat before(maxY - minY + 50, maxX - minX + 50, CV_8UC3, Scalar(0, 0, 0));

		circle(before, Point(centroid.x - minX + 25, centroid.y - minY + 25), 8, Scalar(50, 255, 50), -1, 4, 0);
		for (int i = 0; i < pointsSize; i++) {
			circle(before, Point(points[i].x - minX + 25, points[i].y - minY + 25), 4,
					Scalar(100 + 155 / pointsSize * i, 100, 100), -1, 8, 0);
		}
		TShighgui::imshow("BEFORE", before);
#endif

		// Sorting with clockwise comparator
		sort(points.begin(), points.end(), ClockwiseComparator(centroid));

#ifdef DEBUG_UTILS_SORT_POINT_CLOCKWISE
		Mat after(maxY - minY + 50, maxX - minX + 50, CV_8UC3, Scalar(0, 0, 0));

		circle(after, Point(centroid.x - minX + 25, centroid.y - minY + 25), 8, Scalar(50, 255, 50), -1, 4, 0);
		for (int i = 0; i < pointsSize; i++) {
			circle(after, Point(points[i].x - minX + 25, points[i].y - minY + 25), 4,
					Scalar(100 + 155 / pointsSize * i, 100, 100), -1, 8, 0);

			TShighgui::imshow("AFTER", after);
			TShighgui::waitKey(200);
		}
		TShighgui::waitKey(0);
#endif
	}

	/*
	 * Returns the affine invariant given F points
	 */
	static double getAffineInvariant(const vector<Point>& points) {
		double invariant = -1;

		if (points.size() == Parameters::F) {
			Triangle t1(points[0], points[2], points[3]);
			Triangle t2(points[0], points[1], points[2]);

			if (t2.getArea() != 0) {
				invariant = t1.getArea() / t2.getArea();
			}
		}

		return invariant;
	}

	/*
	 * Executes a command and reads the pipe verbose
	 */
	static std::string executeAndGetVerbose(const string cmd) {
		FILE* pipe = popen(cmd.c_str(), "r");
		if (!pipe) {
			return "ERROR";
		}
		char buffer[128];
		std::string result = "";
		while (!feof(pipe)) {
			if (fgets(buffer, 128, pipe) != NULL) {
				result += buffer;
			}
		}
		pclose(pipe);

		return result;
	}

	/*
	 * Returns the original pdf filename.
	 * Truncates the filename at the last occurrence of CONVERT_DELIMITATOR
	 */
	static std::string getPdfFilename(const string jpgFilename) {
		string pdfFilename(jpgFilename);
		string::reverse_iterator rit = pdfFilename.rbegin();

		bool founded = false;
		while (rit != pdfFilename.rend() && !founded) {
			if (*rit == Parameters::CONVERT_DELIMITATOR) {
				founded = true;
			}
			pdfFilename.erase(pdfFilename.size() - 1);
			rit++;
		}

		return pdfFilename;
	}

	/**
	 * Selects k random and distinct indexes from an interval (reservoir sampling)
	 */
	static void selectRandomDistinctIndexes(int n, vector<int>& indexes, int k) {
		for (int i = 0; i < n; i++) {
			if (i < k) {
				indexes.push_back(i);
			} else {
				int c = rand() % (i + 1);
				if (c < k) {
					indexes[c] = i;
				}
			}
		}
	}

	/*
	 * Shows commands window
	 */
	static void showAsynchronousCommands(vector<pair<string, string> > commands) {
		Mat commandsImage = Mat::zeros(Parameters::COMMANDS_IMAGE_SIZE, CV_8UC3);

		Point commandsTitlePosition = Point(15, 25);
		Scalar textColor = Scalar(0, 255, 0);
		int textTickness = 1;
		int lineThickness = 1;
		double textScale = 0.5;
		int topLinePositionY = 40;

		putText(commandsImage, "Asynchronous Commands:", commandsTitlePosition, FONT_HERSHEY_SIMPLEX, textScale,
				textColor, textTickness, CV_AA);
		line(commandsImage, Point(commandsTitlePosition.x, topLinePositionY),
				Point(commandsImage.cols - commandsTitlePosition.x, topLinePositionY), textColor, lineThickness, CV_AA);

		// Commands positions
		int cmd_x = 15;
		int cmd_y = 45;

		if (commands.size() == 0) {
			// No command is available
			putText(commandsImage, "no command is available at the moment", Point(cmd_x, cmd_y += 20),
					FONT_HERSHEY_SIMPLEX, textScale, textColor, textTickness, CV_AA);
		} else {
			for (unsigned int i = 0; i < commands.size(); i++) {
				pair<string, string> command(commands[i]);

				stringstream ss;
				ss << command.first << " - " << command.second;

				putText(commandsImage, ss.str(), Point(cmd_x, cmd_y += 20), FONT_HERSHEY_SIMPLEX, textScale, textColor,
						textTickness, CV_AA);
			}
		}

		// Show
		TShighgui::imshow("Commands", commandsImage);
		TShighgui::moveWindow("Commands", Parameters::WINDOW_FIRST_COL, Parameters::WINDOW_SECOND_ROW);
	}

	/*
	 * Shows info
	 */
	static void showInfoWindow(vector<pair<string, string> > infoVector) {
		Mat infoImage = Mat::zeros(Parameters::INFO_IMAGE_SIZE, CV_8UC3);

		Point infoTitlePosition = Point(15, 25);
		Scalar textColor = Scalar(0, 255, 0);
		int textTickness = 1;
		int lineThickness = 1;
		double textScale = 0.5;
		int topLinePositionY = 40;

		putText(infoImage, "Info:", infoTitlePosition, FONT_HERSHEY_SIMPLEX, textScale, textColor, textTickness, CV_AA);
		line(infoImage, Point(infoTitlePosition.x, topLinePositionY),
				Point(infoImage.cols - infoTitlePosition.x, topLinePositionY), textColor, lineThickness, CV_AA);

		// Info positions
		int info_x = 15;
		int info_y = 45;

		if (infoVector.size() == 0) {
			// No command is available
			putText(infoImage, "no info is available at the moment", Point(info_x, info_y += 20), FONT_HERSHEY_SIMPLEX,
					textScale, textColor, textTickness, CV_AA);
		} else {
			for (unsigned int i = 0; i < infoVector.size(); i++) {
				pair<string, string> info(infoVector[i]);

				stringstream ss;
				ss << info.first << ": " << info.second;

				putText(infoImage, ss.str(), Point(info_x, info_y += 20), FONT_HERSHEY_SIMPLEX, textScale, textColor,
						textTickness, CV_AA);
			}
		}

		// Show
		TShighgui::imshow("Info", infoImage);
		TShighgui::moveWindow("Info", Parameters::WINDOW_SECOND_COL, Parameters::WINDOW_SECOND_ROW);
	}

	//************************************CALIBRATION UTILS***********************************************//

	/*
	 * Gets thresholded image
	 */
	static void getThresholdedImage(Mat& thresholded, const Mat* channelsVector,
			const vector<Parameters::ThresholdStructure> tsVector) {
		thresholded = Mat(channelsVector[0].size(), CV_8UC1, Scalar(255));

		// For each threshold
		for (uint i = 0; i < tsVector.size(); i++) {
			Parameters::ThresholdStructure ts = tsVector[i];

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
		const double BETA = 1.;

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

		return pair<double, double>(precision, recall);
	}

	/*
	 * Objective Function (for a combination of threshold channels)
	 */
	static double objFunction(const Mat& truth, const Mat* channelsVector, const vector<Parameters::ThresholdStructure> tsVector) {
		Mat thresholded;
		getThresholdedImage(thresholded, channelsVector, tsVector);

		pair<double, double> precisionRecall = getPrecisionRecall(truth, thresholded);

		double fMeasure = getFmeasure(precisionRecall.first, precisionRecall.second);

		return fMeasure;
	}

	/*
	 * Objective Function (for a single thresh on a channel)
	 */
	static double objFunction(const Mat& truth, const Mat* channelsVector, Parameters::ThresholdStructure ts) {
		vector<Parameters::ThresholdStructure> tsVector;
		tsVector.push_back(ts);

		return objFunction(truth, channelsVector, tsVector);
	}

	/*
	 * Brute Force Search
	 * NOTE: don't use this type of search!
	 */
	static vector<Parameters::ThresholdStructure> bruteForceSearch(const Mat& truth, const Mat* channelsVector,
			const vector<int>& channelToOptimize, const vector<string>& channelLabels) {
		vector<Parameters::ThresholdStructure> tsVector;

		// For each channel to optimize
		for (unsigned int ch = 0; ch < channelToOptimize.size(); ch++) {
			std::cout << "Optimizing channel " << channelLabels[channelToOptimize[ch]] << "\n";
			Parameters::ThresholdStructure best_ts(channelToOptimize[ch]);
			best_ts.value = numeric_limits<double>::min();

			int steps = 0;
			for (int lower = 0; lower <= 256 - Parameters::THRESH_DISTANCE; lower++) {
				for (int upper = lower + Parameters::THRESH_DISTANCE; upper <= 256; upper++) {
					Parameters::ThresholdStructure ts(channelToOptimize[ch]);
					ts.thresh = pair<int, int>(lower, upper);
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
	static vector<Parameters::ThresholdStructure> greedySearch(const Mat& truth, const Mat* channelsVector,
			const vector<int>& channelToOptimize, const vector<string>& channelLabels) {
		vector<Parameters::ThresholdStructure> tsVector;

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
					if (truth.at<uchar>(i, j) == 255) {
						pixels.push_back(channelsVector[channelToOptimize[ch]].at<uchar>(i, j));
					}
				}
			}

			// Closest thresh based on true pixels
			pair<int, int> closest_thresh;
			closest_thresh.first = (int) *(std::min_element(pixels.begin(), pixels.end()));
			closest_thresh.second = (int) *(std::max_element(pixels.begin(), pixels.end()));

			Parameters::ThresholdStructure ts(channelToOptimize[ch]);
			ts.thresh = closest_thresh;
			ts.value = objFunction(truth, channelsVector, ts);
			Parameters::ThresholdStructure best_ts = greedySearchCore(truth, channelsVector, ts);

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
	static Parameters::ThresholdStructure greedySearchCore(const Mat& truth, const Mat* channelsVector,
			Parameters::ThresholdStructure init_ts) {
		Parameters::ThresholdStructure ts = init_ts;

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
			Parameters::ThresholdStructure best_ts = ts;
			bool finalRangeIncrease = false;
			for (int m = 0; m < 4; m++) {
				Parameters::ThresholdStructure current_ts = best_ts;

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
						if (rangeIncrease
								&& successiveEqualValueAcceptedMoves
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
