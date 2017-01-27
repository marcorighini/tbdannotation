#include <iostream>
#include "Parameters.h"
#include "Utils.hpp"
#include "CustomPermutation.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
	if (argc != 2) {
		std::cerr << "ERROR: Invalid number of arguments" << "\n";
		return 0;
	}

	string destinationPath(argv[1]);
	stringstream filename;
	filename << destinationPath << Parameters::ANNOTATIONS_THRESHOLD_FILENAME;
	FileStorage fs(filename.str(), FileStorage::WRITE);

	if (!fs.isOpened()) {
		std::cerr << "ERROR: Can't create file \"" << filename.str() << "\"" << "\n";
		return 0;
	}

	VideoCapture cap(0);
	bool drawRectangleOnTruth = true;

	// Instructions
	cout << "\nPlease, align a completely white sheet into the green rectangle and then\n"
		"draw a line (or whatever is enough visible) into each red rectangle.\n"
		"Then you have to put your hand in the second half of the window, below the green line.\n"
		"Finally, don't forget to leave visible some part of the background (desk\n"
		"or something else) out of the green boundary.\n"
		"Take a look to the \"Truth\" window: the goal is to view only lines in the red rectangles.\n"
		"\"Truth\" is the mask that the algorithm will try to replicate (note that \"Truth\"\n"
		"is given in grayscale mode thanks to the absence of text).\n"
		"\n"
		"Press 's' to start searching...\n"
		"Press 't' to toggle rectangle draw in \"Truth\" window...\n"
		"Press 'Esc' to quit...\n";

	for (;;) {
		Mat frame;
		cap >> frame;

		Utils::adjustWebCamImage(frame, Parameters::SELECTED_WEB_CAM_ORIENTATION);

		Mat frameGray = frame.clone();
		cvtColor(frameGray, frameGray, CV_BGR2GRAY);

		// ############################################################# //
		// Truth definition section

		Mat adaptiveThresh;
		adaptiveThreshold(frameGray, adaptiveThresh, Parameters::THRESHOLD_MAX_VALUE, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
				CV_THRESH_BINARY, Parameters::ADAPTIVE_THRESHOLD_BLOCK_SIZE, Parameters::ADAPTIVE_THRESHOLD_CONSTANT);

		Mat element = getStructuringElement(Parameters::MORPH_SE_SHAPE, Size(3, 3), Point(1, 1));
		morphologyEx(adaptiveThresh, adaptiveThresh, MORPH_CLOSE, element);

		Mat truth = adaptiveThresh.clone();
		// Clean up
		for (int i = 0; i < truth.rows; i++) {
			for (int j = 0; j < truth.cols; j++) {
				if (!(i >= Parameters::BOUNDARY_MARGIN && i <= truth.rows / 2 && j >= Parameters::BOUNDARY_MARGIN && j
						<= truth.cols - Parameters::BOUNDARY_MARGIN)) {
					truth.at<uchar> (i, j) = 255;
				}
			}
		}

		// Truth: white foreground
		truth = 255 - truth;

		// ######################################################
		// Rectangles definition section

		// Page boundary rectangle
		Rect boundaryRect(Point(Parameters::BOUNDARY_MARGIN, Parameters::BOUNDARY_MARGIN),
				Point(frame.cols - Parameters::BOUNDARY_MARGIN, frame.rows - Parameters::BOUNDARY_MARGIN));

		// Left rectangle
		int rectangleSxWidth = 50;
		Rect annotationRectSX(
				Point(Parameters::BOUNDARY_MARGIN * 2,
						(frame.rows / 2 - Parameters::BOUNDARY_MARGIN) / 4 + Parameters::BOUNDARY_MARGIN),
				Point(Parameters::BOUNDARY_MARGIN * 2 + rectangleSxWidth,
						(frame.rows / 2 - Parameters::BOUNDARY_MARGIN) * 3. / 4 + Parameters::BOUNDARY_MARGIN));

		// Right rectangle
		int rectangleDxWidth = 50;
		Rect annotationRectDX(
				Point(frame.cols - (Parameters::BOUNDARY_MARGIN * 2 + rectangleDxWidth),
						(frame.rows / 2 - Parameters::BOUNDARY_MARGIN) / 4 + Parameters::BOUNDARY_MARGIN),
				Point(frame.cols - Parameters::BOUNDARY_MARGIN * 2,
						(frame.rows / 2 - Parameters::BOUNDARY_MARGIN) * 3. / 4 + Parameters::BOUNDARY_MARGIN));

		// Top rectangle
		int rectangleTopHeight = 50;
		Rect annotationRectTOP(
				Point((frame.cols - Parameters::BOUNDARY_MARGIN * 2) / 4 + Parameters::BOUNDARY_MARGIN,
						Parameters::BOUNDARY_MARGIN * 2),
				Point((frame.cols - Parameters::BOUNDARY_MARGIN * 2) * 3. / 4 + Parameters::BOUNDARY_MARGIN,
						Parameters::BOUNDARY_MARGIN * 2 + rectangleTopHeight));

		// Bottom rectangle
		int rectangleBotHeight = 50;
		Rect annotationRectBOTTOM(
				Point((frame.cols - Parameters::BOUNDARY_MARGIN * 2) / 4 + Parameters::BOUNDARY_MARGIN,
						frame.rows / 2 - Parameters::BOUNDARY_MARGIN),
				Point((frame.cols - Parameters::BOUNDARY_MARGIN * 2) * 3. / 4 + Parameters::BOUNDARY_MARGIN,
						frame.rows / 2 - (Parameters::BOUNDARY_MARGIN + rectangleBotHeight)));

		// ############################################################# //
		// Rectangles and lines drawing section

		Mat frameToShow = frame.clone();

		Mat truthToShow;
		cvtColor(truth, truthToShow, CV_GRAY2BGR);

		// Linea di metà
		line(frameToShow, Point(0, frame.rows / 2), Point(frame.cols, frame.rows / 2), Scalar(0, 255, 0), 2, CV_AA);
		line(truthToShow, Point(0, frame.rows / 2), Point(frame.cols, frame.rows / 2), Scalar(0, 255, 0), 2, CV_AA);

		// Rettangoli vari
		rectangle(frameToShow, boundaryRect, Scalar(0, 255, 0), 1, CV_AA);
		rectangle(frameToShow, annotationRectSX, Scalar(0, 0, 255), 1, CV_AA);
		rectangle(frameToShow, annotationRectDX, Scalar(0, 0, 255), 1, CV_AA);
		rectangle(frameToShow, annotationRectTOP, Scalar(0, 0, 255), 1, CV_AA);
		rectangle(frameToShow, annotationRectBOTTOM, Scalar(0, 0, 255), 1, CV_AA);

		if (drawRectangleOnTruth) {
			rectangle(truthToShow, boundaryRect, Scalar(0, 255, 0), 1, CV_AA);
			rectangle(truthToShow, annotationRectSX, Scalar(0, 0, 255), 1, CV_AA);
			rectangle(truthToShow, annotationRectDX, Scalar(0, 0, 255), 1, CV_AA);
			rectangle(truthToShow, annotationRectTOP, Scalar(0, 0, 255), 1, CV_AA);
			rectangle(truthToShow, annotationRectBOTTOM, Scalar(0, 0, 255), 1, CV_AA);
		}

		// ############################################################# //
		// Image show section

		imshow("Webcam", frameToShow);
		moveWindow("Webcam", Parameters::WINDOW_FIRST_COL, Parameters::WINDOW_FIRST_ROW);

		imshow("Truth", truthToShow);
		moveWindow("Truth", Parameters::WINDOW_SECOND_COL, Parameters::WINDOW_FIRST_ROW);

		// ############################################################# //
		// Wait key section

		char key = waitKey(50);
		if (key == Parameters::ESC_KEY) {
			break;
		} else if (key == 't') {
			drawRectangleOnTruth = !drawRectangleOnTruth;
		} else if (key == 's') {
			// ############################################################# //
			// Search section

			// Channels vector (4 color spaces)
			Mat channelsVector[12];
			vector<string> channelLabels;
			channelLabels.push_back("BGR_B");
			channelLabels.push_back("BGR_G");
			channelLabels.push_back("BGR_R");
			channelLabels.push_back("HSV_H");
			channelLabels.push_back("HSV_S");
			channelLabels.push_back("HSV_V");
			channelLabels.push_back("LAB_L");
			channelLabels.push_back("LAB_A");
			channelLabels.push_back("LAB_B");
			channelLabels.push_back("YCRCB_Y");
			channelLabels.push_back("YCRCB_CR");
			channelLabels.push_back("YCRCB_CB");

			// Blurred
			Mat blurred;
			blur(frame, blurred, Parameters::GAUSSIAN_BLUR_SIZE);

			// Channel split data structure
			vector<Mat> channelsSplit(3);

			// BGR
			split(blurred, channelsSplit);
			channelsVector[Parameters::BGR_B] = channelsSplit[0].clone();
			channelsVector[Parameters::BGR_G] = channelsSplit[1].clone();
			channelsVector[Parameters::BGR_R] = channelsSplit[2].clone();

			// HSV
			Mat hsv;
			cvtColor(blurred, hsv, CV_BGR2HSV);
			split(hsv, channelsSplit);
			channelsVector[Parameters::HSV_H] = channelsSplit[0].clone();
			channelsVector[Parameters::HSV_S] = channelsSplit[1].clone();
			channelsVector[Parameters::HSV_V] = channelsSplit[2].clone();

			// Lab
			Mat lab;
			cvtColor(blurred, lab, CV_BGR2Lab);
			split(lab, channelsSplit);
			channelsVector[Parameters::LAB_L] = channelsSplit[0].clone();
			channelsVector[Parameters::LAB_A] = channelsSplit[1].clone();
			channelsVector[Parameters::LAB_B] = channelsSplit[2].clone();

			// yCbCr
			Mat yCbCr;
			cvtColor(blurred, yCbCr, CV_BGR2YCrCb);
			split(yCbCr, channelsSplit);
			channelsVector[Parameters::YCRCB_Y] = channelsSplit[0].clone();
			channelsVector[Parameters::YCRCB_CR] = channelsSplit[1].clone();
			channelsVector[Parameters::YCRCB_CB] = channelsSplit[2].clone();

			if (Parameters::CHANNEL_PREPROCESSING) {
				// For each channel do some preprocessing
				for (int i = 0; i < 12; i++) {
					Utils::bottomHat(channelsVector[i], channelsVector[i], Parameters::BOTTOM_HAT_MORPH_SE_SIZE);
					medianBlur(channelsVector[i], channelsVector[i], Parameters::MEDIAN_BLUR_SIZE);
				}
			}

			/*
			 * Channel choice (for optimization process)
			 * NOTE: value channels are discarded!
			 */
			vector<int> channelToOptimize;
			channelToOptimize.push_back(Parameters::BGR_B);
			channelToOptimize.push_back(Parameters::BGR_G);
			channelToOptimize.push_back(Parameters::BGR_R);
			channelToOptimize.push_back(Parameters::HSV_H);
			channelToOptimize.push_back(Parameters::HSV_S);
			channelToOptimize.push_back(Parameters::LAB_A);
			channelToOptimize.push_back(Parameters::LAB_B);
			channelToOptimize.push_back(Parameters::YCRCB_CR);
			channelToOptimize.push_back(Parameters::YCRCB_CB);

			// Results vector
			vector<Utils::ThresholdStructure> srVector;

			// Search type selection
			switch (Parameters::SELECTED_SEARCH) {
			case Parameters::BRUTE_FORCE_SEARCH:
				srVector = Utils::bruteForceSearch(truth, channelsVector, channelToOptimize, channelLabels);
				break;
			case Parameters::GREEDY_SEARCH:
				srVector = Utils::greedySearch(truth, channelsVector, channelToOptimize, channelLabels);
				break;
			default:
				break;
			}

			/*
			 * Searching for a good combination of channels (max Parameters::MAX_COMBINATION channels)
			 */
			vector<Utils::ThresholdStructure> best_vector;
			double best_value = std::numeric_limits<double>::min();
			for (int k = 1; k <= Parameters::MAX_COMBINATION; k++) {
				double best_k_value = std::numeric_limits<double>::min();
				vector<Utils::ThresholdStructure> best_k_vector;

				cp::NextCombinationGenerator<Utils::ThresholdStructure> combinationGen(srVector, k);
				cp::NextCombinationGenerator<Utils::ThresholdStructure>::iterator combIt;
				for (combIt = combinationGen.begin(); combIt != combinationGen.end(); combIt++) {
					// Vector with the current combination
					vector<Utils::ThresholdStructure> combination(*combIt);

					double value = Utils::objFunction(truth, channelsVector, combination);

					if (value > best_k_value) {
						best_k_value = value;
						best_k_vector = combination;
					}
				}

				/*
				 * Is really necessary to select a more complicated combination?
				 * Aspiration criteria: choose a more complicated combination if it leads
				 * to an improvement proportional to the current best value
				 */
				if (best_k_value > best_value * Parameters::ASPIRATION_CRITERIA_FACTOR) {
					best_value = best_k_value;
					best_vector = best_k_vector;
				}
			}

			// ############################################################# //
			// Final results section
			Mat thresholded;
			Utils::getThresholdedImage(thresholded, channelsVector, best_vector);

			Mat result;
			Utils::getErrorImage(result, truth, thresholded);

			destroyWindow("Webcam");
			destroyWindow("Truth");

			imshow("Final Thresh", thresholded);
			moveWindow("Final Thresh", Parameters::WINDOW_FIRST_COL, Parameters::WINDOW_FIRST_ROW);

			imshow("Error wrt Truth", result);
			moveWindow("Error wrt Truth", Parameters::WINDOW_SECOND_COL, Parameters::WINDOW_FIRST_ROW);

			pair<double, double> precisionRecall = Utils::getPrecisionRecall(truth, thresholded);

			std::cout << "\nBest combination (max " << Parameters::MAX_COMBINATION
					<< " channels):\n-------------------------------------------------\n";
			cout << "F-Measure:\t" << best_value << "\n";
			cout << "\tPrecision:\t" << precisionRecall.first << "\n";
			cout << "\tRecall:\t\t" << precisionRecall.second << "\n";
			cout << "Channels:\n";
			for (unsigned int ch = 0; ch < best_vector.size(); ch++) {
				cout << "\t" << channelLabels[best_vector[ch].channel] << "\t";
				cout << "[" << best_vector[ch].thresh.first << ", " << best_vector[ch].thresh.second << "]\n";
			}

			// ############################################################# //
			// Saving section
			fs << "number" << (int) best_vector.size();

			for (unsigned int ch = 0; ch < best_vector.size(); ch++) {
				stringstream ss;
				ss << "thresh" << "_" << ch;
				fs << ss.str() << "{";
				fs << "channel" << channelLabels[best_vector[ch].channel];
				fs << "channel_id" << best_vector[ch].channel;
				fs << "thresh" << "{" << "first" << best_vector[ch].thresh.first << "second"
						<< best_vector[ch].thresh.second << "}";
				fs << "}";
			}
			fs.release();

			std::cout << "\nSaving" << "\n-------------------------------------------------\n";
			std::cout << "Saved on yml file \"" << filename.str() << "\"\n\n";

			std::cout << "Press a key to reset...\n";
			std::cout << "Press Esc to quit...\n";
			key = waitKey(0);
			if (key == Parameters::ESC_KEY) {
				break;
			}

			destroyWindow("Final Thresh");
			destroyWindow("Error wrt Truth");
		}
	}

	return 0;
}
