/*
 * PageDiscovering.hpp
 *
 *  Created on: 22/apr/2013
 *      Author: marco
 */

#ifndef PAGEDISCOVERING_HPP_
#define PAGEDISCOVERING_HPP_

#include <iostream>
#include "StateMachine.hpp"
#include "MachineEvents.hpp"
#include "../WebCamCapture.hpp"
#include "../TShighgui.hpp"
#include "../ImageProcessor.hpp"
#include "../Utils.hpp"
#include "../Parameters.h"
#include <boost/lexical_cast.hpp>

using namespace cv;
using namespace std;
using namespace ssm;

struct PageDiscovering: public SimpleState {
	double startTick;

	PageDiscovering(SimpleStateMachine* _context) :
		SimpleState(_context), startTick(0) {
	}

	virtual void doAction() {
		Mat frame;
		// Gets a frame
		WebCamCapture::captureFrame(frame);

		Mat gray;
		// Converts to gray scale
		cvtColor(frame, gray, CV_BGR2GRAY);

		// Adjust image for visualization
		ImageProcessor::adjustWebCamImage(gray, Parameters::SELECTED_WEB_CAM_ORIENTATION);

		Mat adaptiveThresh;
		// Computes adaptive threshold
		adaptiveThreshold(gray, adaptiveThresh, Parameters::THRESHOLD_MAX_VALUE, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
				CV_THRESH_BINARY, Parameters::ADAPTIVE_THRESHOLD_BLOCK_SIZE, Parameters::ADAPTIVE_THRESHOLD_CONSTANT);

		// Convert scale
		convertScaleAbs(adaptiveThresh, adaptiveThresh, -1, Parameters::THRESHOLD_MAX_VALUE);

		vector<Vec4i> lines;
		// Computes Hough's transformation
		ImageProcessor::houghLines(adaptiveThresh, lines);

		vector<Vec4i> goodLines;
		// Computes the page discover score and gets goodLines to be drawn
		double score = Utils::computePageDiscoverScore(lines, goodLines);

		// Control timing event
		double elapsedTime = 0;
		if (score >= Parameters::PAGE_DISCOVERING_THRESHOLD) {
			if (startTick == 0) {
				startTick = (double) getTickCount();
			}
			elapsedTime = ((double) getTickCount() - startTick) / getTickFrequency();

			if (elapsedTime >= Parameters::PAGE_DISCOVERING_TIME_TO_RETRIEVE) {
				dynamic_cast<StateMachine*> (context)->setDiscoverToRetrievalImage(gray);

				context->process_event(EvPageDiscoveringSuccess());
				startTick = 0;
			}

		} else {
			startTick = 0;
		}

		// Image to display
		Mat imageToShow(gray);
		cvtColor(imageToShow, imageToShow, CV_GRAY2BGR);

		// Draws good lines on image
		Utils::drawLines(imageToShow, goodLines, Parameters::PAGE_DISCOVERING_LINE_COLOR,
				Parameters::PAGE_DISCOVERING_LINE_THICKNESS);

		// Adds verbose on the image
		putVerboseOnImage(imageToShow, "Page Discovering", score, elapsedTime);

		// Show!
		TShighgui::imshow("Window", imageToShow);
		TShighgui::moveWindow("Window",
				Parameters::SHOW_WEBCAM_THREAD_IMAGE ? Parameters::WINDOW_SECOND_COL : Parameters::WINDOW_FIRST_COL,
				Parameters::WINDOW_FIRST_ROW);

		// Asynchronous commands
		vector<pair<string, string> > commands;
		commands.push_back(pair<string, string>("s", "start image retrieval"));
		commands.push_back(pair<string, string>("Esc", "exit"));
		Utils::showAsynchronousCommands(commands);

		// Info window
		vector<pair<string, string> > infoVector;
		infoVector.push_back(pair<string, string>("Dataset path", dynamic_cast<StateMachine*> (context)->getDatasetPath().string()));
		infoVector.push_back(pair<string, string>("PDF output path", dynamic_cast<StateMachine*> (context)->getPdfOutputPath().string()));
		Utils::showInfoWindow(infoVector);

		char keyPressed = TShighgui::waitKey();
		if (keyPressed >= 0) {
			switch (keyPressed) {
			case Parameters::ESC_KEY:
				context->process_event(EvExit());
				break;
			case 's':
				dynamic_cast<StateMachine*> (context)->setDiscoverToRetrievalImage(gray);

				context->process_event(EvPageDiscoveringSuccess());
				startTick = 0;
				break;
			default:
				break;
			}
		}
	}

	void putVerboseOnImage(Mat& image, const char* text, double score, double elapsedTime) {
		// Parameters
		int topLinePositionY = 50;
		int bottomLinePositionY = 590;
		int lineThickness = 1;
		Point stateTextPosition = Point(15, 30);
		Point scorePosition = Point(400, 30);
		Point adviceTextPosition = Point(15, 620);

		Scalar stateTextColor = Scalar(0, 0, 255);
		Scalar adviceTextColor = Scalar(0, 100, 0);

		double textScale = 0.5;
		int textTickness = 1;

		putText(image, text, stateTextPosition, FONT_HERSHEY_DUPLEX, textScale, stateTextColor, textTickness, CV_AA);
		line(image, Point(0, topLinePositionY), Point(image.cols, topLinePositionY), stateTextColor, lineThickness,
				CV_AA);

		int percentual = ((score + 0.005) * 100);
		string percentualText = boost::lexical_cast<std::string>(percentual);
		percentualText = percentualText + std::string(" %");

		Scalar scoreColor;
		if (score >= Parameters::PAGE_DISCOVERING_THRESHOLD) {
			scoreColor = Scalar(0, 100, 0);

			int timeToRetrieve = std::ceil(Parameters::PAGE_DISCOVERING_TIME_TO_RETRIEVE - elapsedTime);

			stringstream adviceSS;
			adviceSS << "Adjust page. Time to retrieve: " << timeToRetrieve << " s";
			putText(image, adviceSS.str(), adviceTextPosition, FONT_HERSHEY_DUPLEX, textScale, adviceTextColor,
					textTickness, CV_AA);
			line(image, Point(0, bottomLinePositionY), Point(image.cols, bottomLinePositionY), adviceTextColor,
					lineThickness, CV_AA);
		} else {
			scoreColor = Scalar(0, 0, 255);
		}
		putText(image, percentualText, scorePosition, FONT_HERSHEY_DUPLEX, textScale, scoreColor, textTickness, CV_AA);
	}
};

#endif /* PAGEDISCOVERING_HPP_ */
