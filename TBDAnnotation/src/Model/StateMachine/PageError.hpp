/*
 * PageError.hpp
 *
 *  Created on: 22/apr/2013
 *      Author: marco
 */

#ifndef PAGEERROR_HPP_
#define PAGEERROR_HPP_

#include <iostream>
#include "SimpleStateMachine.hpp"
#include "../WebCamCapture.hpp"
#include "../TShighgui.hpp"
#include "MachineEvents.hpp"

using namespace cv;
using namespace std;
using namespace ssm;

struct PageError: public SimpleState {
	PageError(SimpleStateMachine* _context) :
		SimpleState(_context) {
	}

	virtual void doAction() {
		// Get error message
		string pageErrorWhat = dynamic_cast<StateMachine*> (context)->getPageErrorWhat();

		Mat frame;
		// Gets a frame
		WebCamCapture::captureFrame(frame);

		Mat gray;
		// Converts to gray scale
		cvtColor(frame, gray, CV_BGR2GRAY);

		// Adjust image for visualization
		ImageProcessor::adjustWebCamImage(gray, Parameters::SELECTED_WEB_CAM_ORIENTATION);

		// Image to display
		Mat imageToShow(gray);
		cvtColor(imageToShow, imageToShow, CV_GRAY2BGR);

		// Adds verbose on the image
		putVerboseOnImage(imageToShow, "Page Error", pageErrorWhat);

		// Asynchronous commands
		vector<pair<string, string> > commands;
		commands.push_back(pair<string, string> ("key", "press a key to reset"));
		commands.push_back(pair<string, string> ("Esc", "exit"));
		Utils::showAsynchronousCommands(commands);

		// Info window
		vector<pair<string, string> > infoVector;
		infoVector.push_back(pair<string, string>("Dataset path", dynamic_cast<StateMachine*> (context)->getDatasetPath().string()));
		infoVector.push_back(pair<string, string>("PDF output path", dynamic_cast<StateMachine*> (context)->getPdfOutputPath().string()));
		Utils::showInfoWindow(infoVector);

		// Show!
		TShighgui::imshow("Window", imageToShow);
		TShighgui::moveWindow("Window",
				Parameters::SHOW_WEBCAM_THREAD_IMAGE ? Parameters::WINDOW_SECOND_COL : Parameters::WINDOW_FIRST_COL,
				Parameters::WINDOW_FIRST_ROW);
		char keyPressed = TShighgui::waitKey();
		if (keyPressed >= 0) {
			if (keyPressed == Parameters::ESC_KEY) {
				context->process_event(EvExit());
			} else {
				context->process_event(EvErrorReset());
			}
		}
	}

	static void putVerboseOnImage(Mat& image, const char* text, const string what) {
		// Parameters
		Scalar textColor = Scalar(0, 0, 255);
		int linePositionY = 50;
		int lineThickness = 1;
		Point textPosition = Point(15, 30);
		Point pressKeyPosition = Point(100, 600);
		double textScale = 0.5;
		double pressKeyScale = 0.8;
		int textTickness = 1;

		stringstream ss;
		ss << text << " - " << what;
		putText(image, ss.str(), textPosition, FONT_HERSHEY_DUPLEX, textScale, textColor, textTickness, CV_AA);
		line(image, Point(0, linePositionY), Point(image.cols, linePositionY), textColor, lineThickness, CV_AA);

		putText(image, "Press a key to reset", pressKeyPosition, FONT_HERSHEY_DUPLEX, pressKeyScale, textColor,
				textTickness, CV_AA);
	}
};

#endif /* PAGEERROR_HPP_ */
