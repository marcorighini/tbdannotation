/*
 * PageAnnotation.hpp
 *
 *  Created on: 22/apr/2013
 *      Author: marco
 */

#ifndef PAGEANNOTATION_HPP_
#define PAGEANNOTATION_HPP_

#include <opencv2/video/tracking.hpp>
#include <iostream>
#include "SimpleStateMachine.hpp"
#include "../WebCamCapture.hpp"
#include "../TShighgui.hpp"
#include "MachineEvents.hpp"

using namespace cv;
using namespace std;
using namespace ssm;

struct PageAnnotation: public SimpleState {
	PageAnnotation(SimpleStateMachine* _context) :
		SimpleState(_context), startTick(0) {
	}

	double startTick;
	vector<cv::Point2f> previousPoints;
	Mat framePrevious;

	virtual void doAction() {
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

		// Noise reduction
		GaussianBlur(gray, gray, Parameters::PAGE_ANNOTATION_BLUR_SIZE, 0, 0, BORDER_CONSTANT);

		// Total motion
		double totalMotion = 0;

		// Waiting for a previous image
		if (previousPoints.size() > 0) {
			vector<cv::Point2f> nextPoints(previousPoints.size());

			std::vector<uchar> status;
			Mat err;

			// Optical flow
			calcOpticalFlowPyrLK(framePrevious, gray, previousPoints, nextPoints, status, err,
					Parameters::PAGE_ANNOTATION_WIN_SIZE);

			for (unsigned int i = 0; i < nextPoints.size(); i++) {
				// Filtering with status information
				if (status[i] == 1) {
					Point2f np = nextPoints[i];
					Point2f pp = previousPoints[i];

					// Motion
					double motion = sqrt((np.y - pp.y) * (np.y - pp.y) + (np.x - pp.x) * (np.x - pp.x));

					// Motion threshold
					if (motion > Parameters::ANNOTATION_MOTION_LOWER_THRESHOLD && motion
							< Parameters::ANNOTATION_MOTION_UPPER_THRESHOLD) {
						totalMotion += motion;

						// Good optical flow points ot show
						cv::circle(imageToShow, previousPoints[i], Parameters::PAGE_ANNOTATION_CIRCLE_RADIUS,
								Parameters::PAGE_ANNOTATION_OPTICAL_FLOW_COLOR,
								Parameters::PAGE_ANNOTATION_CIRCLE_THICKNESS, 1);
						cv::line(imageToShow, pp, np, Parameters::PAGE_ANNOTATION_OPTICAL_FLOW_COLOR,
								Parameters::PAGE_ANNOTATION_LINE_THICKNESS);
					}
				}
			}
		}

		// Control timing event
		double elapsedTime = 0;
		if (totalMotion < Parameters::ANNOTATION_TOTAL_MOTION_THRESHOLD) {
			if (startTick == 0) {
				startTick = (double) getTickCount();
			}
			elapsedTime = ((double) getTickCount() - startTick) / getTickFrequency();

			if (elapsedTime >= Parameters::PAGE_ANNOTATION_TIME_TO_SAVE) {
				dynamic_cast<StateMachine*> (context)->setSaveAnnotationRequest(true);
				context->process_event(EvPageAnnotationEnd());
				startTick = 0;
			}

		} else {
			startTick = 0;
		}

		// New definition of a lattice of points for previousPoints
		previousPoints.clear();
		int latticeDistance = Parameters::ANNOTATION_LATTICE_DISTANCE;
		for (int i = latticeDistance; i < gray.cols; i += latticeDistance) {
			for (int j = latticeDistance; j < gray.rows; j += latticeDistance) {
				previousPoints.push_back(Point2f(i, j));
			}
		}

		// Clone
		framePrevious = gray.clone();

		// Adds verbose on the image
		putVerboseOnImage(imageToShow, "Page Annotation", totalMotion, elapsedTime);

		// Show!
		TShighgui::imshow("Window", imageToShow);
		TShighgui::moveWindow("Window",
				Parameters::SHOW_WEBCAM_THREAD_IMAGE ? Parameters::WINDOW_SECOND_COL : Parameters::WINDOW_FIRST_COL,
				Parameters::WINDOW_FIRST_ROW);

		// Asynchronous commands
		vector<pair<string, string> > commands;
		commands.push_back(pair<string, string> ("s", "terminate and save annotation"));
		commands.push_back(pair<string, string> ("Esc", "exit"));
		Utils::showAsynchronousCommands(commands);

		// Info window
		vector<pair<string, string> > infoVector;
		infoVector.push_back(pair<string, string>("Dataset path", dynamic_cast<StateMachine*> (context)->getDatasetPath().string()));
		infoVector.push_back(pair<string, string>("PDF output path", dynamic_cast<StateMachine*> (context)->getPdfOutputPath().string()));
		infoVector.push_back(pair<string, string>("Retrieved image path", dynamic_cast<StateMachine*> (context)->getRetrievedImagePath().string()));
		infoVector.push_back(pair<string, string>("Original document path", dynamic_cast<StateMachine*> (context)->getRetrievedOriginalDocumentPath().string()));
		Utils::showInfoWindow(infoVector);

		char keyPressed = TShighgui::waitKey();
		if (keyPressed >= 0) {
			switch (keyPressed) {
			case Parameters::ESC_KEY:
				context->process_event(EvExit());
				break;
			case 's':
				// Notification to PageTracking for annotation save
				dynamic_cast<StateMachine*> (context)->setSaveAnnotationRequest(true);
				context->process_event(EvPageAnnotationEnd());
				break;
			default:
				break;
			}
		}
	}

	static void putVerboseOnImage(Mat& image, const char* text, double totalMotion, double elapsedTime) {
		// Parameters
		int topLinePositionY = 50;
		int bottomLinePositionY = 590;
		int lineThickness = 1;
		Point stateTextPosition = Point(15, 30);
		Point motionPosition = Point(380, 30);
		Point adviceTextPosition = Point(15, 620);

		Scalar stateTextColor = Scalar(0, 0, 255);
		Scalar adviceTextColor = Scalar(0, 100, 0);
		Scalar motionColor = Scalar(0, 100, 0);

		double textScale = 0.5;
		int textTickness = 1;

		putText(image, text, stateTextPosition, FONT_HERSHEY_DUPLEX, textScale, stateTextColor, textTickness, CV_AA);
		line(image, Point(0, topLinePositionY), Point(image.cols, topLinePositionY), stateTextColor, lineThickness,
				CV_AA);

		stringstream adviceSS;
		adviceSS << "Complete annotation. ";
		if (totalMotion < Parameters::ANNOTATION_TOTAL_MOTION_THRESHOLD) {
			int timeToSave = std::ceil(Parameters::PAGE_ANNOTATION_TIME_TO_SAVE - elapsedTime);

			if (timeToSave < Parameters::PAGE_ANNOTATION_TIME_TO_SAVE) {
				adviceSS << "Time to save: " << timeToSave << " s";
			}

			putText(image, "No motion", motionPosition, FONT_HERSHEY_DUPLEX, textScale, motionColor, textTickness,
					CV_AA);
		}

		putText(image, adviceSS.str(), adviceTextPosition, FONT_HERSHEY_DUPLEX, textScale, adviceTextColor,
				textTickness, CV_AA);
		line(image, Point(0, bottomLinePositionY), Point(image.cols, bottomLinePositionY), adviceTextColor,
				lineThickness, CV_AA);
	}
};

#endif /* PAGEANNOTATION_HPP_ */
