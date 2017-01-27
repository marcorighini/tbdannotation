/*
 * PageRetrieval.hpp
 *
 *  Created on: 22/apr/2013
 *      Author: marco
 */

#ifndef PAGERETRIEVAL_HPP_
#define PAGERETRIEVAL_HPP_

#include <iostream>
#include "SimpleStateMachine.hpp"
#include "MachineEvents.hpp"
#include "../WebCamCapture.hpp"
#include "../ImageProcessor.hpp"
#include "../Utils.hpp"

using namespace cv;
using namespace std;
using namespace ssm;

struct PageRetrieval: public SimpleState {
	PageRetrieval(SimpleStateMachine* _context) :
		SimpleState(_context) {
	}

	virtual void doAction() {
		Mat imageFromDiscovering;

		// Image recovering
		imageFromDiscovering = dynamic_cast<StateMachine*> (context)->getDiscoverToRetrieval();

		vector<Point> featurePoints;
		ImageProcessor::getImageFeaturePoints(imageFromDiscovering, featurePoints);

		Mat imageToShow(imageFromDiscovering);
		cvtColor(imageToShow, imageToShow, CV_GRAY2BGR);

		// Adds verbose on the image
		putVerboseOnImage(imageToShow, "Retrieving page...");

		// Asynchronous commands
		vector<pair<string, string> > commands;
		Utils::showAsynchronousCommands(commands);

		// Info window
		vector<pair<string, string> > infoVector;
		infoVector.push_back(
				pair<string, string> ("Dataset path", dynamic_cast<StateMachine*> (context)->getDatasetPath().string()));
		infoVector.push_back(
				pair<string, string> ("PDF output path",
						dynamic_cast<StateMachine*> (context)->getPdfOutputPath().string()));
		Utils::showInfoWindow(infoVector);

		// Show!
		TShighgui::imshow("Window", imageToShow);
		TShighgui::moveWindow("Window",
				Parameters::SHOW_WEBCAM_THREAD_IMAGE ? Parameters::WINDOW_SECOND_COL : Parameters::WINDOW_FIRST_COL,
				Parameters::WINDOW_FIRST_ROW);

		START_TIMER(1);
		string votedPageId = FeaturesManager::retrievePage(featurePoints);
		std::cout << "\nVoted page Id: " << votedPageId << "\n";
		END_TIME(1, "Elapsed time");

		if (!dynamic_cast<StateMachine*> (context)->isTest()) {
			// Normal annotation mode

			// Gets into page tracking if retrieve page method gave a good pageId
			if (votedPageId.compare("") != 0) {
				path datasetPath = dynamic_cast<StateMachine*> (context)->getDatasetPath();

				// Get retrieved jpg image
				path imagesFolder = datasetPath / Parameters::IMAGES_PATH;
				path imagePath = imagesFolder / votedPageId;
				Mat imageFromRetrieval = imread(imagePath.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
				dynamic_cast<StateMachine*> (context)->setRetrievalToTrackingImage(imageFromRetrieval);
				dynamic_cast<StateMachine*> (context)->setRetrievedImagePath(imagePath);

				RegisteredPdfMap::const_iterator result = Parameters::registeredPdfMap.find(
						Utils::getPdfFilename(votedPageId));
				if (result != Parameters::registeredPdfMap.end()) {
					dynamic_cast<StateMachine*> (context)->setRetrievedOriginalDocumentPath(result->second);
				}

				// Get mask for retrieved jpg image (or save black image if not exists)
				path masksPath = datasetPath / Parameters::MASKS_PATH;
				path maskImagePath = path((masksPath / (path(votedPageId.c_str())).stem()).string() + ".bmp");
				Mat maskImage;
				if (exists(maskImagePath)) {
					maskImage = imread(maskImagePath.string(), CV_LOAD_IMAGE_GRAYSCALE);
				} else {
					maskImage = Mat::zeros(imageToShow.size(), CV_8UC1);
					imwrite(maskImagePath.string(), maskImage);
				}

				// Annotations path
				path annotationsPath = datasetPath / Parameters::ANNOTATIONS_PATH;
				path annotationsImagePath = path(
						(annotationsPath / (path(votedPageId.c_str())).stem()).string() + ".jpg");

				// Set data for PageTracking execution
				dynamic_cast<StateMachine*> (context)->setMaskImage(maskImage);
				dynamic_cast<StateMachine*> (context)->setMaskImagePath(maskImagePath);
				dynamic_cast<StateMachine*> (context)->setAnnotationsImagePath(annotationsImagePath);

				context->process_event(EvPageRetrievalDone());
			} else {
				context->process_event(EvPageRetrievalError());
				dynamic_cast<StateMachine*> (context)->setPageErrorWhat("Failed page retrieval: try again");
			}

		} else {
			// Test mode

			string pageName = dynamic_cast<StateMachine*> (context)->getPageName();
			std::cout << "\n\t\t\t" << "------------------------------------------\n";
			std::cout << "\t\t\t" << "TRUE PAGE NAME: " << pageName << "\n";
			std::cout << "\t\t\t" << "------------------------------------------\n";

			context->process_event(EvPageRetrievalDone());
		}
	}

	static void putVerboseOnImage(Mat& image, const char* text) {
		// Parameters
		Scalar textColor = Scalar(0, 0, 255);
		int linePositionY = 50;
		int lineThickness = 1;
		Point textPosition = Point(15, 30);
		double textScale = 0.5;
		int textTickness = 1;

		putText(image, text, textPosition, FONT_HERSHEY_DUPLEX, textScale, textColor, textTickness, CV_AA);
		line(image, Point(0, linePositionY), Point(image.cols, linePositionY), textColor, lineThickness, CV_AA);

	}
};

#endif /* PAGERETRIEVAL_HPP_ */
