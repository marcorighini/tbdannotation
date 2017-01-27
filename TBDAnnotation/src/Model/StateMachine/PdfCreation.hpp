/*
 * PdfCreation.hpp
 *
 *  Created on: 22/apr/2013
 *      Author: marco
 */

#ifndef PDFCREATION_HPP_
#define PDFCREATION_HPP_

#include <iostream>
#include "SimpleStateMachine.hpp"
#include "../WebCamCapture.hpp"
#include "../TShighgui.hpp"
#include "MachineEvents.hpp"

using namespace cv;
using namespace std;
using namespace ssm;

static bool conversionResult;
static bool pdfCreationDone;

struct PdfCreation: public SimpleState {
	PdfCreation(SimpleStateMachine* _context) :
		SimpleState(_context), initialized(false) {
	}

	bool initialized;
	path pdfOutputFilenamePath;

	virtual void doAction() {
		if (!initialized) {
			pdfCreationDone = false;
			conversionResult = 0;

			path datasetPath = dynamic_cast<StateMachine*> (context)->getDatasetPath();
			path pdfOutputPath = dynamic_cast<StateMachine*> (context)->getPdfOutputPath();
			path jpgFilename(dynamic_cast<StateMachine*> (context)->getAnnotationsImagePath().stem());
			path pdfFilename = path(Utils::getPdfFilename(jpgFilename.string()));

			path imagesPrefixPath = datasetPath / Parameters::ANNOTATIONS_PATH / pdfFilename;
			pdfOutputFilenamePath = pdfOutputPath / pdfFilename;

			boost::thread(PdfCreationStruct(imagesPrefixPath, pdfOutputFilenamePath));

			initialized = true;
		}

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
		putVerboseOnImage(imageToShow, "Pdf Creation");

		// Show!
		TShighgui::imshow("Window", imageToShow);
		TShighgui::moveWindow("Window",
				Parameters::SHOW_WEBCAM_THREAD_IMAGE ? Parameters::WINDOW_SECOND_COL : Parameters::WINDOW_FIRST_COL,
				Parameters::WINDOW_FIRST_ROW);

		// Asynchronous commands
		vector<pair<string, string> > commands;
		if (pdfCreationDone) {
			commands.push_back(pair<string, string> ("key", "press a key to reset"));
			commands.push_back(pair<string, string> ("Esc", "exit"));
		}
		Utils::showAsynchronousCommands(commands);

		// Info window
		vector<pair<string, string> > infoVector;
		infoVector.push_back(pair<string, string>("Dataset path", dynamic_cast<StateMachine*> (context)->getDatasetPath().string()));
		infoVector.push_back(pair<string, string>("PDF output path", dynamic_cast<StateMachine*> (context)->getPdfOutputPath().string()));
		infoVector.push_back(pair<string, string>("Retrieved image path", dynamic_cast<StateMachine*> (context)->getRetrievedImagePath().string()));
		infoVector.push_back(pair<string, string>("Original document path", dynamic_cast<StateMachine*> (context)->getRetrievedOriginalDocumentPath().string()));
		if (pdfCreationDone){
			infoVector.push_back(pair<string, string>("Created PDF path", pdfOutputFilenamePath.string()));
		}
		Utils::showInfoWindow(infoVector);


		char keyPressed = TShighgui::waitKey();
		if (keyPressed >= 0) {
			if (keyPressed == Parameters::ESC_KEY) {
				context->process_event(EvExit());
			} else {
				context->process_event(EvPdfCreationDone());
				initialized = false;
				pdfCreationDone = false;
				conversionResult = 0;
			}
		}
	}

	static void putVerboseOnImage(Mat& image, const char* text) {
		// Parameters
		int topLinePositionY = 50;
		int bottomLinePositionY = 590;
		int lineThickness = 1;
		Point stateTextPosition = Point(15, 30);
		Point adviceTextPosition = Point(15, 620);

		Scalar stateTextColor = Scalar(0, 0, 255);
		Scalar adviceTextColor = Scalar(0, 100, 0);

		double textScale = 0.5;
		int textTickness = 1;

		putText(image, text, stateTextPosition, FONT_HERSHEY_DUPLEX, textScale, stateTextColor, textTickness, CV_AA);
		line(image, Point(0, topLinePositionY), Point(image.cols, topLinePositionY), stateTextColor, lineThickness,
				CV_AA);

		stringstream adviceSS;
		if (pdfCreationDone) {
			if (conversionResult == 0) {
				adviceSS << "Pdf created! Press a key to go back page tracking";
			} else {
				adviceSS << "Error! convert call has returned " << conversionResult << ", press a key...";
			}
		} else {
			adviceSS << "Pdf creation in progress...";
		}
		putText(image, adviceSS.str(), adviceTextPosition, FONT_HERSHEY_DUPLEX, textScale, adviceTextColor,
				textTickness, CV_AA);
		line(image, Point(0, bottomLinePositionY), Point(image.cols, bottomLinePositionY), adviceTextColor,
				lineThickness, CV_AA);
	}

	/*
	 * Struct used by pdf creation thread
	 */
	struct PdfCreationStruct {
		path imagesPrefixPath;
		path pdfOutputFilenamePath;

		PdfCreationStruct(path _imagesPrefixPath, path _pdfOutputFilenamePath) :
			imagesPrefixPath(_imagesPrefixPath), pdfOutputFilenamePath(_pdfOutputFilenamePath) {
		}

		void operator()() {
			conversionResult = FileManager::createPdfFromImages(imagesPrefixPath, pdfOutputFilenamePath);

			pdfCreationDone = true;
		}
	};
};

#endif /* PDFCREATION_HPP_ */
