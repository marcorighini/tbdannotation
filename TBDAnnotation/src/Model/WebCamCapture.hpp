/*
 * WebcamCapture.hpp
 *
 *  Created on: 21/apr/2013
 *      Author: alessandro
 */

#ifndef WEBCAMCAPTURE_HPP_
#define WEBCAMCAPTURE_HPP_

#include <boost/thread/thread.hpp>
#include "Parameters.h"
#include "TShighgui.hpp"
#include "ImageProcessor.hpp"

using namespace cv;

static boost::mutex currentFrameMutex;
static Mat currentFrame;

/*
 * Object for thread initialization
 */
struct WebCamCaptureStruct {
	VideoCapture * cap;
	int samplePeriod;

	WebCamCaptureStruct(VideoCapture * _cap, int _samplePeriod) :
		cap(_cap), samplePeriod(_samplePeriod) {
	}

	void operator()() {
		while (true) {
			{
				boost::mutex::scoped_lock scoped_lock(currentFrameMutex);

				// Do operator >> while VideoCapture gets an invalid image
				do {
					cap->operator >>(currentFrame);
				} while (currentFrame.cols == 0 || currentFrame.rows == 0);

				// Show real time webcam image
				if (Parameters::SHOW_WEBCAM_THREAD_IMAGE) {
					Mat imageToShow = currentFrame.clone();

					// Adjust frame orientation
					ImageProcessor::adjustWebCamImage(imageToShow, Parameters::SELECTED_WEB_CAM_ORIENTATION);

					TShighgui::imshow("Webcam", imageToShow);
					TShighgui::moveWindow("Webcam", Parameters::WINDOW_FIRST_COL, Parameters::WINDOW_FIRST_ROW);
				}

				/*
				 * IMPORTANT!!!
				 * It must be the unique place for call to performWaitKey() method in the application.
				 * This thread runs faster than any state in the annotation state machine,
				 * so cv::waitKey() can update windows state with higher frequency
				 */
				TShighgui::performWaitKey();
			}

			// Sleep...
			boost::xtime xt;
			boost::xtime_get(&xt, boost::TIME_UTC_);
			xt.nsec += samplePeriod;
			boost::thread::sleep(xt);
		}
	}
};

/*
 * Singleton WebCamCapture
 */
class WebCamCapture {
public:
	static const int DEVICE_NUMBER = 0;
	static const int SAMPLE_PERIOD = 1e6;

	// Frame capture
	static void captureFrame(Mat& image) {
		getInstance()._captureFrame(image);
	}

	// Release thread
	static void release() {
		getInstance()._release();
	}

	// VideoCapture getter
	static VideoCapture * getVideoCapture() {
		return getInstance()._getVideoCapture();
	}
private:
	boost::thread* captureThread;
	VideoCapture * cap;

	WebCamCapture() :
		captureThread(NULL) {
	}

	static WebCamCapture& getInstance() {
		// Guaranteed to be destroyed. Instantiated on first use.
		static WebCamCapture instance;

		if (instance.captureThread == NULL) {
			instance._initializeInstance();
		}

		return instance;
	}

	void _initializeInstance() {
		if (captureThread == NULL) {
			cap = new VideoCapture(DEVICE_NUMBER);
			captureThread = new boost::thread(WebCamCaptureStruct(cap, SAMPLE_PERIOD));
		}
	}

	void _captureFrame(Mat& image) {
		boost::mutex::scoped_lock scoped_lock(currentFrameMutex);

		// If device isn't ready, return a black image
		if (currentFrame.cols == 0 || currentFrame.rows == 0) {
			image = Mat(Size(cap->get(CV_CAP_PROP_FRAME_WIDTH), cap->get(CV_CAP_PROP_FRAME_HEIGHT)), CV_8UC3,
					Scalar(0, 0, 0));
		} else {
			image = currentFrame.clone();
		}
	}

	void _release() {
		boost::mutex::scoped_lock scoped_lock(currentFrameMutex);

		captureThread->interrupt();
		delete captureThread;
		captureThread = NULL;
	}

	VideoCapture * _getVideoCapture() {
		return cap;
	}

	// Dont forget to declare these two. You want to make sure they
	// are unaccessable otherwise you may accidently get copies of
	// your singleton appearing.
	WebCamCapture(WebCamCapture const&); // Don't Implement
	void operator=(WebCamCapture const&); // Don't implement

	~WebCamCapture() {
		if (captureThread != NULL) {
			release();
		}
	}
};

#endif /* WEBCAMCAPTURE_HPP_ */
