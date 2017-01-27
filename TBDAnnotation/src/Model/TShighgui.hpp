/*
 * TShighgui.hpp
 *
 *  Created on: 28/may/2013
 *      Author: alessandro
 */

#ifndef TSHIGHGUI_HPP_
#define TSHIGHGUI_HPP_

#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>

using namespace cv;

/*
 * Singleton Thread-Safe Highgui
 * Multi-threading implementation of most common highgui functions
 */
class TShighgui {
public:
	/*
	 * This method should be called ONLY ONCE (in the fastest thread)
	 */
	static void performWaitKey() {
		getInstance()._performWaitKey();
	}

	/*
	 * Picks up the last non-kept key pressed
	 */
	static char waitKey() {
		return getInstance()._waitKey();
	}

	/*
	 * Simulation of cv::waitKey behaviour with millisecondsDelay set to zero for a infinite delay
	 */
	static char waitKey(int millisecondsDelay) {
		return getInstance()._waitKey(millisecondsDelay);
	}

	/*
	 * Thread safe imshow
	 */
	static void imshow(const char* window, const Mat& image) {
		getInstance()._imshow(window, image);
	}

	/*
	 * Thread safe moveWindow
	 */
	static void moveWindow(const char* window, int x, int y) {
		getInstance()._moveWindow(window, x, y);
	}

	/*
	 * Thread safe destroyWindow
	 */
	static void destroyWindow(const char* window) {
		getInstance()._destroyWindow(window);
	}

private:
	static const int WAIT_KEY_DELAY = 3;
	char lastKeyPressed;
	bool isLastKeyKept;

	boost::mutex waitKeyMutex;
	boost::mutex imshowMutex;

	TShighgui() :
		lastKeyPressed(-1), isLastKeyKept(true) {
	}

	static TShighgui& getInstance() {
		// Guaranteed to be destroyed. Instantiated on first use.
		static TShighgui instance;

		return instance;
	}

	void _performWaitKey() {
		boost::mutex::scoped_lock waitKey_scoped_lock(waitKeyMutex);
		boost::mutex::scoped_lock imshow_scoped_lock(imshowMutex);

		char keyPressed = cv::waitKey(WAIT_KEY_DELAY);
		if (isLastKeyKept && keyPressed != -1) {
			lastKeyPressed = keyPressed;
			isLastKeyKept = false;
		}
	}

	char _waitKey() {
		char key = -1;

		boost::mutex::scoped_lock scoped_lock(waitKeyMutex);
		key = lastKeyPressed;
		lastKeyPressed = -1;
		isLastKeyKept = true;

		return key;
	}

	char _waitKey(int millisecondsDelay) {
		char key = -1;

		double trackingStartTick = (double) getTickCount();
		while (millisecondsDelay == 0 || (getTickCount() - trackingStartTick) < double(millisecondsDelay) / 1000
				* getTickFrequency()) {
			boost::mutex::scoped_lock scoped_lock(waitKeyMutex);

			if (isLastKeyKept == false) {
				key = lastKeyPressed;
				lastKeyPressed = -1;
				isLastKeyKept = true;
				break;
			}
		}

		return key;
	}

	void _imshow(const char* window, const Mat& image) {
		boost::mutex::scoped_lock scoped_lock(imshowMutex);

		cv::imshow(window, image);
	}

	void _moveWindow(const char* window, int x, int y) {
		boost::mutex::scoped_lock scoped_lock(imshowMutex);

		cv::moveWindow(window, x, y);
	}

	void _destroyWindow(const char* window) {
		boost::mutex::scoped_lock scoped_lock(imshowMutex);

		cv::destroyWindow(window);
	}

	// Dont forget to declare these two. You want to make sure they
	// are unaccessable otherwise you may accidently get copies of
	// your singleton appearing.
	TShighgui(TShighgui const&); // Don't Implement
	void operator=(TShighgui const&); // Don't implement

	~TShighgui() {
	}
};

#endif /* TSHIGHGUI_HPP_ */
