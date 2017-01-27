/*
 * PageTracking.hpp
 *
 *  Created on: 22/apr/2013
 *      Author: marco
 */

#ifndef PAGETRACKING_HPP_
#define PAGETRACKING_HPP_

#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "SimpleStateMachine.hpp"
#include "MachineEvents.hpp"
#include "../WebCamCapture.hpp"
#include "../TShighgui.hpp"
#include "../ImageProcessor.hpp"
#include "../Utils.hpp"
#include <tuple>
#include <algorithm>
#include <utility>

#include<math.h>

using namespace cv;
using namespace std;
using namespace ssm;

typedef typename std::tuple<int, int, int> ContourDescriptor;
typedef typename std::tuple<bool, ContourDescriptor, ContourDescriptor> ContourDescriptorsCorrespondence;

struct PageTracking: public SimpleState {
	const static int TRACKING_SUCCESS = 0;
	const static int TRACKING_HAND_OCCLUSION = 1;
	const static int TRACKING_GENERIC_ERROR = 2;
	const static int TRACKING_ANNOTATION_SAVE_ERROR = 3;

	PageTracking(SimpleStateMachine* _context) :
		SimpleState(_context), sift(NULL), initialized(false), failureStartTick(0) {
	}

	xfeatures2d::SIFT * sift;
	Mat imageFromRetrieval;
	Mat imageFromRetrievalResized;
	Mat imageFromRetrievalToShow;
	Mat maskImage;
	vector<KeyPoint> keyPointsFromRetrieval;
	Mat descriptorsFromRetrieval;

	bool initialized;
	double failureStartTick;
	double newAnnotationStartTick;

	virtual void doAction() {
		if (!initialized) {
			initializeTracking();
		}

		// Windows are aligned, so if one try to move the retrieved image...
		TShighgui::moveWindow("Retrieved image",
				Parameters::SHOW_WEBCAM_THREAD_IMAGE ? Parameters::WINDOW_THIRD_COL : Parameters::WINDOW_SECOND_COL,
				Parameters::WINDOW_FIRST_ROW);

		// Gets a frame from webcam
		Mat frame;
		WebCamCapture::captureFrame(frame);

		// Adjust frame orientation
		ImageProcessor::adjustWebCamImage(frame, Parameters::SELECTED_WEB_CAM_ORIENTATION);

		Mat grayFrame;
		// Converts to gray scale
		cvtColor(frame, grayFrame, CV_BGR2GRAY);

		/*
		 * Preprocessing is much effective before resizing
		 * First a bottom hat...
		 */
		ImageProcessor::bottomHat(grayFrame, grayFrame, Parameters::MORPH_SE_SIZE);
		convertScaleAbs(grayFrame, grayFrame, -1, Parameters::THRESHOLD_MAX_VALUE);

		/*
		 * ...then an adaptive threshold on bottom hat result
		 */
		adaptiveThreshold(grayFrame, grayFrame, Parameters::THRESHOLD_MAX_VALUE, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
				CV_THRESH_BINARY, Parameters::ADAPTIVE_THRESHOLD_BLOCK_SIZE, Parameters::ADAPTIVE_THRESHOLD_CONSTANT);

		/*
		 * Resizing after preprocessing causes comparable loss of information in retrieval and webcam image
		 */
		Mat frameResized;
		if (Parameters::RESIZE_TRACKING) {
			resize(grayFrame, frameResized, Size(imageFromRetrievalResized.cols, imageFromRetrievalResized.rows), 0, 0,
					CV_INTER_AREA);
		} else {
			frameResized = grayFrame.clone();
		}

		// SIFT detection on webcam image
		vector<KeyPoint> keyPointsWebcam;
		sift->detect(frameResized, keyPointsWebcam);

		Mat descriptorsWebcam;
		sift->compute(frameResized, keyPointsWebcam, descriptorsWebcam);

		// Find KNN (K=2) matches
		BFMatcher knnSearcher(Parameters::TRACKING_KNN_NORM);

		vector<vector<DMatch> > rawMatches;
		knnSearcher.knnMatch(descriptorsWebcam, descriptorsFromRetrieval, rawMatches, 2);

		// Filter matches with Lowe criterion
		vector<DMatch> loweMatches;
		PageTracking::filterMatchesLowe(rawMatches, loweMatches, Parameters::LOWE_THRESHOLD);

		// Computes matches min distance
		double minDistance = PageTracking::computeMatchesMinDistance(loweMatches);

		int trackingResult = TRACKING_SUCCESS;

		// Image to display
		Mat imageToShow(frame);

		// Check if min distance is less than threshold
		if (minDistance < Parameters::MIN_DISTANCE_THRESHOLD) {
			// Get matches which distance is less then LOWE_FACTOR*min distance
			std::vector<cv::DMatch> loweRadiusMatches;
			PageTracking::filterMatchesRadius(loweMatches, loweRadiusMatches, Parameters::LOWE_FACTOR * minDistance);

			// Compute homography
			if (loweRadiusMatches.size() > Parameters::MIN_HOMOGRAPHY_POINTS) {
				// Gets point2f vectors (query and train) from keyPoint vectors and matches
				vector<Point2f> pointsWebcam;
				vector<Point2f> pointsFromRetrieval;
				PageTracking::point2fFromKeyPointsAndMatches(keyPointsWebcam, keyPointsFromRetrieval,
						loweRadiusMatches, pointsWebcam, pointsFromRetrieval);

				// Adjust points to original position (homography applicated on original image size)
				if (Parameters::RESIZE_TRACKING) {
					adjustPoints(pointsWebcam);
					adjustPoints(pointsFromRetrieval);
				}

				// Compute homography and normalize
				Mat H = findHomography(pointsWebcam, pointsFromRetrieval, CV_RANSAC,
						Parameters::RANSAC_REPROJ_THRESHOLD);
				H /= H.at<double> (2, 2);

				if (determinant(H) > Parameters::HOMOGRAPHY_DETERMINANT_THRESHOLD) {
					failureStartTick = 0;

					// Show warped image for computed homography
					warpPerspective(frame, imageToShow, H, Size(frame.cols, frame.rows), INTER_LINEAR, BORDER_CONSTANT,
							Scalar(255, 255, 255));
				} else {
					trackingResult = processTrackingError(frame);
				}

			} else {
				trackingResult = processTrackingError(frame);
			}

		} else {
			trackingResult = processTrackingError(frame);
		}

		double failureElapsedTime = 0;
		double newAnnotationElapsedTime = 0;
		if (trackingResult != TRACKING_SUCCESS) {
			newAnnotationStartTick = 0;

			// Control timing event
			if (failureStartTick == 0) {
				failureStartTick = (double) getTickCount();
			}
			failureElapsedTime = ((double) getTickCount() - failureStartTick) / getTickFrequency();

			if (failureElapsedTime >= Parameters::PAGE_TRACKING_TIME_TO_FAIL) {
				initialized = false;
				destroyWindow("Retrieved image");
				if (trackingResult == TRACKING_GENERIC_ERROR) {
					context->process_event(EvPageTrackingFailGenericError());
					dynamic_cast<StateMachine*> (context)->setPageErrorWhat("Failed page tracking: generic error");
				} else if (trackingResult == TRACKING_HAND_OCCLUSION) {
					context->process_event(EvPageTrackingFailHandOcclusion());
					dynamic_cast<StateMachine*> (context)->setPageErrorWhat("Failed page tracking: hand occlusion");
				} else if (trackingResult == TRACKING_ANNOTATION_SAVE_ERROR) {
					context->process_event(EvPageTrackingAnnotationSaveError());
					dynamic_cast<StateMachine*> (context)->setPageErrorWhat("Failed page annotation save!");
					dynamic_cast<StateMachine*> (context)->setSaveAnnotationRequest(false);
				}
			}
		} else {
			Mat annotationColorMask;
			ImageProcessor::annotationColorMask(imageToShow, annotationColorMask);

			// Find contours for annotation color mask
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			ImageProcessor::contours(annotationColorMask, contours, hierarchy);

			// Find contours that not intesect or are inner the mask and have and area bigger than a threshold (to remove uneven noise)
			vector<vector<Point> > goodContours;
			for (unsigned int i = 0; i < contours.size(); i++) {
				Moments mu = moments(contours[i]);
				if (mu.m00 > Parameters::ANNOTATION_CONTOUR_MIN_AREA && !ImageProcessor::countourIntersects(maskImage,
						contours[i])) {
					goodContours.push_back(contours[i]);
				}
			}

			// If there is a good candidate contour
			if (goodContours.size() > 0) {
				if (dynamic_cast<StateMachine*> (context)->isSaveAnnotationRequest()) {

					/********** SAVE ANNOTATIONS ***********/

					// Construct contours descriptors and list
					list<ContourDescriptor> contourDescriptors;

					for (unsigned i = 0; i < goodContours.size(); i++) {
						vector<Point> contour = goodContours.at(i);

						Point upperPoint(0, INT_MAX);
						Point lowerPoint(0, 0);

						for (unsigned j = 0; j < contour.size(); j++) {
							Point p = contour.at(j);

							if (p.y < upperPoint.y) {
								upperPoint = p;
							}
							if (p.y > lowerPoint.y) {
								lowerPoint = p;
							}
						}

						ContourDescriptor contourDescriptor = make_tuple((upperPoint.x + lowerPoint.x) / 2, upperPoint.y,
								lowerPoint.y);
						contourDescriptors.insert(contourDescriptors.end(), contourDescriptor);
					}

					// Sorts list by x
					contourDescriptors.sort(compareContourDescriptor);

					// Associate contours
					vector<ContourDescriptorsCorrespondence> contourDescriptorsCorrespondences;
					std::list<ContourDescriptor>::iterator iterator = contourDescriptors.begin();
					while (iterator != contourDescriptors.end()) {
						bool foundCorrespondence = false;
						std::list<ContourDescriptor>::iterator correspondenceIterator = iterator;
						correspondenceIterator++;
						while (!foundCorrespondence && correspondenceIterator != contourDescriptors.end()) {
							int diffUpperY = abs(get<1> (*iterator) - get<1> (*correspondenceIterator));
							int diffLen = abs(
									(get<2> (*iterator) - get<1> (*iterator)) - (get<2> (*correspondenceIterator)
											- get<1> (*correspondenceIterator)));
							if (diffUpperY < Parameters::DIFF_UPPER_Y_THRESHOLD && diffLen
									< Parameters::DIFF_LENGTH_THRESHOLD) {
								foundCorrespondence = true;
								break;
							}
							++correspondenceIterator;
						}

						if (foundCorrespondence) {
							ContourDescriptorsCorrespondence contourDescriptorsCorrespondence = make_tuple(true, *iterator,
									*correspondenceIterator);
							contourDescriptorsCorrespondences.push_back(contourDescriptorsCorrespondence);
							contourDescriptors.erase(correspondenceIterator);
							iterator = contourDescriptors.erase(iterator);
						} else {
							ContourDescriptorsCorrespondence contourDescriptorsCorrespondence = make_tuple(false, *iterator,
									*iterator);
							contourDescriptorsCorrespondences.push_back(contourDescriptorsCorrespondence);
							iterator = contourDescriptors.erase(iterator);
						}
					}

					// Compute mask
					Mat mask = Mat::zeros(annotationColorMask.size(), CV_8UC1);
					for (unsigned i = 0; i < contourDescriptorsCorrespondences.size(); i++) {
						ContourDescriptorsCorrespondence contourDescriptorsCorrespondence =
								contourDescriptorsCorrespondences.at(i);
						bool isCouple = get<0> (contourDescriptorsCorrespondence);
						ContourDescriptor contourDescriptorFirst = get<1> (contourDescriptorsCorrespondence);
						ContourDescriptor contourDescriptorSecond = get<2> (contourDescriptorsCorrespondence);

						Point leftUpperPoint;
						Point rightLowerPoint;
						if (isCouple) {
							leftUpperPoint = Point(get<0> (contourDescriptorFirst),
									min(get<1> (contourDescriptorFirst), get<1> (contourDescriptorSecond)));
							rightLowerPoint = Point(get<0> (contourDescriptorSecond),
									max(get<2> (contourDescriptorFirst), get<2> (contourDescriptorSecond)));
						} else {
							leftUpperPoint = Point(0, get<1> (contourDescriptorFirst));
							rightLowerPoint = Point(mask.cols, get<2> (contourDescriptorFirst));
						}
						rectangle(mask, leftUpperPoint, rightLowerPoint, Scalar(255), CV_FILLED);
					}

					// Bitwise OR, adding new mask to the original one
					maskImage |= mask;

					// Saving mask
					path maskImagePath = dynamic_cast<StateMachine*> (context)->getMaskImagePath();
					imwrite(maskImagePath.string(), maskImage);

					// Save annotations image
					Mat annotationsImage = Mat::zeros(imageFromRetrieval.size(), CV_8UC1);
					resize(maskImage, annotationsImage, annotationsImage.size(), 0, 0, INTER_NEAREST);
					bitwise_and(annotationsImage, Parameters::THRESHOLD_MAX_VALUE - imageFromRetrieval,
							annotationsImage);
					annotationsImage = Parameters::THRESHOLD_MAX_VALUE - annotationsImage;
					path annotationsImagePath = dynamic_cast<StateMachine*> (context)->getAnnotationsImagePath();
					imwrite(annotationsImagePath.string(), annotationsImage,
							vector<int> ( { CV_IMWRITE_JPEG_QUALITY, Parameters::JPEG_COMPRESSION_FACTOR }));

					// Sharpening annotated area
					sharpenAnnotatedArea(imageFromRetrievalToShow, mask);

					// Show!
					TShighgui::imshow("Retrieved image", imageFromRetrievalToShow);

#ifdef DEBUG_TRACKING_SHOW_ANNOTATIONS
					TShighgui::imshow("New mask", mask);
					TShighgui::imshow("All masks", maskImage);
#endif

					dynamic_cast<StateMachine*> (context)->setSaveAnnotationRequest(false);

					/********** END SAVE ANNOTATIONS ***********/
				} else {
					// Control timing event
					if (newAnnotationStartTick == 0) {
						newAnnotationStartTick = (double) getTickCount();
					}
					newAnnotationElapsedTime = ((double) getTickCount() - newAnnotationStartTick) / getTickFrequency();

					if (newAnnotationElapsedTime >= Parameters::PAGE_TRACKING_TIME_PAGE_ANNOTATION) {
						context->process_event(EvPageAnnotationStart());

						newAnnotationStartTick = 0;
					}
				}
			} else {
				/*
				 * TODO importante: cosa fare nel caso ci sia una richiesta di salvataggio annotazione e non viene rilevata la stessa?!?!?
				 * Soluzioni:
				 * 1 - Si azzera (SOLUZIONE ATTUALE)
				 * 2 - Avviso di annotazione fuori dall'inquadratura
				 */
				dynamic_cast<StateMachine*> (context)->setSaveAnnotationRequest(false);

				newAnnotationStartTick = 0;
			}

#ifdef DEBUG_TRACKING_COLOR_COMPONENTS
			Mat contoursMat = Mat::zeros(annotationColorMask.size(), CV_8UC1);
			drawContours(contoursMat, contours, -1, Scalar(255));
			TShighgui::imshow("RAW CONTOURS", contoursMat);

			Mat goodContoursMat = Mat::zeros(annotationColorMask.size(), CV_8UC1);
			drawContours(goodContoursMat, goodContours, -1, Scalar(255));
			TShighgui::imshow("GOOD CONTOURS", goodContoursMat);
#endif
		}

		// Adds verbose on the image
		putVerboseOnImage(imageToShow, "Page Tracking", trackingResult, failureElapsedTime, newAnnotationElapsedTime);

		// Show!
		TShighgui::imshow("Window", imageToShow);
		TShighgui::moveWindow("Window",
				Parameters::SHOW_WEBCAM_THREAD_IMAGE ? Parameters::WINDOW_SECOND_COL : Parameters::WINDOW_FIRST_COL,
				Parameters::WINDOW_FIRST_ROW);

		// Asynchronous commands
		vector<pair<string, string> > commands;
		commands.push_back(pair<string, string>("f", "exit from page tracking"));
		commands.push_back(pair<string, string>("s", "create pdf from annotations"));
		commands.push_back(pair<string, string>("Esc", "exit"));
		Utils::showAsynchronousCommands(commands);


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
			case 'f':
				initialized = false;
				destroyWindow("Retrieved image");
				context->process_event(EvPageTrackingFailGenericError());
				dynamic_cast<StateMachine*> (context)->setPageErrorWhat("Asynchronous exit from page tracking");
				break;
			case 's':
				newAnnotationStartTick = 0;
				failureStartTick = 0;
				context->process_event(EvPdfCreationRequest());
				break;
			default:
				break;
			}
		}
	}

	/* ##############################################################
	 *
	 * Useful methods
	 *
	 * ##############################################################
	 */

	void initializeTracking() {
		failureStartTick = 0;
		newAnnotationStartTick = 0;

		// Get retrieved jpg image and compute jpg keypoints and descriptors
		imageFromRetrieval = dynamic_cast<StateMachine*> (context)->getRetrievalToTrackingImage();
		imageFromRetrievalResized = imageFromRetrieval.clone();

		imageFromRetrievalToShow = Mat(imageFromRetrievalResized);
		cvtColor(imageFromRetrievalToShow, imageFromRetrievalToShow, CV_GRAY2BGR);
		// Gets VideoCapture instance for obtain frame size for resizing
		VideoCapture * vCap = WebCamCapture::getVideoCapture();
		if (Parameters::RESIZE_TRACKING) {
			resize(
					imageFromRetrievalResized,
					imageFromRetrievalResized,
					Size(vCap->get(CV_CAP_PROP_FRAME_HEIGHT) * Parameters::RESIZE_FACTOR,
							vCap->get(CV_CAP_PROP_FRAME_WIDTH) * Parameters::RESIZE_FACTOR), 0, 0, CV_INTER_AREA);
		} else {
			resize(imageFromRetrievalResized, imageFromRetrievalResized,
					Size(vCap->get(CV_CAP_PROP_FRAME_HEIGHT), vCap->get(CV_CAP_PROP_FRAME_WIDTH)), 0, 0, CV_INTER_AREA);
		}

		// SIFT instance creation
		sift = xfeatures2d::SIFT::create(Parameters::SIFT_N_FEATURES, Parameters::SIFT_N_OCTAVE_LAYERS,
				Parameters::SIFT_CONTRAST_THRESHOLD, Parameters::SIFT_EDGE_THRESHOLD, Parameters::SIFT_SIGMA);

		// SIFT detection on retrieved jpg image
		sift->detect(imageFromRetrievalResized, keyPointsFromRetrieval);

		// SIFT descriptors on retrieved jpg image
		sift->compute(imageFromRetrievalResized, keyPointsFromRetrieval, descriptorsFromRetrieval);

		// Image from retrieval to show
		resize(imageFromRetrievalToShow, imageFromRetrievalToShow,
				Size(vCap->get(CV_CAP_PROP_FRAME_HEIGHT), vCap->get(CV_CAP_PROP_FRAME_WIDTH)), 0, 0, CV_INTER_AREA);

		// Load mask image
		maskImage = dynamic_cast<StateMachine*> (context)->getMaskImage();

		// Sharpening annotated area
		sharpenAnnotatedArea(imageFromRetrievalToShow, maskImage);

		// Show!
		TShighgui::imshow("Retrieved image", imageFromRetrievalToShow);

		// Flag initilialized to true
		initialized = true;
	}

	// Processes tracking error
	int processTrackingError(Mat frame) {
		if (dynamic_cast<StateMachine*> (context)->isSaveAnnotationRequest()) {
			return TRACKING_ANNOTATION_SAVE_ERROR;
		} else if (ImageProcessor::skinDetection(frame)) {
			return TRACKING_HAND_OCCLUSION;
		} else {
			return TRACKING_GENERIC_ERROR;
		}
	}

	// Contour descriptors comparator
	static bool compareContourDescriptor(const ContourDescriptor& a, const ContourDescriptor& b) {
		return get<0>(a) < get<0>(b);
	}

	// Readjust points of the original image size
	static void adjustPoints(vector<Point2f>& points) {
		for (unsigned int i = 0; i < points.size(); i++) {
			points[i].x = points[i].x / Parameters::RESIZE_FACTOR;
			points[i].y = points[i].y / Parameters::RESIZE_FACTOR;
		}
	}

	// Adds a coloured shadow on annotated area
	static void sharpenAnnotatedArea(Mat& image, Mat& mask) {
		for (int i = 0; i < image.rows; i++) {
			for (int j = 0; j < image.cols; j++) {
				if (mask.at<uchar> (i, j) > 0) {
					// Blue
					if (image.at<Vec3b> (i, j)[0] >= Parameters::SHARPEN_COLOR_BLUE) {
						image.at<Vec3b> (i, j)[0] -= Parameters::SHARPEN_COLOR_BLUE;
					} else {
						image.at<Vec3b> (i, j)[0] = 0;
					}

					// Green
					if (image.at<Vec3b> (i, j)[1] >= Parameters::SHARPEN_COLOR_GREEN) {
						image.at<Vec3b> (i, j)[1] -= Parameters::SHARPEN_COLOR_GREEN;
					} else {
						image.at<Vec3b> (i, j)[1] = 0;
					}

					// Red
					if (image.at<Vec3b> (i, j)[2] >= Parameters::SHARPEN_COLOR_RED) {
						image.at<Vec3b> (i, j)[2] -= Parameters::SHARPEN_COLOR_RED;
					} else {
						image.at<Vec3b> (i, j)[2] = 0;
					}
				}
			}
		}
	}

	// Filters 2NN matches with Lowe criterion (retains match where first match distance / second match distance minor than threshold)
	static void filterMatchesLowe(vector<vector<DMatch> >& src, vector<DMatch>& dst, double loweThreshold) {
		int i = 0;
		for (vector<vector<DMatch> >::iterator it = src.begin(); it != src.end(); ++it) {
			vector<DMatch>& singleDescriptorMatches = *it;
			if (singleDescriptorMatches.size() >= 2) {
				double ratio = singleDescriptorMatches[0].distance / singleDescriptorMatches[1].distance;
				if (ratio < loweThreshold) {
					dst.push_back(singleDescriptorMatches[0]);
				}
			} else {
				dst.push_back(singleDescriptorMatches[0]); // TODO: what shall we do in case of a single match? Keep it? Leave it?
			}
			i++;
		}
	}

	// Computes min distance of matches vector
	static double computeMatchesMinDistance(const vector<DMatch>& src) {
		double min_dist = INFINITY;
		for (unsigned int i = 0; i < src.size(); i++) {
			double dist = src[i].distance;
			if (dist < min_dist)
				min_dist = dist;
		}
		return min_dist;
	}

	// Filter matches which distance is less than threshold
	static void filterMatchesRadius(const vector<DMatch>& src, vector<DMatch>& dst, double threshold) {
		for (unsigned int i = 0; i < src.size(); i++) {
			if (src[i].distance < threshold) {
				dst.push_back(src[i]);
			}
		}
	}

	// Gets Point2f vectors from vectors of keypoints (query and train) and matches
	static void point2fFromKeyPointsAndMatches(const vector<KeyPoint>& querySrc, const vector<KeyPoint>& trainSrc,
			const vector<DMatch>& matches, vector<Point2f>& queryDst, vector<Point2f>& trainDst) {
		for (unsigned int i = 0; i < matches.size(); i++) {
			queryDst.push_back(querySrc.at(matches[i].queryIdx).pt);
			trainDst.push_back(trainSrc.at(matches[i].trainIdx).pt);
		}
	}

	// Substate in which image is discriminated between hand and page disappear events. Returns

	static void putVerboseOnImage(Mat& image, const char* text, int trackingResult, double failureElapsedTime,
			double newAnnotationElapsedTime) {
		// Parameters
		int topLinePositionY = 50;
		int bottomLinePositionY = 590;
		int lineThickness = 1;
		Point stateTextPosition = Point(15, 30);
		Point adviceTextPosition = Point(15, 620);

		Scalar stateTextColor = Scalar(0, 0, 255);
		Scalar failureTextColor = Scalar(0, 0, 255);
		Scalar newAnnotationTextColor = Scalar(0, 100, 0);

		double textScale = 0.5;
		int textTickness = 1;

		putText(image, text, stateTextPosition, FONT_HERSHEY_DUPLEX, textScale, stateTextColor, textTickness, CV_AA);
		line(image, Point(0, topLinePositionY), Point(image.cols, topLinePositionY), stateTextColor, lineThickness,
				CV_AA);

		stringstream adviceSS;
		if (trackingResult != TRACKING_SUCCESS) {
			int timeToFail = std::ceil(Parameters::PAGE_TRACKING_TIME_TO_FAIL - failureElapsedTime);

			if (trackingResult == TRACKING_HAND_OCCLUSION) {
				adviceSS << "Please take HAND out of the page. Time to fail: " << timeToFail << " s";
			} else if (trackingResult == TRACKING_GENERIC_ERROR) {
				adviceSS << "Tracking generic error. Time to fail: " << timeToFail << " s";
			} else if (trackingResult == TRACKING_ANNOTATION_SAVE_ERROR) {
				adviceSS << "Can't save annotation! Adjust page. Time to fail: " << timeToFail << " s";
			}
			putText(image, adviceSS.str(), adviceTextPosition, FONT_HERSHEY_DUPLEX, textScale, failureTextColor,
					textTickness, CV_AA);
			line(image, Point(0, bottomLinePositionY), Point(image.cols, bottomLinePositionY), failureTextColor,
					lineThickness, CV_AA);
		} else if (newAnnotationElapsedTime != 0) {
			int timeToPageAnnotation = std::ceil(
					Parameters::PAGE_TRACKING_TIME_PAGE_ANNOTATION - newAnnotationElapsedTime);

			if (timeToPageAnnotation < Parameters::PAGE_TRACKING_TIME_PAGE_ANNOTATION) {
				adviceSS << "Detected colour annotation. Time to annotation: " << timeToPageAnnotation << " s";

				putText(image, adviceSS.str(), adviceTextPosition, FONT_HERSHEY_DUPLEX, textScale,
						newAnnotationTextColor, textTickness, CV_AA);
				line(image, Point(0, bottomLinePositionY), Point(image.cols, bottomLinePositionY),
						newAnnotationTextColor, lineThickness, CV_AA);
			}
		}
	}
};

#endif /* PAGETRACKING_HPP_ */
