#include "Model/Page.hpp"
#include "Model/Utils.hpp"
#include "Model/DatabaseHandler.hpp"
#include "Model/RegisteredPdfMap.hpp"
#include "Model/FileManager.hpp"
#include "Model/ImageProcessor.hpp"
#include "Model/Triangle.hpp"
#include "Model/FeaturesManager.hpp"
#include "Model/WebCamCapture.hpp"
#include <iostream>
#include "Model/ProgramOptions.hpp"
#include "Model/StateMachine/StateMachine.hpp"
#include "Model/StateMachine/ExitState.hpp"
#include "Model/StateMachine/PageDiscovering.hpp"
#include "Model/StateMachine/MachineEvents.hpp"
#include "Model/StateMachine/PageRetrieval.hpp"
#include "Model/StateMachine/PageTracking.hpp"
#include "Model/StateMachine/PageError.hpp"
#include "Model/StateMachine/PageAnnotation.hpp"
#include "Model/StateMachine/PdfCreation.hpp"
#include <thread>
#include <X11/Xlib.h>
#include <vector>

int main(int argc, char** argv) {

	//##############################################################################//
	//							PROGRAM OPTIONS
	//##############################################################################//

	// Program options variables
	string appName = "TBDAnnotation";
	string subcommand;
	string documentsPathString, outputPathString, datasetPathString;
	int nSets = 1;
	uint nRetained = 1;
	string pageNameString;
	string calibrationPathString;
	bool calibrationSupplied = false;

	// Help string
	std::string subcommandDesc = std::string("subcommand to execute: calibrate, register, annotate, test");
	std::string helpDesc = std::string("print usage messages\n");
	std::string documentsPathDesc = std::string("path to pdf documents");
	std::string outputPathDesc = std::string("output files directory");
	std::string datasetPathDesc = std::string("dataset files directory");
	std::string nSetsDesc = std::string(
			"number of datasets. The i-th dataset has size i*nPages/nSets. Optional, default=1");
	std::string nRetainedDesc = std::string(
			"number of same pages that will be present in every dataset. Required if nSets is setted");
	std::string pageNameDesc = std::string("name of page to retrieve. Required if test mode enabled");
	std::string calibrationPathDesc = std::string("calibration file path. Optional");

	// Usage
	string usageFirstLine =
			"\nUsage:\n"
					"TBDAnnotation calibrate [-h] [--output CALIBRATION_OUTPUT_PATH] \n"
					"TBDAnnotation register [-h] [--documents DOCUMENTS_PATH] [--output DATASET_OUTPUT_PATH] [--nSets N_SETS] [--nRetained N_RETAINED]\n"
					"TBDAnnotation annotate [-h] [--dataset DATASET_PATH] [--output PDF_OUTPUT_PATH] [--calibration CALIBRATION_FILE]\n"
					"TBDAnnotation test [-h] [--dataset DATASET_PATH] [--pageName PAGE_NAME] \n\n";
	po::options_description desc(usageFirstLine, 120);

	// Options
	desc.add_options()("help,h", helpDesc.c_str())("documents", po::value(&documentsPathString),
			documentsPathDesc.c_str())("output", po::value(&outputPathString), outputPathDesc.c_str())("nSets",
			po::value(&nSets), nSetsDesc.c_str())("nRetained", po::value(&nRetained), nRetainedDesc.c_str())("dataset",
			po::value(&datasetPathString), datasetPathDesc.c_str())("pageName", po::value(&pageNameString),
			pageNameDesc.c_str())("calibration", po::value(&calibrationPathString), calibrationPathDesc.c_str())(
			"subcommand", po::value<string>(&subcommand)->required(), subcommandDesc.c_str());

	po::positional_options_description positionalOptions;
	positionalOptions.add("subcommand", 1);

	po::variables_map vm;

	try {
		// Parse command line
		po::store(po::command_line_parser(argc, argv).options(desc).positional(positionalOptions).run(), vm);

		if (vm.count("help") || argc == 1) {
			std::cout << usageFirstLine;
			po::OptionPrinter::printParametersDesc(appName, std::cout, desc, &positionalOptions);
		}

		// Handle valid subcommands and required options
		vector<string> subcommands;
		subcommands.push_back("calibrate");
		subcommands.push_back("register");
		subcommands.push_back("annotate");
		subcommands.push_back("test");
		po::validate_subcommands(vm, "subcommand", subcommands);

		vector<string> subcommand_calibrate_dependencies;
		subcommand_calibrate_dependencies.push_back("output");

		vector<string> subcommand_register_dependencies;
		subcommand_register_dependencies.push_back("documents");
		subcommand_register_dependencies.push_back("output");

		vector<string> subcommand_annotate_dependencies;
		subcommand_annotate_dependencies.push_back("dataset");
		subcommand_annotate_dependencies.push_back("output");

		vector<string> subcommand_test_dependencies;
		subcommand_test_dependencies.push_back("dataset");
		subcommand_test_dependencies.push_back("pageName");

		po::subcommand_option_dependency(vm, "subcommand", "calibrate", subcommand_calibrate_dependencies);
		po::subcommand_option_dependency(vm, "subcommand", "register", subcommand_register_dependencies);
		po::subcommand_option_dependency(vm, "subcommand", "annotate", subcommand_annotate_dependencies);
		po::subcommand_option_dependency(vm, "subcommand", "test", subcommand_test_dependencies);

		po::notify(vm);

		if (nSets > 1) {
			po::option_dependency(vm, "nSets", "nRetained");
		}
		po::notify(vm);

		// nSets cannot be zero or negative
		if (nSets < 1) {
			throw std::logic_error(std::string("Option nSets cannot be zero or negative."));
		}

		// nRetained cannot be zero or negative
		if (nRetained < 1) {
			throw std::logic_error(std::string("Option nRetained cannot be zero or negative."));
		}

		// Check if calibration is supplied
		if (vm.count("calibration")) {
			calibrationSupplied = true;
		}
	} catch (std::exception& e) {
		std::cerr << e.what() << "\n";
		return 0;
	}

	/**********************************************************************************************************/
	/**********************************************************************************************************/

	if (vm["subcommand"].as<string>().compare("calibrate") == 0) {

		//##############################################################################//
		//							CALIBRATION
		//##############################################################################//

		string destinationPath(outputPathString);
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
				"In the same area, draw some black lines for text simulation.\n"
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

			ImageProcessor::adjustWebCamImage(frame, Parameters::SELECTED_WEB_CAM_ORIENTATION);

			Mat frameGray = frame.clone();
			cvtColor(frameGray, frameGray, CV_BGR2GRAY);

			// ############################################################# //
			// Truth definition section

			Mat adaptiveThresh;
			adaptiveThreshold(frameGray, adaptiveThresh, Parameters::THRESHOLD_MAX_VALUE, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
					CV_THRESH_BINARY, Parameters::ADAPTIVE_THRESHOLD_BLOCK_SIZE,
					Parameters::ADAPTIVE_THRESHOLD_CONSTANT);

			Mat element = getStructuringElement(Parameters::MORPH_SE_SHAPE, Size(3, 3), Point(1, 1));
			morphologyEx(adaptiveThresh, adaptiveThresh, MORPH_CLOSE, element);

			Mat truth = adaptiveThresh.clone();
			// Clean up
			for (int i = 0; i < truth.rows; i++) {
				for (int j = 0; j < truth.cols; j++) {
					if (!(i >= Parameters::BOUNDARY_MARGIN && i <= truth.rows / 2 && j >= Parameters::BOUNDARY_MARGIN
							&& j <= truth.cols - Parameters::BOUNDARY_MARGIN)) {
						truth.at<uchar>(i, j) = 255;
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
				blur(frame, blurred, Parameters::CALIBRATION_GAUSSIAN_BLUR_SIZE);

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
						ImageProcessor::bottomHat(channelsVector[i], channelsVector[i],
								Parameters::CALIBRATION_BOTTOM_HAT_MORPH_SE_SIZE);
						medianBlur(channelsVector[i], channelsVector[i], Parameters::CALIBRATION_MEDIAN_BLUR_SIZE);
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
				vector<Parameters::ThresholdStructure> srVector;

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
				vector<Parameters::ThresholdStructure> best_vector;
				double best_value = std::numeric_limits<double>::min();
				for (int k = 1; k <= Parameters::MAX_COMBINATION; k++) {
					double best_k_value = std::numeric_limits<double>::min();
					vector<Parameters::ThresholdStructure> best_k_vector;

					cp::NextCombinationGenerator<Parameters::ThresholdStructure> combinationGen(srVector, k);
					cp::NextCombinationGenerator<Parameters::ThresholdStructure>::iterator combIt;
					for (combIt = combinationGen.begin(); combIt != combinationGen.end(); combIt++) {
						// Vector with the current combination
						vector<Parameters::ThresholdStructure> combination(*combIt);

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

	} else if (vm["subcommand"].as<string>().compare("register") == 0) {

		//##############################################################################//
		//							REGISTRATION
		//##############################################################################//

		std::cout << "Register\n------------------------------------------" << "\n\n";

		path documentsPath(documentsPathString);
		path outputPath(outputPathString);

		// Number of sets
		std::cout << "> Number of sets:" << nSets << "\n\n";

		if (nSets == 1) {

			// Standard registration process for all the documents

			path registeredPdfMapPath = outputPath / Parameters::REGISTERED_PDF_MAP_PATH;
			path databasePath = outputPath / Parameters::DATABASE_PATH;
			path outputImagesPath = outputPath / Parameters::IMAGES_PATH;

			// Validates dataset output path
			std::cout << "> Dataset output path validation" << std::endl;
			FileManager::parentPathValidation(outputPath, outputImagesPath);
			std::cout << std::endl;

			// Loads the registeredPdfMap
			FileManager::loadRegisteredPdfMap(Parameters::registeredPdfMap, registeredPdfMapPath);

			std::cout << "> RegisteredPdfMap:" << "\n" << "\t" << "Path: " << registeredPdfMapPath << "\n" << "\t"
					<< "Size: " << Parameters::registeredPdfMap.size() << "\n\n";

			// Open a connection to the given database
			DatabaseHandler::openDatabase(databasePath);

			std::cout << "> Database:" << "\n" << "\t" << "Path: " << databasePath << "\n\n";

			// Gets a list of pdf if pdfPath is a directory of pdf, gets a list with one pdf if pdfPath is a pdf
			std::cout << "> Documents path:\t" << documentsPath << "\n\n";

			list<string> pdfList;
			FileManager::getPdfListFromPath(documentsPath, pdfList);

			std::cout << "> # Pdf: " << pdfList.size() << "\n";
			std::cout << "> Pdf List:\n";

			// RegisteredPdfMap update flag
			bool updateData = false;

			// Iterate for all pdf in pdfList
			list<string>::iterator pdfListIt;
			for (pdfListIt = pdfList.begin(); pdfListIt != pdfList.end(); pdfListIt++) {
				string pdfPath = *pdfListIt;
				string pdfFileName = path(pdfPath).filename().string();
				std::cout << "\t> " << pdfPath << "\n";

				// Checks if the pdf was already registered (fileName is the key)
				if (Parameters::registeredPdfMap.find(pdfFileName) == Parameters::registeredPdfMap.end()) {
					// Pdf isn't already registered (and converted)
					std::cout << "\t\tNot registered, conversion in progress..." << std::flush;

					stringstream jpgPath;
					jpgPath << outputImagesPath.string() << pdfFileName << ".jpg";

					// Conversion
					list<string> jpgList;
					int numPages = FileManager::createImagesFromPdf(pdfPath, jpgPath.str(), jpgList);
					if (numPages > 0) {
						updateData = true;

						std::cout << numPages << " pages converted\n";

						bool errorDuringRegistration = false;
						list<string>::iterator it = jpgList.begin();
						int n = 0;
						while (it != jpgList.end() && !errorDuringRegistration) {
							std::cout << "\t\tRegistration for " << *it << " (" << (n + 1) << "/" << jpgList.size()
									<< ")\n";

							// ##################################
							//  PAGE REGISTRATION
							// ##################################

							// Get image features (centroids)
							vector<Point> featurePoints;
							Mat page = cv::imread(*it, CV_LOAD_IMAGE_GRAYSCALE);
							ImageProcessor::getImageFeaturePoints(page, featurePoints);

							string pageId = path(*it).filename().string();
							FeaturesManager::registerFeatures(featurePoints, pageId);

							it++;
							n++;
						}

						if (!errorDuringRegistration) {
							std::pair<string, string> record(pdfFileName, pdfPath);

							// Updating the registeredPdfMap
							Parameters::registeredPdfMap.insert(record);
						}
					} else {
						std::cout << "an error occurred\n";
					}
				} else {
					std::cout << "\t\tAlready registered" << "\n";
				}
			}

			if (updateData) {
				std::cout << "\n" << "Saving RegisteredPdfMap..." << std::flush;
				// Saves the registeredPdfMap
				FileManager::saveRegisteredPdfMap(Parameters::registeredPdfMap, registeredPdfMapPath);
				std::cout << "DONE\n";
			}

			// Close database connection
			DatabaseHandler::closeDatabase();

#ifdef DEBUG_FEATURES_MANAGER_DISCRETIZATION_STATISTICS
			std::cout << "\n" << "> Max Affine invariant: " << Parameters::maxAffineInvariant << "\n";
			std::cout << "> Invalid affine invariants: " << Parameters::invalidAffineInvariants << "\n";
			std::cout << "> Affine invariant discretization statistics:" << "\n";
			for (unsigned int i = 0; i < Parameters::K; i++) {
				std::cout << "\t[" << i << "] = " << Parameters::DISCRETIZATION_STATISTICS[i] << "\n";
			}

			std::cout << "\n" << "> Sorting affine invariant vector (" << Parameters::affineInvariantVector.size()
			<< " elements)..." << std::flush;
			std::sort(Parameters::affineInvariantVector.begin(), Parameters::affineInvariantVector.end());
			std::cout << "DONE\n";

			std::cout << "> Optimal discretization vector ripartion (for these pdf):\n";
			int step = Parameters::affineInvariantVector.size() / Parameters::K;
			for (unsigned int i = 1; i < Parameters::K; i++) {
				std::cout << "\t[" << (i * step) << "] = " << Parameters::affineInvariantVector[i * step] << "\n";
			}
#endif

			std::cout << "\n" << "> Terminated" << "\n";

		} else {

			//##############################################################################//
			//							REGISTRATION (WITH N_SETS>1)
			//##############################################################################//

			// srand initialization
			unsigned int seed;

			// Choose a fixed rand SEED or a new SEED based on time function
			if (Parameters::USE_FIXED_RAND_SEED) {
				seed = Parameters::FIXED_RAND_SEED;
			} else {
				seed = time(0);
			}

			srand(seed);

			path outputAllImagesPath = outputPath / Parameters::IMAGES_PATH;
			path convertedPdfMapPath = outputPath / Parameters::CONVERTED_PDF_MAP_PATH;

			// Validates dataset output path
			std::cout << "> Datasets output path validation" << std::endl;
			FileManager::parentPathValidation(outputPath, outputAllImagesPath);
			std::cout << std::endl;

			// Loads the convertedPdfMap
			FileManager::loadConvertedPdfMap(Parameters::convertedPdfMap, convertedPdfMapPath);
			std::cout << "> ConvertedPdfMap:" << "\n" << "\t" << "Path: " << convertedPdfMapPath << "\n" << "\t"
					<< "Size: " << Parameters::convertedPdfMap.size() << "\n\n";

			// Gets a list of pdf if pdfPath is a directory of pdf, gets a list with one pdf if pdfPath is a pdf
			std::cout << "> Documents path:\t" << documentsPath << "\n\n";

			list<string> pdfList;
			FileManager::getPdfListFromPath(documentsPath, pdfList);

			std::cout << "> # Pdf: " << pdfList.size() << "\n";
			std::cout << "> Pdf List:\n";

			// Iterate for all pdf in pdfList and converts if not already converted
			list<string>::iterator pdfListIt;
			int pdfCounter = 1;
			uint numPages = 0;

			for (pdfListIt = pdfList.begin(); pdfListIt != pdfList.end(); pdfListIt++) {
				string pdfPath = *pdfListIt;
				string pdfFileName = path(pdfPath).filename().string();

				// Checks if the pdf was already registered (fileName is the key)

				typedef map<string, pair<string, int>>::const_iterator const_iterator;
				const_iterator it = Parameters::convertedPdfMap.find(pdfFileName);
				if (it == Parameters::convertedPdfMap.end()) {
					// Pdf isn't already converted
					std::cout << "\t> " << pdfPath << " conversion in progress (" << pdfCounter << "/" << pdfList.size()
							<< ")\n";

					stringstream jpgPath;
					jpgPath << outputAllImagesPath.string() << pdfFileName << ".jpg";

					// Conversion
					list<string> jpgList;
					int pages = FileManager::createImagesFromPdf(pdfPath, jpgPath.str(), jpgList);
					numPages += pages;

					if (pages > 0) {
						// Update convertedPdfMap
						std::pair<string, pair<string, int>> record(pdfFileName, make_pair(pdfPath, pages));
						Parameters::convertedPdfMap.insert(record);
					}
				} else {
					numPages += (it->second).second;

					std::cout << "\t> " << pdfPath << " already converted (" << pdfCounter << "/" << pdfList.size()
							<< ")\n";
				}

				pdfCounter++;
			}

			// Save convertedPdfMap
			std::cout << "\n" << "> Saving convertedPdfMap..." << "\n";
			FileManager::saveConvertedPdfMap(Parameters::convertedPdfMap, convertedPdfMapPath);

			std::cout << "\n" << "> # Total pages: " << numPages << "\n\n";

			try {
				// Check if there are same pages converted but not in the converted list
				vector<string> jpgVector;
				FileManager::getJpgVectorFromPath(outputAllImagesPath, jpgVector);
				if (numPages != jpgVector.size()) {
					throw std::logic_error(
							std::string(
									"Number of pages in converted folder is inconsistent with the converted jpg list. Same pages may have been moved away from folder"));
				}

				// Check if pages converted are more then the nSets*nRetained
				if (numPages < nSets * nRetained) {
					throw std::logic_error(
							std::string("Converted pages cannot be less than nSets*nRetained (")
									+ boost::lexical_cast<string>(nSets * nRetained) + std::string(")."));
				}

				// Check if retained folder already exists
				path outputRetainedImagesPath = outputPath / Parameters::RETAINED_IMAGES_PATH;
				std::cout << "> Retained images path validation" << std::endl;
				if (exists(outputRetainedImagesPath)) {
					throw std::logic_error(
							std::string("Retained images folder already exists. Please delete the folder."));
				}
				FileManager::parentPathValidation(outputPath, outputRetainedImagesPath);

				// Check if some folder for sets already exists
				int nIncrement = numPages / nSets;
				std::cout << "\n> Sets output path validation" << std::endl << std::endl;
				for (int i = 0; i < nSets; i++) {
					int setSize = i * nIncrement + nIncrement;
					path datasetPath = outputPath
							/ std::string(Parameters::DATASET_PREFIX + boost::lexical_cast<string>(setSize));
					if (exists(datasetPath)) {
						throw std::logic_error(
								std::string(datasetPath.string() + " already exists. Please delete the folder."));
					}
				}

				// Choose nRetained elements and save to retained folder
				vector<int> retainedIndexes;
				vector<string> retainedImagesPath;
				Utils::selectRandomDistinctIndexes(jpgVector.size(), retainedIndexes, nRetained);
				sort(retainedIndexes.begin(), retainedIndexes.end(), greater<int>());
				for (unsigned i = 0; i < retainedIndexes.size(); i++) {
					int index = retainedIndexes.at(i);
					string retainedJpgPath = jpgVector.at(index);
					std::cout << "> Saving retained JPG #" << i + 1 << ": " << retainedJpgPath << "\n";
					path imagePathFrom = retainedJpgPath;
					path imagePathTo = outputRetainedImagesPath / path(retainedJpgPath).filename();
					copy_file(imagePathFrom, imagePathTo, copy_option::overwrite_if_exists);
					jpgVector.erase(jpgVector.begin() + index);
					retainedImagesPath.push_back(retainedJpgPath);
				}

				// Create and register each set
				for (int i = 0; i < nSets; i++) {

#ifdef DEBUG_FEATURES_MANAGER_DISCRETIZATION_STATISTICS
					// Clear statistics vector
					Parameters::affineInvariantVector.clear();
					Parameters::maxAffineInvariant = 0;
					Parameters::invalidAffineInvariants = 0;
#endif

					int setSize = i * nIncrement + nIncrement;
					std::cout << "\n> Registering set #" << i + 1 << " (" << setSize << " elements)" << std::endl;

					// Create folders
					path datasetPath = outputPath
							/ std::string(Parameters::DATASET_PREFIX + boost::lexical_cast<string>(setSize));
					path databasePath = datasetPath / Parameters::DATABASE_PATH;
					path outputImagesPath = datasetPath / Parameters::IMAGES_PATH;

					// Validates dataset output paths
					std::cout << "> Dataset output paths validation" << std::endl;
					FileManager::parentPathValidation(outputPath, datasetPath);
					FileManager::parentPathValidation(datasetPath, outputImagesPath);

					// Open a connection to the given database
					DatabaseHandler::openDatabase(databasePath);

					// Create vector of pages
					int nToChoose = setSize - nRetained;
					list<string> datasetImages;
					for (unsigned j = 0; j < retainedImagesPath.size(); j++) {
						path imagePathFrom = retainedImagesPath[j];
						path imagePathTo = outputImagesPath / path(retainedImagesPath[j]).filename();
						copy_file(imagePathFrom, imagePathTo, copy_option::overwrite_if_exists);
						datasetImages.push_back(imagePathTo.string());
					}
					vector<int> choosenIndexes;
					Utils::selectRandomDistinctIndexes(jpgVector.size(), choosenIndexes, nToChoose);
					for (unsigned k = 0; k < choosenIndexes.size(); k++) {
						int index = choosenIndexes.at(k);
						string jpgPath = jpgVector.at(index);
						path imagePathFrom = jpgPath;
						path imagePathTo = outputImagesPath / path(jpgPath).filename();
						copy_file(imagePathFrom, imagePathTo, copy_option::overwrite_if_exists);
						datasetImages.push_back(imagePathTo.string());
					}

					// Register
					bool errorDuringRegistration = false;
					list<string>::iterator it = datasetImages.begin();
					int n = 0;
					while (it != datasetImages.end() && !errorDuringRegistration) {
						std::cout << "\tRegistration for " << *it << " (" << (n + 1) << "/" << datasetImages.size()
								<< ")\n";

						// ##################################
						//  PAGE REGISTRATION
						// ##################################

						// Get image features (centroids)
						vector<Point> featurePoints;
						Mat page = cv::imread(*it, CV_LOAD_IMAGE_GRAYSCALE);
						ImageProcessor::getImageFeaturePoints(page, featurePoints);

						string pageId = path(*it).filename().string();
						FeaturesManager::registerFeatures(featurePoints, pageId);

						it++;
						n++;
					}

					// Close database connection
					DatabaseHandler::closeDatabase();

#ifdef DEBUG_FEATURES_MANAGER_DISCRETIZATION_STATISTICS
					std::cout << "\n" << "> Max Affine invariant: " << Parameters::maxAffineInvariant << "\n";
					std::cout << "> Invalid affine invariants: " << Parameters::invalidAffineInvariants << "\n";
					std::cout << "> Affine invariant discretization statistics:" << "\n";
					for (unsigned int i = 0; i < Parameters::K; i++) {
						std::cout << "\t[" << i << "] = " << Parameters::DISCRETIZATION_STATISTICS[i] << "\n";
					}

					std::cout << "\n" << "> Sorting affine invariant vector ("
					<< Parameters::affineInvariantVector.size() << " elements)..." << std::flush;
					std::sort(Parameters::affineInvariantVector.begin(), Parameters::affineInvariantVector.end());
					std::cout << "DONE\n";

					std::cout << "> Optimal discretization vector ripartion (for these pdf):\n";
					int step = Parameters::affineInvariantVector.size() / Parameters::K;
					for (unsigned int i = 1; i < Parameters::K; i++) {
						std::cout << "\t[" << (i * step) << "] = " << Parameters::affineInvariantVector[i * step]
						<< "\n";
					}
#endif

				}

			} catch (std::exception& e) {
				std::cerr << e.what() << "\n";
				return 0;
			}
		}
	} else if (vm["subcommand"].as<string>().compare("annotate") == 0) {

		XInitThreads();

		//##############################################################################//
		//							ANNOTATION
		//##############################################################################//

		std::cout << "\nAnnotate\n------------------------------------------" << "\n";

		path datasetPath(datasetPathString);
		path pdfOutputPath(outputPathString);

		path registeredPdfMapPath = datasetPath / Parameters::REGISTERED_PDF_MAP_PATH;
		path databasePath = datasetPath / Parameters::DATABASE_PATH;
		path masksPath = datasetPath / Parameters::MASKS_PATH;
		path annotationsPath = datasetPath / Parameters::ANNOTATIONS_PATH;

		// Validates masks path
		std::cout << "> Masks path validation" << std::endl;
		FileManager::parentPathValidation(datasetPath, masksPath);
		std::cout << std::endl;

		// Validates annotations path
		std::cout << "> Annotations path validation" << std::endl;
		FileManager::parentPathValidation(datasetPath, annotationsPath);
		std::cout << std::endl;

		// Validates pdf output path
		std::cout << "> Pdf output path validation" << std::endl;
		FileManager::pathValidation(pdfOutputPath);
		std::cout << std::endl;

		// Calibration file handle
		if (calibrationSupplied) {
			Parameters::ANNOTATION_COLOR_MODE = Parameters::ANNOTATION_COLOR_THRESHOLD_FILE_MODE;
		} else {
			Parameters::ANNOTATION_COLOR_MODE = Parameters::ANNOTATION_COLOR_FIXED_MODE;
		}

		if (Parameters::ANNOTATION_COLOR_MODE == Parameters::ANNOTATION_COLOR_THRESHOLD_FILE_MODE) {
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

			// Loads calibrated annotation thresholds
			FileManager::loadAnnotationThresholds(calibrationPathString);

			std::cout << "> Loaded calibrated annotation thresholds" << std::endl;
			cout << "> Channels:\n";
			for (unsigned int ch = 0; ch < Parameters::annotationThresholds.size(); ch++) {
				cout << "\t" << channelLabels[Parameters::annotationThresholds[ch].channel] << "\t";
				cout << "[" << Parameters::annotationThresholds[ch].thresh.first << ", "
						<< Parameters::annotationThresholds[ch].thresh.second << "]\n";
			}
		} else {
			std::cout << "> Fixed annotation thresholds (light blue)" << std::endl;
		}
		std::cout << std::endl;

		// Loads the registeredPdfMap
		FileManager::loadRegisteredPdfMap(Parameters::registeredPdfMap, registeredPdfMapPath);

		std::cout << "> RegisteredPdfMap:" << "\n" << "\t" << "Path: " << registeredPdfMapPath << "\n" << "\t"
				<< "Size: " << Parameters::registeredPdfMap.size() << "\n\n";

		// Open a connection to the given database
		DatabaseHandler::openDatabase(databasePath);

		std::cout << "> Database:" << "\n" << "\t" << "Path: " << databasePath << "\n\n";

		/*
		 * Inizializzazione macchina
		 * ####################################################################################
		 */
		StateMachine* sm = new StateMachine();

		// Inizializzazione stati
		PageDiscovering* pageDiscoveringState = new PageDiscovering(sm);
		PageRetrieval* pageRetrievalState = new PageRetrieval(sm);
		PageTracking* pageTrackingState = new PageTracking(sm);
		PageError* pageErrorState = new PageError(sm);
		PageAnnotation* pageAnnotationState = new PageAnnotation(sm);
		PdfCreation* pdfCreationState = new PdfCreation(sm);
		ExitState* exitState = new ExitState(sm);

		// Inizializzazione stati iniziali e finali
		sm->setInitialState(pageDiscoveringState);
		sm->setFinalState(exitState);

		// Inizializzazione transizioni
		sm->addTransition(pageDiscoveringState, pageDiscoveringState, EvPageDiscoveringFail());
		sm->addTransition(pageDiscoveringState, pageRetrievalState, EvPageDiscoveringSuccess());
		sm->addTransition(pageRetrievalState, pageTrackingState, EvPageRetrievalDone());
		sm->addTransition(pageRetrievalState, pageErrorState, EvPageRetrievalError());
		sm->addTransition(pageTrackingState, pageTrackingState, EvPageTrackingSuccess());
		sm->addTransition(pageTrackingState, pageAnnotationState, EvPageAnnotationStart());
		sm->addTransition(pageTrackingState, pageErrorState, EvPageTrackingFailGenericError());
		sm->addTransition(pageTrackingState, pageErrorState, EvPageTrackingFailHandOcclusion());
		sm->addTransition(pageTrackingState, pageErrorState, EvPageTrackingAnnotationSaveError());
		sm->addTransition(pageTrackingState, pdfCreationState, EvPdfCreationRequest());
		sm->addTransition(pageAnnotationState, pageTrackingState, EvPageAnnotationEnd());
		sm->addTransition(pdfCreationState, pageTrackingState, EvPdfCreationDone());
		sm->addTransition(pageErrorState, pageDiscoveringState, EvErrorReset());

		// Exit transitions
		sm->addTransition(pageDiscoveringState, exitState, EvExit());
		sm->addTransition(pageRetrievalState, exitState, EvExit());
		sm->addTransition(pageTrackingState, exitState, EvExit());
		sm->addTransition(pageAnnotationState, exitState, EvExit());
		sm->addTransition(pdfCreationState, exitState, EvExit());
		sm->addTransition(pageErrorState, exitState, EvExit());

		// Set dataset path and pdf output path
		sm->setDatasetPath(datasetPath);
		sm->setPdfOutputPath(pdfOutputPath);

		sm->initiate();

		// Deleting objects
		delete exitState;
		delete pageTrackingState;
		delete pageRetrievalState;
		delete pageDiscoveringState;
		delete pageErrorState;
		delete pageAnnotationState;
		delete pdfCreationState;

		delete sm;

		// Close database connection
		DatabaseHandler::closeDatabase();

		std::cout << "\n" << "> Terminated" << "\n";
	} else if (vm["subcommand"].as<string>().compare("test") == 0) {

		XInitThreads();

		//##############################################################################//
		//							TEST
		//##############################################################################//

#ifndef DEBUG_RETRIEVE_PAGE_PRINT_FINAL_STATISTICS
#define DEBUG_RETRIEVE_PAGE_PRINT_FINAL_STATISTICS
#endif

		std::cout << "\nTest\n------------------------------------------" << "\n";

		path datasetPath(datasetPathString);
		path databasePath = datasetPath / Parameters::DATABASE_PATH;

		// Open a connection to the given database
		DatabaseHandler::openDatabase(databasePath);

		std::cout << "> Database:" << "\n" << "\t" << "Path: " << databasePath << "\n\n";

		/*
		 * Inizializzazione macchina
		 * ####################################################################################
		 */
		StateMachine* sm = new StateMachine();

		// Inizializzazione stati
		PageDiscovering* pageDiscoveringState = new PageDiscovering(sm);
		PageRetrieval* pageRetrievalState = new PageRetrieval(sm);
		ExitState* exitState = new ExitState(sm);

		// Inizializzazione variabili di test
		sm->setTest(true);
		sm->setPageName(pageNameString);

		// Inizializzazione stati iniziali e finali
		sm->setInitialState(pageDiscoveringState);
		sm->setFinalState(exitState);

		// Inizializzazione transizioni
		sm->addTransition(pageDiscoveringState, pageDiscoveringState, EvPageDiscoveringFail());
		sm->addTransition(pageDiscoveringState, pageRetrievalState, EvPageDiscoveringSuccess());
		sm->addTransition(pageRetrievalState, exitState, EvPageRetrievalDone());

		// Exit transitions
		sm->addTransition(pageDiscoveringState, exitState, EvExit());
		sm->addTransition(pageRetrievalState, exitState, EvExit());

		// Set dataset path and pdf output path
		sm->setDatasetPath(datasetPath);

		sm->initiate();

		// Deleting objects
		delete exitState;
		delete pageRetrievalState;
		delete pageDiscoveringState;

		delete sm;

		// Close database connection
		DatabaseHandler::closeDatabase();

		std::cout << "\n" << "> Terminated" << "\n";
	}

	return 0;
}
