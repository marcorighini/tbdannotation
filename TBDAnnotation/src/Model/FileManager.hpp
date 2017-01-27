/*
 * FileManager.hpp
 *
 *  Created on: 20/apr/2013
 *      Author: alessandro
 */

#ifndef FILEMANAGER_HPP_
#define FILEMANAGER_HPP_

#include "DatabaseHandler.hpp"
#include "Parameters.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <fstream>
#include <map>

using namespace std;
using namespace boost::filesystem;
using namespace boost;

/*
 * Class FileManager
 */
class FileManager {
public:
	typedef boost::filesystem::path path;

	/*
	 * Saves the registered pdf map
	 */
	static void saveRegisteredPdfMap(const RegisteredPdfMap& registeredPdfMap, const path& registeredPdfMapPath) {
		std::ofstream ofs(registeredPdfMapPath.c_str());
		if (ofs.is_open()) {
			archive::binary_oarchive oa(ofs);

			// Saves the map
			oa << registeredPdfMap;
		} else {
			throw std::logic_error(string("ERROR: Failed to open file \"") + registeredPdfMapPath.string());
		}
	}

	/*
	 * Loads the registered pdf map
	 */
	static void loadRegisteredPdfMap(RegisteredPdfMap& registeredPdfMap, const path& registeredPdfMapPath) {
		if (exists(registeredPdfMapPath)) {
			std::ifstream ifs(registeredPdfMapPath.c_str(), std::ios::binary);
			archive::binary_iarchive ia(ifs);

			// Loads the map
			ia >> registeredPdfMap;
		}
	}

	/*
	 * Saves the converted pdf map
	 */
	static void saveConvertedPdfMap(const ConvertedPdfMap& convertedPdfMap, const path& convertedPdfMapPath) {
		std::ofstream ofs(convertedPdfMapPath.c_str());
		if (ofs.is_open()) {
			archive::binary_oarchive oa(ofs);

			// Saves the map
			oa << convertedPdfMap;
		} else {
			throw std::logic_error(string("ERROR: Failed to open file \"") + convertedPdfMapPath.string());
		}
	}

	/*
	 * Loads the converted pdf map
	 */
	static void loadConvertedPdfMap(ConvertedPdfMap& convertedPdfMap, const path& convertedPdfMapPath) {
		if (exists(convertedPdfMapPath)) {
			std::ifstream ifs(convertedPdfMapPath.c_str(), std::ios::binary);
			archive::binary_iarchive ia(ifs);

			// Loads the map
			ia >> convertedPdfMap;
		}
	}

	/*
	 * Checks if filePath is a pdf
	 */
	static bool isPdf(const path& filePath) {
		string extension = filePath.extension().string();
		to_lower(extension);
		if (is_regular_file(filePath) && (extension == ".pdf" || extension == ".PDF")) {
			return true;
		} else
			return false;
	}

	/*
	 * Checks if filePath is a jpg
	 */
	static bool isJpg(const path& filePath) {
		string extension = filePath.extension().string();
		to_lower(extension);
		if (is_regular_file(filePath) && (extension == ".jpg" || extension == ".JPG")) {
			return true;
		} else
			return false;
	}

	/*
	 * Validates parentPath and creates subdirectories
	 */
	static void parentPathValidation(const path& parentPath, const path& childPath) {
		if (!exists(parentPath)) {
			if (!create_directory(parentPath)) {
				throw std::logic_error(
						string("ERROR: can't create directory \"") + parentPath.string() + string(
								"\". Permission denied"));
			} else {
				std::cout << "\tCreated directory \"" << parentPath.string() << "\"\n";
			}
		}

		if (!exists(childPath)) {
			if (!create_directory(childPath)) {
				throw std::logic_error(
						string("ERROR: can't create directory \"") + childPath.string() + string(
								"\". Permission denied"));
			} else {
				std::cout << "\tCreated directory \"" << childPath.string() << "\"\n";
			}
		}
	}

	/*
	 * Validates path and creates directory
	 */
	static void pathValidation(const path& path) {
		if (!exists(path)) {
			if (!create_directory(path)) {
				throw std::logic_error(
						string("ERROR: can't create directory \"") + path.string() + string("\". Permission denied"));
			} else {
				std::cout << "\tCreated directory \"" << path.string() << "\"\n";
			}
		}
	}

	/*
	 * Creates a list of pdf from pdfPath
	 */
	static void getPdfListFromPath(const path& pdfPath, list<string>& pdfList) {
		if (exists(pdfPath)) {
			if (is_directory(pdfPath)) {
				directory_iterator endItr;
				for (directory_iterator beginItr(pdfPath); beginItr != endItr; ++beginItr) {
					if (FileManager::isPdf(beginItr->path())) {
						std::string filePath = (absolute(beginItr->path())).string();
						pdfList.push_back(filePath);
					}
				}
			} else if (FileManager::isPdf(pdfPath)) {
				std::string filePath = pdfPath.string();
				pdfList.push_back(filePath);
			} else {
				throw std::logic_error(
						string("ERROR: \"") + pdfPath.string() + string("\" is not a directory or a PDF"));
			}
		} else {
			throw std::logic_error(string("ERROR: path \"") + pdfPath.string() + string("\" does not exists"));
		}
	}

	/*
	 * Creates a list of jpg from jpgPath
	 */
	static void getJpgVectorFromPath(const path& jpgPath, vector<string>& jpgList) {
		if (exists(jpgPath)) {
			if (is_directory(jpgPath)) {
				directory_iterator endItr;
				for (directory_iterator beginItr(jpgPath); beginItr != endItr; ++beginItr) {
					if (FileManager::isJpg(beginItr->path())) {
						std::string filePath = (absolute(beginItr->path())).string();
						jpgList.push_back(filePath);
					}
				}
			} else if (FileManager::isJpg(jpgPath)) {
				std::string filePath = jpgPath.string();
				jpgList.push_back(filePath);
			} else {
				throw std::logic_error(
						string("ERROR: \"") + jpgPath.string() + string("\" is not a directory or a JPG"));
			}
		} else {
			throw std::logic_error(string("ERROR: path \"") + jpgPath.string() + string("\" does not exists"));
		}

		sort(jpgList.begin(), jpgList.end());
	}

	/*
	 * Calls system for a conversion from pdf to a list of jpg images
	 */
	static int createImagesFromPdf(const string pdfPath, const string jpgPath, list<string>& jpgList) {
		int numPages = 0;

		// Utils
		string jpgFileName = path(jpgPath).stem().string(); // Without extension
		string jpgExtension = path(jpgPath).extension().string();
		string jpgParentPath = path(jpgPath).parent_path().string();

		// Pdf info command definition
		stringstream pagesCommand;
		pagesCommand << "pdfinfo " << pdfPath << " | grep Pages: | awk '{print $2}'";
		string verbose = Utils::executeAndGetVerbose(pagesCommand.str());

		try {
			// Erasing line feed and carriage return
			verbose.erase(remove(verbose.begin(), verbose.end(), '\n'), verbose.end());
			verbose.erase(remove(verbose.begin(), verbose.end(), '\r'), verbose.end());

			numPages = boost::lexical_cast<int>(verbose);
		} catch (boost::bad_lexical_cast const&) {
			std::cout << "ERROR: input string \"" << verbose << "\" was not valid";
		}

		// Checks if the pdf was already converted
		bool alreadyConverted = true;
		int i = 0;
		while (i < numPages && alreadyConverted != false) {
			stringstream convertedJpgFilePath;
			convertedJpgFilePath << jpgParentPath << "/" << jpgFileName << "-" << i << jpgExtension;

			if (!exists(convertedJpgFilePath.str())) {
				alreadyConverted = false;
			}
			i++;
		}

		if (!alreadyConverted) {
			// Conversion command definition
			stringstream conversionCommand;
			conversionCommand << "convert -colorspace RGB -interlace none ";
			conversionCommand << "-density " << Parameters::CONVERT_DENSITY << "x" << Parameters::CONVERT_DENSITY
					<< " ";
			conversionCommand << "-quality " << Parameters::CONVERT_QUALITY << " ";

			conversionCommand << pdfPath << " " << jpgPath;

			int errorCode = std::system(conversionCommand.str().c_str());

			// If the conversion ends successfully, creates list of jpg pages
			if (errorCode != 0) {
				numPages = 0;
			}
		}

		// Creating the list of converted jpg
		for (int i = 0; i < numPages; i++) {
			stringstream jpgFilePath;
			jpgFilePath << jpgParentPath << "/" << jpgFileName << "-" << i << jpgExtension;

			// Final check and push
			if (exists(path(jpgFilePath.str()))) {
				jpgList.push_back(jpgFilePath.str());
			}
		}

		return numPages;
	}

	/*
	 * Calls system for a conversion from a list of jpg images to a pdf
	 */
	static int createPdfFromImages(const path& imagesPrefixPath, const path& pdfPath) {
		// Conversion command definition
		stringstream conversionCommand;
		conversionCommand << "convert ";
		conversionCommand << imagesPrefixPath.string() << "*" << ".jpg" << " " << pdfPath.string();

		int errorCode = std::system(conversionCommand.str().c_str());

		return errorCode;
	}

	/*
	 * Loads calibrated annotation thresholds from a file
	 */
	static void loadAnnotationThresholds(const string calibrationFilePath) {
		FileStorage fs(calibrationFilePath, FileStorage::READ);

		if (fs.isOpened()) {
			int numberOfChannels;
			fs["number"] >> numberOfChannels;

			for (int ch = 0; ch < numberOfChannels; ch++) {
				Parameters::ThresholdStructure ts;

				stringstream ss;
				ss << "thresh" << "_" << ch;

				fs[ss.str()]["channel_id"] >> ts.channel;
				fs[ss.str()]["thresh"]["first"] >> ts.thresh.first;
				fs[ss.str()]["thresh"]["second"] >> ts.thresh.second;

				Parameters::annotationThresholds.push_back(ts);
			}
		} else {
			throw std::logic_error(string("ERROR: Can't open thresholds file \"") + calibrationFilePath);
		}
	}
};

#endif /* FILEMANAGER_HPP_ */
