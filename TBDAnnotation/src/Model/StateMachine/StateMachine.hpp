/*
 * StateMachine.hpp
 *
 *  Created on: 22/apr/2013
 *      Author: marco
 */

#ifndef STATEMACHINE_HPP_
#define STATEMACHINE_HPP_

#include "SimpleStateMachine.hpp"

using namespace ssm;
using namespace cv;

class StateMachine: public SimpleStateMachine {
protected:
	path datasetPath;
	bool test;
	string pageName;
	path pdfOutputPath;
	path retrievedImagePath;
	path retrievedOriginalDocumentPath;
	Mat discoverToRetrievalImage;
	Mat retrievalToTrackingImage;
	Mat maskImage;
	path maskImagePath;
	path annotationsImagePath;
	string pageErrorWhat;
	bool saveAnnotationRequest;
public:
	StateMachine() :
		saveAnnotationRequest(false) {
	}

	void setDatasetPath(const path& src) {
		datasetPath = src;
	}

	path getDatasetPath() {
		return datasetPath;
	}

	void setRetrievedImagePath(const path& src){
		retrievedImagePath = src;
	}

	path getRetrievedImagePath() const {
		return retrievedImagePath;
	}

	void setRetrievedOriginalDocumentPath(const path& src){
		retrievedOriginalDocumentPath = src;
	}

	path getRetrievedOriginalDocumentPath() const {
		return retrievedOriginalDocumentPath;
	}

	void setDiscoverToRetrievalImage(const Mat& src) {
		discoverToRetrievalImage = src.clone();
	}

	Mat getDiscoverToRetrieval() const {
		return discoverToRetrievalImage.clone();
	}

	void setRetrievalToTrackingImage(const Mat& src) {
		retrievalToTrackingImage = src.clone();
	}

	Mat getRetrievalToTrackingImage() const {
		return retrievalToTrackingImage.clone();
	}

	void setMaskImage(const Mat& src) {
		maskImage = src.clone();
	}

	Mat getMaskImage() const {
		return maskImage.clone();
	}

	void setPageErrorWhat(const string what) {
		pageErrorWhat = what;
	}

	string getPageErrorWhat() const {
		return pageErrorWhat;
	}

	bool isSaveAnnotationRequest() const {
		return saveAnnotationRequest;
	}

	void setSaveAnnotationRequest(bool saveAnnotationRequest) {
		this->saveAnnotationRequest = saveAnnotationRequest;
	}

	path getMaskImagePath() const {
		return maskImagePath;
	}

	void setMaskImagePath(path maskImagePath) {
		this->maskImagePath = maskImagePath;
	}

	virtual ~StateMachine() {
	}

	const path& getAnnotationsImagePath() const {
		return annotationsImagePath;
	}

	void setAnnotationsImagePath(const path& annotationsImagePath) {
		this->annotationsImagePath = annotationsImagePath;
	}

	path getPdfOutputPath() const {
		return pdfOutputPath;
	}

	void setPdfOutputPath(path pdfOutputPath) {
		this->pdfOutputPath = pdfOutputPath;
	}

	bool isTest() const {
		return test;
	}

	void setTest(bool isTest) {
		this->test = isTest;
	}

	const string& getPageName() const {
		return pageName;
	}

	void setPageName(const string& pageName) {
		this->pageName = pageName;
	}
};

#endif /* STATEMACHINE_HPP_ */
