/*
 * MachineEvents.hpp
 *
 *  Created on: 22/apr/2013
 *      Author: marco
 */

#ifndef MACHINEEVENTS_HPP_
#define MACHINEEVENTS_HPP_

#include "SimpleStateMachine.hpp"

using namespace ssm;

const int EVENT_PAGE_DISCOVERING_FAIL = 0;
const int EVENT_PAGE_DISCOVERING_SUCCESS = 1;
const int EVENT_PAGE_RETRIEVAL_DONE = 2;
const int EVENT_PAGE_RETRIEVAL_ERROR = 3;
const int EVENT_PAGE_TRACKING_FAIL_HAND_OCCLUSION = 4;
const int EVENT_PAGE_TRACKING_FAIL_GENERIC_ERROR = 5;
const int EVENT_PAGE_TRACKING_SUCCESS = 6;
const int EVENT_ERROR_RESET = 7;
const int EVENT_PAGE_ANNOTATION_START = 8;
const int EVENT_PAGE_ANNOTATION_END = 9;
const int EVENT_PAGE_TRACKING_ANNOTATION_SAVE_ERROR = 10;
const int EVENT_PDF_CREATION_REQUEST = 11;
const int EVENT_PDF_CREATION_DONE = 12;
const int EVENT_EXIT = 99;

struct EvPageDiscoveringFail: public SimpleEvent {
	EvPageDiscoveringFail() :
			SimpleEvent(EVENT_PAGE_DISCOVERING_FAIL) {
	}
};

struct EvPageDiscoveringSuccess: public SimpleEvent {
	EvPageDiscoveringSuccess() :
			SimpleEvent(EVENT_PAGE_DISCOVERING_SUCCESS) {
	}
};

struct EvPageRetrievalDone: public SimpleEvent {
	EvPageRetrievalDone() :
			SimpleEvent(EVENT_PAGE_RETRIEVAL_DONE) {
	}
};

struct EvPageRetrievalError: public SimpleEvent {
	EvPageRetrievalError() :
			SimpleEvent(EVENT_PAGE_RETRIEVAL_ERROR) {
	}
};

struct EvPageTrackingFailHandOcclusion: public SimpleEvent {
	EvPageTrackingFailHandOcclusion() :
			SimpleEvent(EVENT_PAGE_TRACKING_FAIL_HAND_OCCLUSION) {
	}
};

struct EvPageTrackingFailGenericError: public SimpleEvent {
	EvPageTrackingFailGenericError() :
			SimpleEvent(EVENT_PAGE_TRACKING_FAIL_GENERIC_ERROR) {
	}
};

struct EvPageTrackingSuccess: public SimpleEvent {
	EvPageTrackingSuccess() :
			SimpleEvent(EVENT_PAGE_TRACKING_SUCCESS) {
	}
};

struct EvErrorReset: public SimpleEvent {
	EvErrorReset() :
			SimpleEvent(EVENT_ERROR_RESET) {
	}
};

struct EvPageAnnotationStart: public SimpleEvent {
	EvPageAnnotationStart() :
			SimpleEvent(EVENT_PAGE_ANNOTATION_START) {
	}
};

struct EvPageAnnotationEnd: public SimpleEvent {
	EvPageAnnotationEnd() :
			SimpleEvent(EVENT_PAGE_ANNOTATION_END) {
	}
};

struct EvPageTrackingAnnotationSaveError: public SimpleEvent {
	EvPageTrackingAnnotationSaveError() :
			SimpleEvent(EVENT_PAGE_TRACKING_ANNOTATION_SAVE_ERROR) {
	}
};

struct EvPdfCreationRequest: public SimpleEvent {
	EvPdfCreationRequest() :
			SimpleEvent(EVENT_PDF_CREATION_REQUEST) {
	}
};

struct EvPdfCreationDone: public SimpleEvent {
	EvPdfCreationDone() :
			SimpleEvent(EVENT_PDF_CREATION_DONE) {
	}
};

struct EvExit: public SimpleEvent {
	EvExit() :
			SimpleEvent(EVENT_EXIT) {
	}
};

#endif /* MACHINEEVENTS_HPP_ */
