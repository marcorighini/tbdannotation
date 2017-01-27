/*
 * ExitState.hpp
 *
 *  Created on: 22/apr/2013
 *      Author: marco
 */

#ifndef EXITSTATE_HPP_
#define EXITSTATE_HPP_

#include <iostream>
#include "SimpleStateMachine.hpp"
#include "../WebCamCapture.hpp"
#include "MachineEvents.hpp"

using namespace cv;
using namespace std;
using namespace ssm;

struct ExitState: public SimpleState {
	ExitState(SimpleStateMachine* _context) :
		SimpleState(_context) {
	}

	virtual void doAction() {
		WebCamCapture::release();
	}
};

#endif /* EXITSTATE_HPP_ */
