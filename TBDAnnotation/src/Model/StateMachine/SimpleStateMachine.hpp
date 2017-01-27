/*
 * SimpleStateMachine.hpp
 *
 *  Created on: 22/apr/2013
 *      Author: marco
 */

#ifndef SIMPLESTATEMACHINE_HPP_
#define SIMPLESTATEMACHINE_HPP_

#include <iostream>
#include <map>

using namespace std;

namespace ssm {
class SimpleStateMachine;

struct SimpleState {
	SimpleStateMachine* context;

	SimpleState() :
		context(NULL) {
	}

	SimpleState(SimpleStateMachine* _context) :
		context(_context) {
	}

	virtual ~SimpleState(){
	}

	virtual void doAction() = 0;
};

struct SimpleEvent {
	int eventId;

	SimpleEvent(int _eventId) :
		eventId(_eventId) {
	}
};

class SimpleStateMachine {
	typedef map<int, map<SimpleState*, SimpleState*> > TransitionMapType;

	SimpleState* initialState;
	SimpleState* finalState;

	SimpleState* currentState;

	TransitionMapType transitionMap;

	void execute() {
		while (currentState != finalState) {
			currentState->doAction();
		}
		finalState->doAction();
	}

public:
	SimpleStateMachine() :
		initialState(NULL), finalState(NULL), currentState(NULL) {
	}

	SimpleStateMachine(SimpleState* _initialState, SimpleState* _finalState) :
		initialState(_initialState), finalState(_finalState) {
		currentState = NULL;
	}

	void setInitialState(SimpleState* _initialState) {
		if (_initialState != NULL) {
			initialState = _initialState;
		}
	}

	void setFinalState(SimpleState* _finalState) {
		if (_finalState != NULL) {
			finalState = _finalState;
		}
	}

	void initiate() {
		if (initialState != NULL) {
			currentState = initialState;
			this->execute();
		}
	}

	void addTransition(SimpleState* _fromState, SimpleState* _toState, SimpleEvent event) {
		pair<SimpleState*, SimpleState*> transitionPair(_fromState, _toState);

		TransitionMapType::iterator mapIt = transitionMap.find(event.eventId);
		if (mapIt != transitionMap.end()) {
			(*mapIt).second.insert(transitionPair);
		} else {
			map<SimpleState*, SimpleState*> valueMap;
			valueMap.insert(transitionPair);

			transitionMap.insert(pair<int, map<SimpleState*, SimpleState*> > (event.eventId, valueMap));
		}
	}

	void process_event(SimpleEvent _event) {
		TransitionMapType::const_iterator mapIt = transitionMap.find(_event.eventId);
		if (mapIt != transitionMap.end()) {
			map<SimpleState*, SimpleState*> valueMap = (*mapIt).second;

			map<SimpleState*, SimpleState*>::const_iterator valueIt = valueMap.find(currentState);
			if (valueIt != valueMap.end()) {
				currentState = (*valueIt).second;
			}
		}
	}

	virtual ~SimpleStateMachine() {
	}
};
}

#endif /* SIMPLESTATEMACHINE_HPP_ */
