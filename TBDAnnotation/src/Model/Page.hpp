/*
 * Page.hpp
 *
 *  Created on: 20/apr/2013
 *      Author: alessandro
 */

#ifndef PAGE_HPP_
#define PAGE_HPP_

#include <string>
#include <list>
#include <boost/serialization/list.hpp>

using namespace std;

/*
 * Class Page
 */
class Page {
public:
	string pageId;
	unsigned short int pointId;
	list<unsigned char> invariants;

	Page() {
	}

	Page(string _pageId,unsigned short int _pointId, list<unsigned char> _invariants) :
		pageId(_pageId), pointId(_pointId), invariants(_invariants) {
	}
};

#endif /* PAGE_HPP_ */
