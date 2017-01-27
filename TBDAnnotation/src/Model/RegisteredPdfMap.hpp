/*
 * RegisteredPdfMap.hpp
 *
 *  Created on: 20/apr/2013
 *      Author: alessandro
 */

#ifndef REGISTEREDPDFMAP_HPP_
#define REGISTEREDPDFMAP_HPP_

#include <map>
#include <boost/serialization/map.hpp>

using namespace std;

/*
 * Class RegisteredPdfMap
 *
 * Indicates which pdf is already registered.
 * Implements the serialize function for boost serialization method of std::map<KEY, VALUE>
 * KEY is the pdf filename (without path)
 * VALUE is the pdf path
 */
class RegisteredPdfMap {
private:
	map<string, string> registeredPdfMap;

public:
	typedef map<string, string>::const_iterator const_iterator;
	typedef map<string, string>::iterator iterator;

	RegisteredPdfMap() {
	}

	/*
	 * Capacity
	 */
	size_t size() const {
		return registeredPdfMap.size();
	}

	bool empty() const {
		return registeredPdfMap.empty();
	}

	/*
	 * Iterators
	 */
	const_iterator find(const string& __x) const {
		return registeredPdfMap.find(__x);
	}

	iterator begin() {
		return registeredPdfMap.begin();
	}

	const_iterator begin() const {
		return registeredPdfMap.begin();
	}

	const_iterator end() const {
		return registeredPdfMap.end();
	}

	/*
	 * Operators
	 */
	string&
	operator[](const string& __k) {
		return registeredPdfMap[__k];
	}

	/*
	 * Modifiers
	 */
	pair<iterator, bool> insert(const pair<string, string>& __pair) {
		return registeredPdfMap.insert(__pair);
	}

	/*
	 * Boost Serialization method
	 */
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_NVP(registeredPdfMap);
	}
};

#endif /* REGISTEREDPDFMAP_HPP_ */
