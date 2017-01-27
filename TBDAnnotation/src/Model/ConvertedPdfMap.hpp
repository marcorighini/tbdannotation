/*
 * ConvertedPdfMap.hpp
 *
 *  Created on: 20/apr/2013
 *      Author: alessandro
 */

#ifndef CONVERTEDPDFMAP_HPP_
#define CONVERTEDPDFMAP_HPP_

#include <map>
#include <boost/serialization/map.hpp>

using namespace std;

/*
 * Class ConvertedPdfMap
 *
 * Indicates which pdf is already converted, with number of pages.
 * Implements the serialize function for boost serialization method of std::map<KEY, VALUE>
 * KEY is the pdf filename (without path)
 * VALUE is the pdf path
 */
class ConvertedPdfMap {
private:
	map<string, pair<string,int>> convertedPdfMap;

public:
	typedef map<string, pair<string,int>>::const_iterator const_iterator;
	typedef map<string, pair<string,int>>::iterator iterator;

	ConvertedPdfMap() {
	}

	/*
	 * Capacity
	 */
	size_t size() const {
		return convertedPdfMap.size();
	}

	bool empty() const {
		return convertedPdfMap.empty();
	}

	/*
	 * Iterators
	 */
	const_iterator find(const string& __x) const {
		return convertedPdfMap.find(__x);
	}

	iterator begin() {
		return convertedPdfMap.begin();
	}

	const_iterator begin() const {
		return convertedPdfMap.begin();
	}

	const_iterator end() const {
		return convertedPdfMap.end();
	}

	/*
	 * Operators
	 */
	pair<string,int>&
	operator[](const string& __k) {
		return convertedPdfMap[__k];
	}

	/*
	 * Modifiers
	 */
	pair<iterator, bool> insert(const pair<string, pair<string,int>>& __pair) {
		return convertedPdfMap.insert(__pair);
	}

	/*
	 * Boost Serialization method
	 */
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_NVP(convertedPdfMap);
	}
};

#endif /* CONVERTEDPDFMAP_HPP_ */
