/*
 * CustomPermutation.hpp
 *
 *  Created on: 16/mag/2013
 *      Author: alessandro
 */

#ifndef CUSTOMPERMUTATION_HPP_
#define CUSTOMPERMUTATION_HPP_

#include <vector>
#include <algorithm>
#include <numeric>

namespace cp {

/*
 * Custom next combination generator
 */
template<typename T>
class NextCombinationGenerator {
	std::vector<std::vector<T> > combinations;
	unsigned int m;

	template<typename Iterator>
	bool next_combination(const Iterator first, Iterator k, const Iterator last) {
		if ((first == last) || (first == k) || (last == k))
			return false;
		Iterator itr1 = first;
		Iterator itr2 = last;
		++itr1;
		if (last == itr1)
			return false;
		itr1 = last;
		--itr1;
		itr1 = k;
		--itr2;
		while (first != itr1) {
			if (*--itr1 < *itr2) {
				Iterator j = k;
				while (!(*itr1 < *j))
					++j;
				std::iter_swap(itr1, j);
				++itr1;
				++j;
				itr2 = k;
				std::rotate(itr1, j, last);
				while (last != j) {
					++j;
					++itr2;
				}
				std::rotate(k, itr2, last);
				return true;
			}
		}
		std::rotate(first, k, last);
		return false;
	}

	/*
	 * Extract a subvector given a set of indexes
	 */
	template<typename R>
	std::vector<R> getValuesFromIndexesVector(const std::vector<R>& originals, const std::vector<unsigned int>& indexes) {
		std::vector<R> values(indexes.size());
		for (unsigned int i = 0; i < indexes.size(); i++) {
			values[i] = originals[indexes[i]];
		}

		return values;
	}

public:
	typedef typename std::vector<std::vector<T> >::iterator iterator;
	typedef typename std::vector<std::vector<T> >::const_iterator const_iterator;

	NextCombinationGenerator(const std::vector<T>& _elements, unsigned int _m) :
		m(_m) {
		std::vector<unsigned int> elementsIndexes(_elements.size());
		std::iota(std::begin(elementsIndexes), std::end(elementsIndexes), 0);
		do {
			std::vector<unsigned int> mIndexes(elementsIndexes.begin(), elementsIndexes.begin() + m);
			combinations.push_back(getValuesFromIndexesVector(_elements, mIndexes));
		} while (next_combination(elementsIndexes.begin(), elementsIndexes.begin() + m, elementsIndexes.end()));
	}

	iterator begin() {
		return combinations.begin();
	}

	const_iterator begin() const {
		return combinations.begin();
	}

	iterator end() {
		return combinations.end();
	}

	const_iterator end() const {
		return combinations.end();
	}

	unsigned int size() const {
		return combinations.size();
	}
};

/*
 * Custom next cyclic permutation generator
 */
template<typename T>
class NextCyclicPermutationGenerator {
	std::vector<std::vector<T> > cyclicPermutations;

	template<typename Iterator, typename R>
	static bool next_cyclic_permutation(const Iterator first, const Iterator last, const R terminationValue) {
		Iterator itr1 = first;
		Iterator itr2 = last;

		std::rotate(itr1, itr2 - 1, itr2);

		if (*itr1 == terminationValue)
			return false;
		return true;
	}

	/*
	 * Extract a subvector given a set of indexes
	 */
	template<typename R>
	std::vector<R> getValuesFromIndexesVector(const std::vector<R>& originals, const std::vector<unsigned int>& indexes) {
		std::vector<R> values(indexes.size());
		for (unsigned int i = 0; i < indexes.size(); i++) {
			values[i] = originals[indexes[i]];
		}

		return values;
	}

public:
	typedef typename std::vector<std::vector<T> >::iterator iterator;
	typedef typename std::vector<std::vector<T> >::const_iterator const_iterator;

	NextCyclicPermutationGenerator(const std::vector<T>& _elements) {
		std::vector<unsigned int> cyclicIndexes(_elements.size());
		std::iota (std::begin(cyclicIndexes), std::end(cyclicIndexes), 0);
		unsigned int firstIndex = cyclicIndexes[0];
		do {
			cyclicPermutations.push_back(getValuesFromIndexesVector(_elements, cyclicIndexes));
		} while (next_cyclic_permutation(cyclicIndexes.begin(), cyclicIndexes.end(), firstIndex));
	}

	iterator begin() {
		return cyclicPermutations.begin();
	}

	const_iterator begin() const {
		return cyclicPermutations.begin();
	}

	iterator end() {
		return cyclicPermutations.end();
	}

	const_iterator end() const {
		return cyclicPermutations.end();
	}

	unsigned int size() const {
		return cyclicPermutations.size();
	}
};

}

#endif /* CUSTOMPERMUTATION_HPP_ */
