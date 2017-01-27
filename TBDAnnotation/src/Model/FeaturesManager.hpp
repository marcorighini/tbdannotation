/*
 * FeaturesManager.hpp
 *
 *  Created on: 20/apr/2013
 *      Author: alessandro
 */

#ifndef FEATURESMANAGER_HPP_
#define FEATURESMANAGER_HPP_

#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "DatabaseHandler.hpp"
#include "TShighgui.hpp"
#include "CustomPermutation.hpp"

/*
 * Class FeaturesManager
 */
class FeaturesManager {
public:
	/*
	 * Feature Registration
	 */
	static void registerFeatures(const vector<Point>& featurePoints, string pageId) {
		int totalPoints = 0;
		int totalInvariants = 0;
		unsigned short int pointId = 0;

		// Constructs objects for knn search
		Mat featurePointsMat(featurePoints.size(), 2, CV_32FC1);
		for (unsigned i = 0; i < featurePoints.size(); i++) {
			Point point = featurePoints.at(i);
			featurePointsMat.at<float> (i, 0) = point.x;
			featurePointsMat.at<float> (i, 1) = point.y;
		}

		// For loops
		Mat query(1, 2, CV_32FC1);
		for (unsigned i = 0; i < featurePoints.size(); i++) {
			Point featurePoint = featurePoints.at(i);
			query.at<float> (0, 0) = featurePoint.x;
			query.at<float> (0, 1) = featurePoint.y;

			BFMatcher knnSearcher(NORM_L2);

			// Finds n nearest neighbors.
			vector<vector<DMatch> > matches;
			Mat mask = Mat::ones(1, featurePoints.size(), CV_8UC1);
			mask.at<uchar> (0, i) = 0;
			knnSearcher.knnMatch(query, featurePointsMat, matches, Parameters::N, mask, true);

			vector<Point> neighbors;
			vector<DMatch> neighborsDMatch = matches.front();

			for (unsigned q = 0; q < neighborsDMatch.size(); q++) {
				int featurePointId = neighborsDMatch.at(q).trainIdx;
				Point neighbor(featurePointsMat.at<float> (featurePointId, 0),
						featurePointsMat.at<float> (featurePointId, 1));
				neighbors.push_back(neighbor);
			}

#ifdef DEBUG_FEATURES_MANAGER_KNN
			int maxY = 0, maxX = 0, minY = INT_MAX, minX = INT_MAX;
			for (unsigned i = 0; i < featurePoints.size(); i++) {
				if (featurePoints[i].x > maxX) {
					maxX = featurePoints[i].x;
				}
				if (featurePoints[i].y > maxY) {
					maxY = featurePoints[i].y;
				}
				if (featurePoints[i].x < minX) {
					minX = featurePoints[i].x;
				}
				if (featurePoints[i].y < minY) {
					minY = featurePoints[i].y;
				}
			}

			Mat centroidsMat(maxY - minY + 50, maxX - minX + 50, CV_8UC3, Scalar(0, 0, 0));

			// Centroids
			for (unsigned i = 0; i < featurePoints.size(); i++) {
				circle(centroidsMat, Point(featurePoints[i].x - minX + 25, featurePoints[i].y - minY + 25), 4,
						Scalar(255,255, 255), -1, 8, 0);
			}

			// Center
			circle(centroidsMat, Point(query.at<float>(0, 0) - minX + 25, query.at<float>(0, 1) - minY + 25), 4,
					Scalar(0,0,255),-1,8,0);
			cout<<"Query: "<<query.at<float>(0, 0)<<","<<query.at<float>(0, 1)<<endl;

			// Neighbors
			for(unsigned i=0;i<neighbors.size();i++) {
				cout<<"Neighbor #"<<i<<": "<<neighbors[i]<<endl;
				circle(centroidsMat, Point(neighbors[i].x - minX + 25, neighbors[i].y - minY + 25), 4,
						Scalar(0,255,0),-1,8,0);
			}

			namedWindow("NEIGHBORS",CV_WINDOW_NORMAL);
			TShighgui::imshow("NEIGHBORS",centroidsMat);
			TShighgui::waitKey(0);
#endif

			// Order clockwise
			Utils::sortPointsClockwise(neighbors);

			/*
			 * Combinations of M of N elements
			 */
			cp::NextCombinationGenerator<cv::Point> mCombinationGen(neighbors, Parameters::M);
			cp::NextCombinationGenerator<cv::Point>::iterator mCombIt;
			for (mCombIt = mCombinationGen.begin(); mCombIt != mCombinationGen.end(); mCombIt++) {
				// Vector with the current combination of M on N element
				vector<cv::Point> mCombination(*mCombIt);

#ifdef DEBUG_REGISTER_FEATURE_PRINT_M_COMBINATION
				std::cout << "[ " << mCombination[0];
				for (unsigned int j = 1; j < Parameters::M; ++j) {
					std::cout << ", " << mCombination[j];
				}
				std::cout << " ]" << std::endl;
#endif

				list<unsigned char> affineDiscreteInvariantList;
				/*
				 * Combinations of F of M elements
				 */
				cp::NextCombinationGenerator<cv::Point> fCombinationGen(mCombination, Parameters::F);
				cp::NextCombinationGenerator<cv::Point>::iterator fCombIt;
				for (fCombIt = fCombinationGen.begin(); fCombIt != fCombinationGen.end(); fCombIt++) {
					// Vector with the current combination of F on M element
					vector<cv::Point> fCombination(*fCombIt);

#ifdef DEBUG_REGISTER_FEATURE_PRINT_F_COMBINATION
					std::cout << "\t[ " << fCombination[0];
					for (unsigned int j = 1; j < Parameters::F; ++j) {
						std::cout << ", " << fCombination[j];
					}
					std::cout << " ]" << std::endl;
#endif

					// Calculate the invariant
					double affineInvariant = Utils::getAffineInvariant(fCombination);

					// It might exist a triangle with aligned vertices, the result is -1 from getAffineInvariant()
					if (affineInvariant >= 0) {
						// Discretize the invariant
						unsigned char affineDiscreteInvariant = FeaturesManager::discretizeFeature(affineInvariant);

						affineDiscreteInvariantList.push_back(affineDiscreteInvariant);

						totalInvariants++;

#ifdef DEBUG_FEATURES_MANAGER_DISCRETIZATION_STATISTICS
						Parameters::affineInvariantVector.push_back(affineInvariant);
						Parameters::DISCRETIZATION_STATISTICS[affineDiscreteInvariant]++;

						if (affineInvariant > Parameters::maxAffineInvariant) {
							Parameters::maxAffineInvariant = affineInvariant;
						}
					} else {
						Parameters::invalidAffineInvariants++;
#endif
					}
				} // End F of M combination

				// Calculate hash index
				int hashIndex = Utils::getHashIndex(affineDiscreteInvariantList);

				// Insert in the map
				DatabaseHandler::insert(Page(pageId, pointId, affineDiscreteInvariantList), hashIndex);

				pointId++;
			} // End M of N combination

			totalPoints++;
		}

#ifdef DEBUG_REGISTER_FEATURE_PRINT_FINAL_STATISTICS
		std::cout << "\t\t\t" << "Total points: " << totalPoints << "\n" <<
		"\t\t\t" << "Total combinations: "
		<< pointId << "\n" << "\t\t\t" << "Total invariants: " << totalInvariants << "\n";
#endif
	}

	/*
	 * Page Retrieval
	 */
	static string retrievePage(const vector<Point>& featurePoints) {
		// Page id to be returned
		string votedPageId("");

		/*
		 * Voting data structures
		 */
		VotingMap votingMap;
		VotedPointIdMap votedPointIdMap;

#ifdef DEBUG_RETRIEVE_PAGE_PRINT_FINAL_STATISTICS
		unsigned int totalCondition1 = 0;
		unsigned int totalCondition2 = 0;
		unsigned int totalCondition3 = 0;
		unsigned int totalConditionsSatisfied = 0;
#endif

		// Constructs objects for knn search
		Mat featurePointsMat(featurePoints.size(), 2, CV_32FC1);
		for (unsigned i = 0; i < featurePoints.size(); i++) {
			Point point = featurePoints.at(i);
			featurePointsMat.at<float> (i, 0) = point.x;
			featurePointsMat.at<float> (i, 1) = point.y;
		}

		// For loops
		Mat query(1, 2, CV_32FC1);
		for (unsigned i = 0; i < featurePoints.size(); i++) {
			map<string, bool> votedDocumentForPoint;

			Point featurePoint = featurePoints.at(i);
			query.at<float> (0, 0) = featurePoint.x;
			query.at<float> (0, 1) = featurePoint.y;

			BFMatcher knnSearcher(NORM_L2);

			// Finds n nearest neighbors.
			vector<vector<DMatch> > matches;
			Mat mask = Mat::ones(1, featurePoints.size(), CV_8UC1);
			mask.at<uchar> (0, i) = 0;
			knnSearcher.knnMatch(query, featurePointsMat, matches, Parameters::N, mask, true);

			vector<Point> neighbors;
			vector<DMatch> neighborsDMatch = matches.front();

			for (unsigned q = 0; q < neighborsDMatch.size(); q++) {
				int featurePointId = neighborsDMatch.at(q).trainIdx;
				Point neighbor(featurePointsMat.at<float> (featurePointId, 0),
						featurePointsMat.at<float> (featurePointId, 1));
				neighbors.push_back(neighbor);
			}

#ifdef DEBUG_FEATURES_MANAGER_KNN
			int maxY = 0, maxX = 0, minY = INT_MAX, minX = INT_MAX;
			for (unsigned i = 0; i < featurePoints.size(); i++) {
				if (featurePoints[i].x > maxX) {
					maxX = featurePoints[i].x;
				}
				if (featurePoints[i].y > maxY) {
					maxY = featurePoints[i].y;
				}
				if (featurePoints[i].x < minX) {
					minX = featurePoints[i].x;
				}
				if (featurePoints[i].y < minY) {
					minY = featurePoints[i].y;
				}
			}

			Mat centroidsMat(maxY - minY + 50, maxX - minX + 50, CV_8UC3, Scalar(0, 0, 0));

			// Centroids
			for (unsigned i = 0; i < featurePoints.size(); i++) {
				circle(centroidsMat, Point(featurePoints[i].x - minX + 25, featurePoints[i].y - minY + 25), 4,
						Scalar(255,255, 255), -1, 8, 0);
			}

			// Center
			circle(centroidsMat, Point(query.at<float>(0, 0) - minX + 25, query.at<float>(0, 1) - minY + 25), 4,
					Scalar(0,0,255),-1,8,0);
			cout<<"Query: "<<query.at<float>(0, 0)<<","<<query.at<float>(0, 1)<<endl;

			// Neighbors
			for(unsigned i=0;i<neighbors.size();i++) {
				cout<<"Neighbor #"<<i<<": "<<neighbors[i]<<endl;
				circle(centroidsMat, Point(neighbors[i].x - minX + 25, neighbors[i].y - minY + 25), 4,
						Scalar(0,255,0),-1,8,0);
			}

			namedWindow("NEIGHBORS",CV_WINDOW_NORMAL);
			TShighgui::imshow("NEIGHBORS",centroidsMat);
			TShighgui::waitKey(0);
#endif

			// Order clockwise
			Utils::sortPointsClockwise(neighbors);

			/*
			 * Combinations of M of N elements
			 */
			cp::NextCombinationGenerator<cv::Point> mCombinationGen(neighbors, Parameters::M);
			cp::NextCombinationGenerator<cv::Point>::iterator mCombIt;
			for (mCombIt = mCombinationGen.begin(); mCombIt != mCombinationGen.end(); mCombIt++) {
				// Vector with the current combination of M on N element
				vector<cv::Point> mCombination(*mCombIt);

#ifdef DEBUG_RETRIEVE_PAGE_PRINT_M_COMBINATION
				std::cout << "[ " << mCombination[0];
				for (unsigned int j = 1; j < Parameters::M; ++j) {
					std::cout << ", " << mCombination[j];
				}
				std::cout << " ]" << std::endl;
#endif

				/*
				 * Cyclic permutations
				 */
				cp::NextCyclicPermutationGenerator<cv::Point> nextCyclicCombinationGen(mCombination);
				cp::NextCyclicPermutationGenerator<cv::Point>::iterator cyclicIt;
				for (cyclicIt = nextCyclicCombinationGen.begin(); cyclicIt != nextCyclicCombinationGen.end(); cyclicIt++) {
					// Vector with the current cyclic permutation
					vector<cv::Point> mCyclicPermutation(*cyclicIt);

#ifdef DEBUG_RETRIEVE_PAGE_PRINT_CYCLIC_M_COMBINATION
					std::cout << "\t[ " << mCyclicPermutation[0];
					for (unsigned int j = 1; j < Parameters::M; ++j) {
						std::cout << ", " << mCyclicPermutation[j];
					}
					std::cout << " ]" << std::endl;
#endif

					list<unsigned char> affineDiscreteInvariantList;
					/*
					 * Combinations of F of M elements
					 */
					cp::NextCombinationGenerator<cv::Point> fCombinationGen(mCombination, Parameters::F);
					cp::NextCombinationGenerator<cv::Point>::iterator fCombIt;
					for (fCombIt = fCombinationGen.begin(); fCombIt != fCombinationGen.end(); fCombIt++) {
						// Vector with the current combination of F on M element
						vector<cv::Point> fCombination(*fCombIt);

#ifdef DEBUG_RETRIEVE_PAGE_PRINT_F_COMBINATION
						std::cout << "\t\t[ " << fCombination[0];
						for (unsigned int j = 1; j < Parameters::F; ++j) {
							std::cout << ", " << fCombination[j];
						}
						std::cout << " ]" << std::endl;
#endif

						// Calculate the invariant
						double affineInvariant = Utils::getAffineInvariant(fCombination);

						// It might exist a triangle with aligned vertices, the result is -1 from getAffineInvariant()
						if (affineInvariant >= 0) {
							// Discretize the invariant
							unsigned char affineDiscreteInvariant = FeaturesManager::discretizeFeature(affineInvariant);

							affineDiscreteInvariantList.push_back(affineDiscreteInvariant);
						}
					}

					// Calculate hash index
					int hashIndex = Utils::getHashIndex(affineDiscreteInvariantList);

					/*
					 * Try to locate hashIndex in the database
					 */
					list<Page> fetchedPages;
					DatabaseHandler::find(hashIndex, fetchedPages);

					list<Page>::const_iterator listIt;
					// For all documents in the collision list
					for (listIt = fetchedPages.begin(); listIt != fetchedPages.end(); listIt++) {
						Page page = *listIt;

						/*
						 * Conditions definition
						 */
						bool condition1 = std::mismatch(affineDiscreteInvariantList.begin(),
								affineDiscreteInvariantList.end(), page.invariants.begin()).first
								== affineDiscreteInvariantList.end();
						bool condition2 = votedDocumentForPoint.find(page.pageId) == votedDocumentForPoint.end();
						bool condition3 = votedPointIdMap.isFirstVote(page.pageId, page.pointId);

#ifdef DEBUG_RETRIEVE_PAGE_PRINT_PAGE_MATCH_CONDITIONS
						std::cout << "\t\tAffineDiscreteInvariantList size: " << affineDiscreteInvariantList.size() << "\n\t\t";
						for (list<unsigned char>::iterator it = affineDiscreteInvariantList.begin(); it != affineDiscreteInvariantList.end(); it++) {
							std::cout << (int)*it << " ";
						}
						std::cout << "\n";
						std::cout << "\t\tDatabase invariants size: " << page.invariants.size() << "\n\t\t";
						for (list<unsigned char>::iterator it = page.invariants.begin(); it != page.invariants.end(); it++) {
							std::cout << (int)*it << " ";
						}
						std::cout << "\n";
						std::cout << "\t\tPageId: " << page.pageId << "\n";
						std::cout << "\t\tPointId: " << page.pointId << "\n";
						std::cout << "\t\tC1: " << (condition1 ? "true":"false") << "\t"
						<< "C2: " << (condition2 ? "true":"false") << "\t"
						<< "C3: " << (condition3 ? "true":"false") << "\n";
#endif
#ifdef DEBUG_RETRIEVE_PAGE_PRINT_FINAL_STATISTICS
						condition1 ? totalCondition1++ : 0;
						condition2 ? totalCondition2++ : 0;
						condition3 ? totalCondition3++ : 0;
						condition1 && condition2 && condition3 ? totalConditionsSatisfied++ : 0;
#endif
						// If all conditions are satisfied then the page gain a vote
						if (condition1 && condition2 && condition3) {
							// Update data structures
							votedDocumentForPoint.insert(pair<string, bool> (page.pageId, true));
							votedPointIdMap.addVote(page.pageId, page.pointId);

							// Update voting map
							VotingMap::iterator it = votingMap.find(page.pageId);
							if (it == votingMap.end()) {
								votingMap.insert(VotingMap::Vote(page.pageId, 1));
							} else {
								it->second++;
							}
						}
					}
				} // End Cyclic Permutation
			} // End M of N combination
		}

#ifdef DEBUG_RETRIEVE_PAGE_PRINT_FINAL_STATISTICS
		std::cout << "\n";
		std::cout << "\t\t\t" << "Total condition 1: " << totalCondition1 << "\n";
		std::cout << "\t\t\t" << "Total condition 2: " << totalCondition2 << "\n";
		std::cout << "\t\t\t" << "Total condition 3: " << totalCondition3 << "\n";
		std::cout << "\t\t\t" << "Total all conditions satisfied: " << totalConditionsSatisfied << "\n";
		std::cout << "\t\t\t" << "------------------------------------------\n";
		std::cout << "\t\t\t" << "Voting highscore (" << votingMap.size() << " documents):\n";

		vector<VotingMap::Vote> voteVector;
		VotingMap::iterator it;
		for(it = votingMap.begin(); it != votingMap.end(); it++) {
			voteVector.push_back(*it);
		}

		// Need for a sorted list
		sort(voteVector.begin() , voteVector.end(), VoteComparator());
		vector<VotingMap::Vote>::iterator vectorIt;
		int i=1;
		for(vectorIt = voteVector.begin(); vectorIt != voteVector.end(); vectorIt++) {
			std::cout << "\t\t\t\t" << "POSITION #" << i <<": " << vectorIt->second << " - " << vectorIt->first << "\n";
			i++;
		}
#endif
		if (votingMap.size() != 0) {
			votedPageId = votingMap.getMaxVote().first;
		}

		return votedPageId;
	}

	/*
	 * Feature discretization
	 */
	static unsigned char discretizeFeature(double feature) {
		unsigned char j = 0;
		while (j < Parameters::K - 1 && feature > Parameters::DISCRETIZATION_VECTOR[j]) {
			j++;
		}

		return j;
	}

	/*
	 * Voting data structures
	 * ##########################################################################################
	 */

	class VotedPointIdMap: public map<string, map<unsigned short int, bool> > {
	public:
		typedef map<string, map<unsigned short int, bool> > VotedPointIdMapType;
		typedef map<unsigned short int, bool> PointIdMapType;

		void addVote(string pageId, unsigned short int pointId) {
			VotedPointIdMapType::iterator it = this->find(pageId);
			if (it != this->end()) {
				it->second.insert(std::pair<unsigned short int, bool>(pointId, true));
			} else {
				PointIdMapType pointIdMap;
				pointIdMap.insert(std::pair<unsigned short int, bool>(pointId, true));

				this->insert(pair<string, PointIdMapType> (pageId, pointIdMap));
			}
		}

		bool isFirstVote(string pageId, unsigned short int pointId) {
			bool voted = true;

			VotedPointIdMapType::iterator it = this->find(pageId);
			if (it != this->end()) {
				PointIdMapType::iterator itt = it->second.find(pointId);
				if (itt != it->second.end()) {
					voted = false;
				}
			}

			return voted;
		}
	};

	class VotingMap: public map<string, int> {
	public:
		typedef map<string, int> VotingMapType;
		typedef pair<string, int> Vote;

		Vote getMaxVote() {
			Vote max("", 0);
			VotingMap::iterator it;
			for (it = this->begin(); it != this->end(); it++) {
				if (it->second > max.second) {
					max = Vote(*it);
				}
			}

			return max;
		}
	};

	/*
	 * Vote comparator
	 */
	struct VoteComparator {

		VoteComparator() {
		}

		bool operator()(VotingMap::Vote i, VotingMap::Vote j) {
			return (i.second > j.second);
		}
	};
};

#endif /* FEATURESMANAGER_HPP_ */
