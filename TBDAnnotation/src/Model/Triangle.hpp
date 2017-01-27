/*
 * Triangle.hpp
 *
 *  Created on: 20/apr/2013
 *      Author: alessandro
 */

#ifndef TRIANGLE_HPP_
#define TRIANGLE_HPP_

#include <opencv2/core/core.hpp>
#include <iostream>
#include <boost/lexical_cast.hpp>

using namespace cv;

/*
 * Class Triangle
 */
class Triangle {
private:
	Point p1;
	Point p2;
	Point p3;

public:
	/*
	 * Constructors
	 */
	Triangle() {
	}

	Triangle(Point _p1, Point _p2, Point _p3) :
		p1(_p1), p2(_p2), p3(_p3) {
	}

	/*
	 * Getters and Setters
	 */
	Point getP1() const {
		return p1;
	}

	Point getP2() const {
		return p2;
	}

	Point getP3() const {
		return p3;
	}

	void setP1(Point _p1) {
		p1 = _p1;
	}

	void setP2(Point _p2) {
		p2 = _p2;
	}

	void setP3(Point _p3) {
		p3 = _p3;
	}

	/*
	 * Area given 2D-points
	 * From "http://www.mathopenref.com/coordtrianglearea.html"
	 */
	double getArea() const {
		double area;
		area = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
		area = fabs(area / 2);

		return area;
	}
};

#endif /* TRIANGLE_HPP_ */
