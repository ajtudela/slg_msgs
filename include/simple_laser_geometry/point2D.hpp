/*
 * POINT 2D STRUCT
 *
 * Copyright (c) 2017-2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of simple_laser_geometry.
 * 
 * All rights reserved.
 *
 */

#ifndef SIMPLE_LASER_GEOMETRY__POINT2D_HPP_
#define SIMPLE_LASER_GEOMETRY__POINT2D_HPP_

#include <limits>
#include <cmath>
#include <iostream>

namespace slg{

// Label
typedef enum {BACKGROUND, PERSON, PERSON_CANE, PERSON_WHEEL_CHAIR} Label;
static const char * labelStr[] = {"background", "person", "person_cane", "person_wheel_chair"};

// Point 2D
struct Point2D{
	Point2D(double x = 0.0, double y = 0.0, Label label = BACKGROUND) : x(x), y(y), label(label) {}
	Point2D(const Point2D& p): x(p.x), y(p.y), label(p.label) {}
	~Point2D(){}

	static Point2D fromPolarCoords(const double r, const double phi) {return Point2D(r * cos(phi), r * sin(phi));}
	static Point2D NaN() { return Point2D(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());}

	bool isNaN()           const {return std::isnan(x) || std::isnan(y);}

	double length()        const {return sqrt(pow(x, 2.0) + pow(y, 2.0));}
	double lengthSquared() const {return pow(x, 2.0) + pow(y, 2.0);}
	double angle()         const {return atan2(y, x);}
	double angleDeg()      const {return 180.0 * atan2(y, x) / M_PI;}
	double dot(const Point2D& p)   const {return x * p.x + y * p.y;}
	double cross(const Point2D& p) const {return x * p.y - y * p.x;}
	double angle3(const Point2D& p, const Point2D& q) const {return atan2(q.y - y, q.x - x) - atan2(p.y - y, p.x - x);}

	Point2D normalized() {return (length() > 0.0) ? *this / length() : *this;}
	Point2D reflected(const Point2D& normal) const {return *this - 2.0 * normal * (normal.dot(*this));}
	Point2D perpendicular() const {return Point2D(-y, x);}

	friend Point2D operator+ (const Point2D& p1, const Point2D& p2) {return Point2D(p1.x + p2.x, p1.y + p2.y);}
	friend Point2D operator- (const Point2D& p1, const Point2D& p2) {return Point2D(p1.x - p2.x, p1.y - p2.y);}
	friend Point2D operator* (const double f, const Point2D& p)  {return Point2D(f * p.x, f * p.y);}
	friend Point2D operator* (const Point2D& p, const double f)  {return Point2D(f * p.x, f * p.y);}
	friend Point2D operator/ (const Point2D& p, const double f)  {return (f != 0.0) ? Point2D(p.x / f, p.y / f) : Point2D();}

	Point2D operator- () {return Point2D(-x, -y);}
	Point2D operator+ () {return Point2D( x,  y);}

	Point2D& operator=  (const Point2D& p) {if (this != &p) { x = p.x; y = p.y; } return *this;}
	Point2D& operator+= (const Point2D& p) {x += p.x; y += p.y; return *this;}
	Point2D& operator-= (const Point2D& p) {x -= p.x; y -= p.y; return *this;}

	friend bool operator== (const Point2D& p1, const Point2D& p2) {return (p1.x == p2.x && p1.y == p2.y);}
	friend bool operator!= (const Point2D& p1, const Point2D& p2) {return !(p1 == p2); }
	friend bool operator<  (const Point2D& p1, const Point2D& p2) {return (p1.lengthSquared() < p2.lengthSquared());}
	friend bool operator<= (const Point2D& p1, const Point2D& p2) {return (p1.lengthSquared() <= p2.lengthSquared());}
	friend bool operator>  (const Point2D& p1, const Point2D& p2) {return (p1.lengthSquared() > p2.lengthSquared());}
	friend bool operator>= (const Point2D& p1, const Point2D& p2) {return (p1.lengthSquared() >= p2.lengthSquared());}
	friend bool operator!  (const Point2D& p1) {return (p1.x == 0.0 && p1.y == 0.0);}

	friend std::ostream & operator<<(std::ostream & out, const Point2D & p){
		out << "(" << p.x << ", " << p.y << ")"; return out;
	}

	double x;
	double y;
	Label label;
};
}  // namespace slg

#endif  // SIMPLE_LASER_GEOMETRY__POINT2D_HPP_
