/*
 * POLYGON CLASS
 *
 * Copyright (c) 2020-2021 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of simple_laser_geometry.
 * 
 * All rights reserved.
 *
 */

/*
################################################################
# Ray-casting algorithm
#
# adapted from http://rosettacode.org/wiki/Ray-casting_algorithm
################################################################
*/

#ifndef POLYGON_H
#define POLYGON_H

#include <vector>
#include <algorithm>
#include <limits>
#include <numeric>

#include "point2D.h"

const double epsilon = std::numeric_limits<float>().epsilon();
const std::numeric_limits<double> DOUBLE;
const double MIN = DOUBLE.min();
const double MAX = DOUBLE.max();

namespace slg{
struct Edge{
	Point2D a, b;
	Edge(Point2D a = Point2D(0.0, 0.0), Point2D b = Point2D(0.0, 0.0)): a(a), b(b) {}

	bool operator()(const Point2D& p) const{
		if (a.y > b.y) return Edge{ b, a }(p);
		if (p.y == a.y || p.y == b.y) return operator()({ p.x, p.y + epsilon });
		if (p.y > b.y || p.y < a.y || p.x > std::max(a.x, b.x)) return false;
		if (p.x < std::min(a.x, b.x)) return true;
		auto blue = std::abs(a.x - p.x) > MIN ? (p.y - a.y) / (p.x - a.x) : MAX;
		auto red = std::abs(a.x - b.x) > MIN ? (b.y - a.y) / (b.x - a.x) : MAX;
		return blue >= red;
	}

	double distance(const Point2D& p){
		Point2D difAB = b - a;
		Point2D difPA = a - p;
		return fabs(difAB.x * difPA.y - difAB.y * difPA.x) / difAB.length();
	}
};

class Polygon{
	public:
		int size() 							{ return edges.size(); }
		bool empty() 			const		{ return edges.empty(); }
		void clear()						{ edges.clear(); name.clear(); }
		std::string getName()	const		{ return name; }
		void setName(std::string name)		{ this->name = name; }
		std::vector<Edge> getEdges() const	{ return edges; }
		Edge getEdge(int e) const			{ return edges[e]; }
		void addEdge(Edge edge) 			{ edges.push_back(edge); }

		bool contains(const Point2D& p) const{
			auto c = 0;
			for (auto e : edges) if (e(p)) c++;
			return c % 2 != 0;
		}

		Point2D centroid() const{
			std::vector<Point2D> points;
			for(const auto& edge: edges){
				points.push_back(edge.a);
			}
			Point2D sum = std::accumulate(points.begin(), points.end(), Point2D(0.0,0.0));
			return sum / points.size();
		}

		void offset(float off){
			//for(Edge edge: edges){
				/*if(edge.a.x < centroid().x) edge.a.x += off;
				if(edge.a.x > centroid().x) edge.a.x -= off;
				if(edge.a.y < centroid().y) edge.a.y += off;
				if(edge.a.y > centroid().y) edge.a.y -= off;

				if(edge.b.x < centroid().x) edge.b.x += off;
				if(edge.b.x > centroid().x) edge.b.x -= off;
				if(edge.b.y < centroid().y) edge.b.y += off;
				if(edge.b.y > centroid().y) edge.b.y -= off;*/
				/*Point2D vec = (edge.a - edge.b);
				edge.a += vec.perpendicular() / off;*/
			//}
		}

	private:
		std::string name;
		std::vector<Edge> edges;
};
}

#endif
