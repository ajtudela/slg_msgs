/*
 * POLYGON CLASS
 *
 * Copyright (c) 2020-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of slg_msgs.
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

#ifndef SLG_MSGS__POLYGON_HPP_
#define SLG_MSGS__POLYGON_HPP_

#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include <numeric>

#include <geometry_msgs/msg/polygon.hpp>

#include "point2D.hpp"

const double epsilon = std::numeric_limits<float>().epsilon();
const std::numeric_limits<double> DOUBLE;
const double MIN = DOUBLE.min();
const double MAX = DOUBLE.max();

namespace slg{
struct Edge{
	Point2D a, b;
	Edge(Point2D a = Point2D(0.0, 0.0), Point2D b = Point2D(0.0, 0.0)): a(a), b(b) {}

	bool operator()(const Point2D & p) const{
		if (a.y > b.y) {return Edge{b, a}(p);}
		if (p.y == a.y || p.y == b.y) {return operator()({ p.x, p.y + epsilon });}
		if (p.y > b.y || p.y < a.y || p.x > std::max(a.x, b.x)) {return false;}
		if (p.x < std::min(a.x, b.x)) {return true;}
		auto blue = std::abs(a.x - p.x) > MIN ? (p.y - a.y) / (p.x - a.x) : MAX;
		auto red = std::abs(a.x - b.x) > MIN ? (b.y - a.y) / (b.x - a.x) : MAX;
		return blue >= red;
	}

	double distance(const Point2D & p){
		Point2D difAB = b - a;
		Point2D difPA = a - p;
		return fabs(difAB.x * difPA.y - difAB.y * difPA.x) / difAB.length();
	}
};

class Polygon{
	public:
		Polygon() : name("") {}

		Polygon(const Polygon& poly) : 
					name(poly.get_name()), 
					edges(poly.get_edges()) {}

		Polygon(const geometry_msgs::msg::Polygon & polygonMsg){
			// Read n-1 points
			for (unsigned int i = 0; i < polygonMsg.points.size()-1; i++){
				edges.push_back({polygonMsg.points[i], polygonMsg.points[i+1]});
			}
			// Add the last edge
			edges.push_back({polygonMsg.points[0], polygonMsg.points[polygonMsg.points.size()-1]});
		}

		~Polygon(){}

		operator geometry_msgs::msg::Polygon (){
			geometry_msgs::msg::Polygon polygonMsg;
			for (const auto& edge : edges){
				polygonMsg.points.push_back(edge.a);
			}
			return polygonMsg;
		}

		Polygon& operator= (const Polygon& poly){
			if (this != &poly){
				this->name = poly.get_name();
				this->edges = poly.get_edges();
			}
			return *this;
		}

		Polygon& operator= (const geometry_msgs::msg::Polygon & polygonMsg){
			*this = Polygon(polygonMsg);
			return *this;
		}

		int size() 						const { return edges.size(); }
		bool empty() 					const { return edges.empty(); }
		void clear() 						{ edges.clear(); name.clear(); }
		std::string get_name()			const { return name; }
		void set_name(std::string name)		{ this->name = name; }
		std::vector<Edge> get_edges() 	const { return edges; }
		Edge get_edge(int e) 			const{ return edges[e]; }
		void add_edge(Edge edge) 			{ edges.push_back(edge); }
		void add_edge(Point2D a, Point2D b) { edges.push_back({a, b}); }

		bool contains(const Point2D & p) const{
			auto c = 0;
			for (auto e : edges) if (e(p)) c++;
			return c % 2 != 0;
		}

		Point2D centroid() const{
			std::vector<Point2D> points;
			for (const auto & edge : edges){
				points.push_back(edge.a);
			}
			Point2D sum = std::accumulate(points.begin(), points.end(), Point2D(0.0, 0.0));
			return sum / points.size();
		}

		bool is_closed() const{
			return edges.front().a == edges.back().b;
		}

		void close(){
			if (!is_closed()){
				edges.push_back({edges.back().b, edges.front().a});
			}
		}

		void add_point(const Point2D & p){
			if (edges.empty()){
				edges.push_back({p, p});
			}else{
				if (is_closed()){
					edges.pop_back();
				}
				edges.push_back({edges.back().b, p});
				close();
			}
		}

	private:
		std::string name;
		std::vector<Edge> edges;
};
}  // namespace slg

#endif  // SLG_MSGS__POLYGON_HPP_
