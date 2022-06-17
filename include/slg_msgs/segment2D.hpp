/*
 * SEGMENT 2D CLASS
 *
 * Copyright (c) 2017-2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of slg_msgs.
 * 
 * All rights reserved.
 *
 */

#ifndef SLG_MSGS__SEGMENT2D_HPP_
#define SLG_MSGS__SEGMENT2D_HPP_

#include <vector>
#include <algorithm>
#include <numeric>

#include <slg_msgs/msg/segment.hpp>

#include "point2D.hpp"

namespace slg{
class Segment2D{
	public:
		Segment2D() : id(0), 
					  label(BACKGROUND), 
					  angular_distance_to_closest_boundary(0.0), 
					  last_point_prior_seg(Point2D(0, 0)), 
					  first_point_next_seg(Point2D(0, 0)), 
					  last_centroid(Point2D(0, 0)) {}

		Segment2D(int newId, Point2D prevPoint, Point2D currPoint, Point2D nextPoint) : 
					id(newId), 
					label(BACKGROUND), 
					angular_distance_to_closest_boundary(0.0), 
					points({currPoint}), 
					last_point_prior_seg(prevPoint), 
					first_point_next_seg(nextPoint), 
					last_centroid(currPoint) {}

		Segment2D(const Segment2D& seg) : 
					id(seg.get_id()), 
					label(seg.get_label()), 
					angular_distance_to_closest_boundary(0.0), 
					points(seg.get_points()), 
					last_point_prior_seg(seg.get_prior_segment()), 
					first_point_next_seg(seg.get_next_segment()), 
					last_centroid(seg.get_last_centroid()) {}

		Segment2D(const simple_laser_geometry::msg::Segment& segmentMsg) : 
					id(segmentMsg.id), 
					label(slg::Label(segmentMsg.label)), 
					angular_distance_to_closest_boundary(segmentMsg.angular_distance),
					last_point_prior_seg(segmentMsg.last_point_prior_segment), 
					first_point_next_seg(segmentMsg.first_point_next_segment) 
					{
						points.insert(points.begin(), std::begin(segmentMsg.points), std::end(segmentMsg.points));
					}

		~Segment2D(){}

		operator simple_laser_geometry::msg::Segment() const{
			simple_laser_geometry::msg::Segment segmentMsg;

			// Transform the segment in message
			segmentMsg.id = id;
			segmentMsg.label = label;
			segmentMsg.angular_distance = angular_distance_to_closest_boundary;
			segmentMsg.last_point_prior_segment.x = last_point_prior_seg.x;
			segmentMsg.last_point_prior_segment.y = last_point_prior_seg.y;
			segmentMsg.first_point_next_segment.x = first_point_next_seg.x;
			segmentMsg.first_point_next_segment.y = first_point_next_seg.y;

			for (const auto& point : points){
				segmentMsg.points.push_back(point);
			}

			return segmentMsg;
		}

		Segment2D& operator= (const Segment2D & seg){
			if (this != &seg){
				this->id = seg.get_id();
				this->label = seg.get_label();
				this->points = seg.get_points();
				this->last_point_prior_seg = seg.get_prior_segment();
				this->first_point_next_seg = seg.get_next_segment();
				this->last_centroid = seg.get_last_centroid();
			}
			return *this;
		}

		Segment2D& operator= (const simple_laser_geometry::msg::Segment & segmentMsg){
			*this = Segment2D(segmentMsg);
			return *this;
		}

		int size() 				const		{ return points.size(); }
		bool empty() 			const		{ return points.empty(); }
		void clear() 						{ points.clear(); id = 0; label = BACKGROUND; }
		double width() 			const		{ return (points.back() - points.front()).length(); }
		double width_squared() 	const		{ return (points.back() - points.front()).length_squared(); }
		Point2D first_point()	const		{ return points.front(); }
		Point2D last_point()	const		{ return points.back(); }
		Point2D vector()		const		{ return points.back() - points.front(); }
		double min_angle()		const		{ return points.front().angle(); }
		double max_angle()		const		{ return points.back().angle(); }
		double mean_angle()		const		{ return (points.front().angle() + points.back().angle()) / 2.0; }
		int get_id()			const		{ return id; }
		Label get_label()		const		{ return label; }
		std::vector<Point2D> get_points() const{ return points; }
		Point2D get_prior_segment()	const		{ return last_point_prior_seg; }
		Point2D get_next_segment()	const		{ return first_point_next_seg; }
		Point2D get_last_centroid()	const		{ return last_centroid; }
		double get_angular_distance_to_closest_boundary() const {return angular_distance_to_closest_boundary; }
		void set_id(int id)					{ this->id = id; }
		void set_label(Label label)			{ this->label = label; }
		void set_prior_segment(Point2D point)	{ last_point_prior_seg = point; }
		void set_next_segment(Point2D point)	{ first_point_next_seg = point; }
		void set_last_centroid(Point2D point)	{ last_centroid = point; }
		void set_angular_distance_to_closest_boundary(double angle) {angular_distance_to_closest_boundary = angle; }

		double orientation(){
			return (vector().y != 0.0) ? Point2D(-1, - vector().x / vector().y).angle() : 0.0;
		}

		Point2D projection(const Point2D& p) const {
			Point2D a = points.back() - points.front();
			Point2D b = p - points.front();
			return points.front() + a.dot(b) * a / a.length_squared();
		}

		double distance_to(const Point2D& p) const {
			return (p - projection(p)).length();
		}

		Point2D centroid() const{
			Point2D sum = std::accumulate(points.begin(), points.end(), Point2D(0.0, 0.0));
			return sum / points.size();
		}

		double height() const{
			auto maxPoint = std::max_element(points.begin(), points.end(), [](Point2D p1, Point2D p2){return p1.y < p2.y;});
			return distance_to(*maxPoint);
		}

		void add_point(Point2D point){
			points.push_back(point);
			last_centroid = centroid();
		}

		void add_points(std::vector<Point2D> newPoints){
			points.insert(points.end(), newPoints.begin(), newPoints.end());
		}

		void merge(Segment2D seg){
			if (label != seg.get_label()) label = BACKGROUND;

			std::vector<Point2D> newPoints = seg.get_points();
			points.insert(points.end(), newPoints.begin(), newPoints.end());
			first_point_next_seg = seg.get_next_segment();
			last_centroid = centroid();
		}

		Segment2D left_split(int index){
			std::vector<Point2D> left(points.begin(), points.begin() + index);

			Segment2D splited_segment;
			splited_segment.add_points(left);
			splited_segment.set_id(this->id);
			splited_segment.set_label(this->label);
			splited_segment.set_prior_segment(this->last_point_prior_seg);
			splited_segment.set_next_segment(points[index+1]);

			return splited_segment;
		}

		Segment2D right_split(int index){
			std::vector<Point2D> right(points.begin() + index, points.end());

			Segment2D splited_segment;
			splited_segment.add_points(right);
			splited_segment.set_id(this->id);
			splited_segment.set_label(this->label);
			splited_segment.set_prior_segment(points[index-1]);
			splited_segment.set_next_segment(this->first_point_next_seg);

			return splited_segment;
		}

	private:
		int id;
		Label label;
		double angular_distance_to_closest_boundary;
		std::vector<Point2D> points;
		Point2D last_point_prior_seg, first_point_next_seg, last_centroid;
};
}  // namespace slg

#endif  // SLG_MSGS__SEGMENT2D_HPP_
