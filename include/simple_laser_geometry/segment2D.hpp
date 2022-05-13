/*
 * SEGMENT 2D CLASS
 *
 * Copyright (c) 2017-2021 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of simple_laser_geometry.
 * 
 * All rights reserved.
 *
 */

#ifndef SEGMENT2D_H
#define SEGMENT2D_H

#include <vector>
#include <algorithm>
#include <numeric>

#include "point2D.hpp"

namespace slg{
class Segment2D{
	public:
		Segment2D(){id = 0; label = BACKGROUND;}

		Segment2D(int id, Point2D prevPoint, Point2D currPoint, Point2D nextPoint){
			this->id = id; 
			label = BACKGROUND;
			points.push_back(currPoint);
			lastPointPriorSeg = prevPoint;
			firstPointNextSeg = nextPoint;
			lastCentroid = currPoint;
		}

		Segment2D& operator=  (const Segment2D& seg){
			if(this != &seg){
				this->id = seg.getId();
				this->label = seg.getLabel();
				this->points = seg.getPoints();
				this->lastPointPriorSeg = seg.getPriorSegment();
				this->firstPointNextSeg = seg.getNextSegment();
				this->lastCentroid = seg.getLastCentroid();
			}
			return *this;
		}

		int size() 				const		{ return points.size(); }
		bool empty() 			const		{ return points.empty(); }
		void clear() 						{ points.clear(); id = 0; label = BACKGROUND; }
		double width() 			const		{ return (points.back() - points.front()).length(); }
		double widthSquared() 	const		{ return (points.back() - points.front()).lengthSquared(); }
		Point2D firstPoint()	const		{ return points.front(); }
		Point2D lastPoint()		const		{ return points.back(); }
		Point2D vector()		const		{ return points.back() - points.front(); }
		double minAngle()		const		{ return points.front().angle(); }
		double maxAngle()		const		{ return points.back().angle(); }
		double meanAngle()		const		{ return (points.front().angle() + points.back().angle()) / 2.0; }
		int getId()				const		{ return id; }
		Label getLabel()		const		{ return label; }
		std::vector<Point2D> getPoints() const{ return points; }
		Point2D getPriorSegment()	const		{ return lastPointPriorSeg; }
		Point2D getNextSegment()	const		{ return firstPointNextSeg; }
		Point2D getLastCentroid()	const		{ return lastCentroid; }
		double getAngularDistanceToClosestBoundary() const {return angularDistanceToClosestBoundary; }
		void setId(int id)					{ this->id = id; }
		void setLabel(Label label)			{ this->label = label; }
		void setPriorSegment(Point2D point)	{ lastPointPriorSeg = point; }
		void setNextSegment(Point2D point)	{ firstPointNextSeg = point; }
		void setLastCentroid(Point2D point)	{ lastCentroid = point; }
		void setAngularDistanceToClosestBoundary(double angle) {angularDistanceToClosestBoundary = angle; }

		double orientation(){
			return (vector().y != 0.0) ? Point2D(-1, - vector().x / vector().y).angle() : 0.0;
		}

		Point2D projection(const Point2D& p) const {
			Point2D a = points.back() - points.front();
			Point2D b = p - points.front();
			return points.front() + a.dot(b) * a / a.lengthSquared();
		}

		double distanceTo(const Point2D& p) const {
			return (p - projection(p)).length();
		}

		Point2D centroid() const{
			Point2D sum = std::accumulate(points.begin(), points.end(), Point2D(0.0,0.0));
			return sum / points.size();
		}

		double height() const{
			auto maxPoint = std::max_element(points.begin(), points.end(), [](Point2D p1, Point2D p2){return p1.y < p2.y;});
			return distanceTo(*maxPoint);
		}

		void addPoint(Point2D point){
			points.push_back(point);
			lastCentroid = centroid();
		}

		void addPoints(std::vector<Point2D> newPoints){
			points.insert(points.end(), newPoints.begin(), newPoints.end());
		}

		void merge(Segment2D seg){
			if(label != seg.getLabel()) label = BACKGROUND;

			std::vector<Point2D> newPoints = seg.getPoints();
			points.insert(points.end(), newPoints.begin(), newPoints.end());
			firstPointNextSeg = seg.getNextSegment();
			lastCentroid = centroid();
		}

		Segment2D leftSplit(int index){
			std::vector<Point2D> left(points.begin(), points.begin() + index);

			Segment2D splitedSegment;
			splitedSegment.addPoints(left);
			splitedSegment.setId(this->id);
			splitedSegment.setLabel(this->label);
			splitedSegment.setPriorSegment(this->lastPointPriorSeg);
			splitedSegment.setNextSegment(points[index+1]);

			return splitedSegment;
		}

		Segment2D rightSplit(int index){
			std::vector<Point2D> right(points.begin() + index, points.end());

			Segment2D splitedSegment;
			splitedSegment.addPoints(right);
			splitedSegment.setId(this->id);
			splitedSegment.setLabel(this->label);
			splitedSegment.setPriorSegment(points[index-1]);
			splitedSegment.setNextSegment(this->firstPointNextSeg);

			return splitedSegment;
		}

	private:
		int id;
		Label label;
		double angularDistanceToClosestBoundary;
		std::vector<Point2D> points;
		Point2D lastPointPriorSeg, firstPointNextSeg, lastCentroid;
};
}

#endif
