// Copyright (c) 2017 Alberto J. Tudela Roldán
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "slg_msgs/polygon.hpp"

slg::Polygon square()
{
  slg::Polygon polygon;
  slg::Edge edge(slg::Point2D(0.0, 0.0, slg::PERSON), slg::Point2D(0.0, 1.0, slg::PERSON));
  slg::Edge edge2(slg::Point2D(0.0, 1.0, slg::PERSON), slg::Point2D(1.0, 1.0, slg::PERSON));
  slg::Edge edge3(slg::Point2D(1.0, 1.0, slg::PERSON), slg::Point2D(1.0, 0.0, slg::PERSON));
  slg::Edge edge4(slg::Point2D(1.0, 0.0, slg::PERSON), slg::Point2D(0.0, 0.0, slg::PERSON));
  polygon.add_edge(edge);
  polygon.add_edge(edge2);
  polygon.add_edge(edge3);
  polygon.add_edge(edge4);
  return polygon;
}

// Default constructor, constructor with values and copy constructor for Edge
TEST(EdgeTest, constructors) {
  // Default constructor
  slg::Edge edge;
  EXPECT_DOUBLE_EQ(edge.a.x, 0.0);
  EXPECT_DOUBLE_EQ(edge.a.y, 0.0);
  EXPECT_DOUBLE_EQ(edge.b.x, 0.0);
  EXPECT_DOUBLE_EQ(edge.b.y, 0.0);
  // Constructor with values
  slg::Edge edge2(slg::Point2D(1.0, 2.0, slg::PERSON), slg::Point2D(3.0, 4.0, slg::PERSON));
  EXPECT_DOUBLE_EQ(edge2.a.x, 1.0);
  EXPECT_DOUBLE_EQ(edge2.a.y, 2.0);
  EXPECT_DOUBLE_EQ(edge2.b.x, 3.0);
  EXPECT_DOUBLE_EQ(edge2.b.y, 4.0);
  // Copy constructor
  slg::Edge edge3(edge2);
  EXPECT_DOUBLE_EQ(edge3.a.x, 1.0);
  EXPECT_DOUBLE_EQ(edge3.a.y, 2.0);
  EXPECT_DOUBLE_EQ(edge3.b.x, 3.0);
  EXPECT_DOUBLE_EQ(edge3.b.y, 4.0);
}

// Dimensions for the Edge struct
TEST(EdgeTest, dimensions) {
  // Check the distance between edge and point
  slg::Edge edge(slg::Point2D(1.0, 2.0, slg::PERSON), slg::Point2D(3.0, 4.0, slg::PERSON));
  slg::Point2D point(2.0, 3.0, slg::PERSON);
  EXPECT_DOUBLE_EQ(edge.distance(point), 0.0);
  point = slg::Point2D(4.0, 6.0, slg::PERSON);
  EXPECT_DOUBLE_EQ(edge.distance(point), 0.70710678118654746);
  // Check if the edge is a self edge
  EXPECT_FALSE(edge.self());
  edge = slg::Edge(slg::Point2D(1.0, 2.0, slg::PERSON), slg::Point2D(1.0, 2.0, slg::PERSON));
  EXPECT_TRUE(edge.self());
}

// Check operators for the Edge struct
TEST(EdgeTest, operators) {
  slg::Edge edge(slg::Point2D(1.0, 2.0, slg::PERSON), slg::Point2D(3.0, 4.0, slg::PERSON));
  slg::Point2D point(2.0, 3.0, slg::PERSON);
  EXPECT_TRUE(edge(point));
  point = slg::Point2D(4.0, 5.0, slg::PERSON);
  EXPECT_FALSE(edge(point));
  // Equality operator
  slg::Edge edge2(slg::Point2D(1.0, 2.0, slg::PERSON), slg::Point2D(3.0, 4.0, slg::PERSON));
  EXPECT_TRUE(edge == edge2);
  // Not equal operator
  slg::Edge edge3(slg::Point2D(1.0, 2.0, slg::PERSON), slg::Point2D(3.0, 4.0, slg::PERSON));
  slg::Edge edge4(slg::Point2D(5.0, 6.0, slg::PERSON), slg::Point2D(7.0, 8.0, slg::BACKGROUND));
  EXPECT_TRUE(edge3 != edge4);
  // Assignment operator
  slg::Edge edge5 = edge3;
  EXPECT_TRUE(edge5 == edge3);
  // Stream operator
  std::stringstream ss;
  ss << edge;
  EXPECT_EQ(ss.str(), "[(1, 2) -> (3, 4)] ");
}

// Default constructor, copy constructor
// and constructor from geometry_msgs::msg::Polygon for Polygon
TEST(PolygonTest, constructors) {
  // Default constructor
  slg::Polygon polygon;
  EXPECT_EQ(polygon.get_edges().size(), 0);
  EXPECT_EQ(polygon.get_name(), "");
  // Copy constructor
  slg::Polygon polygon2;
  slg::Edge edge(slg::Point2D(1.0, 2.0, slg::PERSON), slg::Point2D(3.0, 4.0, slg::PERSON));
  polygon2.add_edge(edge);
  polygon2.set_name("test");
  slg::Polygon polygon3(polygon2);
  EXPECT_EQ(polygon3.get_edges().size(), 1);
  EXPECT_DOUBLE_EQ(polygon3.get_edge(0).a.x, 1.0);
  EXPECT_DOUBLE_EQ(polygon3.get_edge(0).a.y, 2.0);
  EXPECT_DOUBLE_EQ(polygon3.get_edge(0).b.x, 3.0);
  EXPECT_DOUBLE_EQ(polygon3.get_edge(0).b.y, 4.0);
  EXPECT_EQ(polygon3.get_name(), "test");
  // Constructor from geometry_msgs::msg::Polygon
  geometry_msgs::msg::Polygon polygon_msg;
  geometry_msgs::msg::Point32 point;
  point.x = 1.0;
  point.y = 2.0;
  polygon_msg.points.push_back(point);
  point.x = 3.0;
  point.y = 4.0;
  polygon_msg.points.push_back(point);
  point.x = 5.0;
  point.y = 6.0;
  polygon_msg.points.push_back(point);
  slg::Polygon polygon4(polygon_msg);
  EXPECT_EQ(polygon4.get_edges().size(), 3);
  EXPECT_DOUBLE_EQ(polygon4.get_edge(0).a.x, 1.0);
  EXPECT_DOUBLE_EQ(polygon4.get_edge(0).a.y, 2.0);
  EXPECT_DOUBLE_EQ(polygon4.get_edge(0).b.x, 3.0);
  EXPECT_DOUBLE_EQ(polygon4.get_edge(0).b.y, 4.0);
}

// Check dimensions
TEST(PolygonTest, dimensions) {
  // Create a square
  slg::Polygon polygon = square();
  // Size of the polygon
  EXPECT_EQ(polygon.size(), 4);
  // Check if the polygon is empty
  EXPECT_FALSE(polygon.empty());
  // Check if the polygon contains a point
  slg::Point2D point(0.5, 0.5, slg::PERSON);
  EXPECT_TRUE(polygon.contains(point));
  // Get the centroid of the polygon
  slg::Point2D centroid = polygon.centroid();
  EXPECT_DOUBLE_EQ(centroid.x, 0.5);
  EXPECT_DOUBLE_EQ(centroid.y, 0.5);
  // Check if the polygon is closed
  EXPECT_TRUE(polygon.is_closed());
  // Clear the polygon
  polygon.clear();
  EXPECT_EQ(polygon.size(), 0);
  // Close the polygon
  slg::Edge edge(slg::Point2D(0.0, 0.0, slg::PERSON), slg::Point2D(0.0, 1.0, slg::PERSON));
  slg::Edge edge2(slg::Point2D(0.0, 1.0, slg::PERSON), slg::Point2D(1.0, 1.0, slg::PERSON));
  slg::Edge edge3(slg::Point2D(1.0, 1.0, slg::PERSON), slg::Point2D(1.0, 0.0, slg::PERSON));
  polygon.add_edge(edge);
  polygon.add_edge(edge2);
  polygon.add_edge(edge3);
  polygon.close();
  EXPECT_TRUE(polygon.is_closed());
}

// Points operations
TEST(PolygonTest, pointsOperations) {
  // Add a point to the polygon
  slg::Polygon polygon;
  polygon.add_point(slg::Point2D(0.0, 0.0, slg::PERSON));
  polygon.add_point(slg::Point2D(0.0, 1.0, slg::PERSON));
  polygon.add_point(slg::Point2D(1.0, 1.0, slg::PERSON));
  polygon.add_point(slg::Point2D(1.0, 0.0, slg::PERSON));
  // Get the points
  std::vector<slg::Point2D> points = polygon.get_points();
  EXPECT_EQ(points.size(), 4);
  EXPECT_DOUBLE_EQ(points[0].x, 0.0);
  EXPECT_DOUBLE_EQ(points[0].y, 0.0);
  EXPECT_DOUBLE_EQ(points[1].x, 0.0);
  EXPECT_DOUBLE_EQ(points[1].y, 1.0);
  EXPECT_DOUBLE_EQ(points[2].x, 1.0);
  EXPECT_DOUBLE_EQ(points[2].y, 1.0);
  EXPECT_DOUBLE_EQ(points[3].x, 1.0);
  EXPECT_DOUBLE_EQ(points[3].y, 0.0);
}

// Implicit conversion to geometry_msgs::msg::Polygon
TEST(PolygonTest, implicitConversionToGeometryMsgsPolygon) {
  slg::Polygon polygon;
  polygon.add_point(slg::Point2D(1.0, 2.0, slg::PERSON));
  polygon.add_point(slg::Point2D(3.0, 4.0, slg::BACKGROUND));
  polygon.add_point(slg::Point2D(5.0, 6.0, slg::PERSON));
  geometry_msgs::msg::Polygon polygon_msg = polygon;
  EXPECT_EQ(polygon_msg.points.size(), 3);
  EXPECT_DOUBLE_EQ(polygon_msg.points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(polygon_msg.points[0].y, 2.0);
  EXPECT_DOUBLE_EQ(polygon_msg.points[1].x, 3.0);
  EXPECT_DOUBLE_EQ(polygon_msg.points[1].y, 4.0);
  EXPECT_DOUBLE_EQ(polygon_msg.points[2].x, 5.0);
  EXPECT_DOUBLE_EQ(polygon_msg.points[2].y, 6.0);
}

// Check the assignment operators
TEST(PolygonTest, assignmentOperators) {
  // Check the operator assignment
  slg::Polygon polygon;
  polygon.add_point(slg::Point2D(1.0, 2.0, slg::PERSON));
  polygon.add_point(slg::Point2D(3.0, 4.0, slg::BACKGROUND));
  polygon.add_point(slg::Point2D(5.0, 6.0, slg::PERSON));
  slg::Polygon polygon2(polygon);
  EXPECT_TRUE(polygon == polygon2);
  polygon2.add_point(slg::Point2D(7.0, 8.0, slg::PERSON));
  EXPECT_FALSE(polygon == polygon2);
  // Assignment operator for the geometry_msgs::msg::Polygon
  slg::Polygon polygon3;
  geometry_msgs::msg::Polygon polygon_msg;
  geometry_msgs::msg::Point32 point;
  point.x = 1.0;
  point.y = 2.0;
  polygon_msg.points.push_back(point);
  point.x = 3.0;
  point.y = 4.0;
  polygon_msg.points.push_back(point);
  point.x = 5.0;
  point.y = 6.0;
  polygon_msg.points.push_back(point);
  polygon3 = polygon_msg;
  EXPECT_EQ(polygon3.get_points().size(), 3);
  EXPECT_DOUBLE_EQ(polygon3.get_points()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(polygon3.get_points()[0].y, 2.0);
  EXPECT_DOUBLE_EQ(polygon3.get_points()[1].x, 3.0);
  EXPECT_DOUBLE_EQ(polygon3.get_points()[1].y, 4.0);
  EXPECT_DOUBLE_EQ(polygon3.get_points()[2].x, 5.0);
  EXPECT_DOUBLE_EQ(polygon3.get_points()[2].y, 6.0);
}

// Check the comparison operators for the Polygon struct
TEST(PolygonTest, comparisonOperators) {
  // Check the operator equal
  slg::Polygon polygon;
  slg::Edge edge(slg::Point2D(1.0, 2.0, slg::PERSON), slg::Point2D(3.0, 4.0, slg::PERSON));
  slg::Edge edge2(slg::Point2D(5.0, 6.0, slg::PERSON), slg::Point2D(7.0, 8.0, slg::PERSON));
  slg::Edge edge3(slg::Point2D(9.0, 10.0, slg::PERSON), slg::Point2D(11.0, 12.0, slg::PERSON));
  polygon.add_edge(edge);
  polygon.add_edge(edge2);
  polygon.add_edge(edge3);
  slg::Polygon polygon2;
  polygon2.add_edge(edge);
  polygon2.add_edge(edge2);
  polygon2.add_edge(edge3);
  EXPECT_TRUE(polygon == polygon2);
  // Check the operator not equal
  slg::Polygon polygon3;
  polygon3.add_edge(edge);
  polygon3.add_edge(edge2);
  polygon3.add_edge(edge3);
  slg::Edge edge4(slg::Point2D(13.0, 14.0, slg::PERSON), slg::Point2D(15.0, 16.0, slg::PERSON));
  polygon3.add_edge(edge4);
  EXPECT_TRUE(polygon != polygon3);
  // Stream operator for the Polygon struct
  std::stringstream ss;
  ss << polygon;
  EXPECT_EQ(ss.str(), "Polygon: \n(1, 2) -> (3, 4)\n(5, 6) -> (7, 8)\n(9, 10) -> (11, 12)\n");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  bool success = RUN_ALL_TESTS();
  return success;
}
