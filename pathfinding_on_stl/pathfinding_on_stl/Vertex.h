#pragma once

#include <vector>
#include <set>
#include <algorithm>

#include "stl_parser/parse_stl.h"

struct Vertex {
	Vertex(int num, stl::point pt) : number(num), p(pt)
	{
	}
	int number;
	stl::point p;
	std::set<Vertex*> neigbours = std::set<Vertex*>();
	Vertex* prev = nullptr;
	double dist = std::numeric_limits<double>::infinity();
	double fScore = std::numeric_limits<double>::infinity(); // only for A*
};

bool sortOnDist(Vertex* v1, Vertex* v2);
