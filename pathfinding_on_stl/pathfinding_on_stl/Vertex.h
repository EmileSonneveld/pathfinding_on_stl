#pragma once

#include <vector>
#include <set>
#include <algorithm>

#include "stl_parser/parse_stl.h"

struct ResultVertex {
	ResultVertex(int num, stl::point pt) : number(num), p(pt)
	{
	}
	int number;
	stl::point p;
};

struct GraphVertex : ResultVertex {
	GraphVertex(int num, stl::point pt) : ResultVertex(num, pt)
	{
	}

	std::set<GraphVertex*> neigbours = std::set<GraphVertex*>();
	GraphVertex* prev = nullptr;
	double dist = std::numeric_limits<double>::infinity();
	//double fScore = std::numeric_limits<double>::infinity(); // only for A*

	ResultVertex getResultCopy() {
		return ResultVertex(number, p);
	}
};

bool sortOnDist(GraphVertex* v1, GraphVertex* v2);
