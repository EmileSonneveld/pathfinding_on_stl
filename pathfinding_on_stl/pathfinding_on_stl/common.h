#pragma once

#include <vector>
#include <set>
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>

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

bool sortOnDist(const GraphVertex* v1, const GraphVertex* v2);


struct DijkstraResult
{
public:
	DijkstraResult(std::vector<ResultVertex> path, double length, bool pathFound) :
		path(path), length(length)
	{
	}

	std::string MakePlainTextStlFromGraph() const;
	bool pathFound() const;

	std::vector<ResultVertex> path;
	double length;
};

std::ostream& operator<<(std::ostream& out, const DijkstraResult& result);


// https://stackoverflow.com/a/21232617/1448736
struct LogStream
{
	std::ofstream fs;
	LogStream()
	{
		fs.open("cout.txt");
	}

	template<typename T> LogStream& operator<<(const T& mValue)
	{
		std::cout << mValue;
		fs << mValue;
		fs.flush();
		return *this;
	}
};

LogStream& log();
