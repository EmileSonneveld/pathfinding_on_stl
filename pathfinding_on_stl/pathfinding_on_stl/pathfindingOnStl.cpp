#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <cassert>

#include "stl_parser/parse_stl.h"
#include "common.h"
#include "OpenNodesHeap.h"

bool operator<(const GraphVertex& l, const GraphVertex& r) {
	return l.number < r.number;
}

float distance(const stl::point& a, const stl::point& b) {
	auto dx = a.x - b.x;
	auto dy = a.y - b.y;
	auto dz = a.z - b.z;
	return sqrt(dx*dx + dy * dy + dz * dz);
}


std::string debugOutput(const std::vector<GraphVertex*>& vertexes) {
	std::stringstream ss;

	for (auto vert : vertexes)
	{
		ss << vert->number << "  " << vert->dist << "   " << vert->prev << "\n";
	}
	ss << "\n";
	return ss.str();
};


// Based on: https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
// Modified for better performance.
// Note that this will edit the temporary vertex list.
DijkstraResult dijkstra(GraphVertex* begin, GraphVertex* goal, std::vector<GraphVertex*>& vertexes)
{
	if (begin == goal) {
		DijkstraResult(std::vector<ResultVertex> { begin->getResultCopy() }, 0, true); // no path needed
	}

	begin->dist = 0;
	OpenNodesHeap vertexSet(vertexes);

	while (!vertexSet.empty()) {

		auto u = vertexSet.pop();
		if (u->dist == std::numeric_limits<double>::infinity())
			break; // This vertex belongs to a different cluster than the begin. No path found.

		if (u == goal) {
			// Chances are slim that we would find 2 paths with the same length, so we only return 1.
			auto s = std::vector<ResultVertex>();
			if (u->prev != nullptr) {
				while (u != nullptr) {
					s.push_back(u->getResultCopy());
					u = u->prev;
				}
			}
			return DijkstraResult(s, goal->dist, true);
		}

		for (auto v : u->neigbours)
		{
			auto alt = u->dist + distance(u->p, v->p);
			if (alt < v->dist) {
				auto vIndex = vertexSet.heapSearchElementIndex(v);
				v->dist = alt;
				vertexSet.heapRevalidateElement(vIndex);
				v->prev = u;
			}
		}
	}

	return DijkstraResult(std::vector<ResultVertex>(), std::numeric_limits<double>::infinity(), false); // no path found
}


/*

bool compareFScore(const GraphVertex* const v1, const GraphVertex* const v2) {
	return v1->fScore < v2->fScore;
};

struct CompareFScoreFunctor {
	bool operator()(const vertex* const v1, const vertex* const v2) const {
		return v1->fScore < v2->fScore;
	}
};

std::string debugOutputSet(const std::set<vertex*, CompareFScoreFunctor>& vertexes) {
	std::stringstream ss;

	for (auto vert : vertexes)
	{
		ss << vert->number << "  fScore: " << vert->fScore << "  " << vert->dist << "   " << vert->prev << "\n";
	}
	ss << "\n";
	return ss.str();
};

// Based on https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
// Is not completely functional, and therefore left out of the solution
std::vector<vertex*> aStar(vertex* begin, vertex* goal, std::vector<vertex*>& vertexes)
{
	if (begin == goal) {
		return std::vector<vertex*> { begin }; // no path needed
	}

	begin->dist = 0;
	begin->fScore = distance(begin->p, goal->p);

	std::set<vertex*> closed;
	//std::set<vertex*, bool(*)(const vertex* const, const vertex* const)> open(compareFScore);
	std::set<vertex*, CompareFScoreFunctor> open;
	open.insert(begin);

	while (!open.empty())
	{
		std::cout << "== while loop ==========";

		auto current = *open.begin();
		std::cout << debugOutputSet(open);
		auto vert = current;
		std::cout << vert->number << "  fScore: " << vert->fScore << "  " << vert->dist << "   " << vert->prev << "\n";

		if (current == goal) {
			auto s = std::vector<vertex*>();
			if (current->prev != nullptr) {
				while (current != nullptr) {
					s.push_back(current);
					current = current->prev;
				}
			}
			return s;
		}

		open.erase(current);
		closed.insert(current);

		for (auto neighbor : current->neigbours)
		{
			if (closed.find(neighbor) != closed.end())
				continue;

			auto tentative_gScore = current->dist + distance(current->p, neighbor->p);
			if (open.find(neighbor) == open.end())
				open.insert(neighbor);
			else if (tentative_gScore >= neighbor->dist)
				continue;
			neighbor->prev = current;
			neighbor->dist = tentative_gScore;
			neighbor->fScore = neighbor->dist + distance(neighbor->p, goal->p);
			open.erase(neighbor); // to keep the element sorted
			open.insert(neighbor);
		}
	}
	return std::vector<vertex*>(); // no path found
}
*/


// For debuging small meshes.
std::string MakeGraphviz(const std::vector<GraphVertex*>& vertexes)
{
	std::stringstream str;
	str << "# You can visualise this file here: http://webgraphviz.com\n";
	str << "digraph G {\n";
	str << "   concentrate = true;\n";
	str << "   rankdir = LR;\n";
	for (auto& vert : vertexes) {
		str << "   " << vert->number << ";\n";
		for (auto neighbor : vert->neigbours)
		{
			str << "   " << vert->number << " -> " << neighbor->number << ";\n";
		}
	}
	str << "};\n";
	return str.str();
}


typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::time_point<std::chrono::steady_clock> TimePoint;
long long millisecondsBetweenTimes(const TimePoint t1, const TimePoint t2)
{
	return (t1 - t2).count() / 1000;
}

DijkstraResult calculatePath(const stl::point& begin, const stl::point& goal, const stl::stl_data& stlData)
{
	auto vertexes = std::map<stl::point, GraphVertex>();

	auto t1 = Clock::now();
	int vertexCount = 0;
	auto assureVertex = [&](stl::point& pt) -> GraphVertex& {
		auto find = vertexes.find(pt);
		if (find == vertexes.end()) {
			auto v = GraphVertex(vertexCount, pt);
			++vertexCount;
			vertexes.insert(std::map<stl::point, GraphVertex>::value_type(pt, v));
			// Pointers to elements in a map stay valid
			return vertexes.find(pt)->second; // search again to return the new copy
		}
		else {
			return find->second;
		}
	};
	auto assureConnection = [&](GraphVertex* v1, GraphVertex* v2) {
		if (v1->neigbours.find(v2) == v1->neigbours.end()) {
			v1->neigbours.insert(v2);
			v2->neigbours.insert(v1);
		}
	};

	for (auto t : stlData.triangles) {
		auto& v1 = assureVertex(t.v1);
		auto& v2 = assureVertex(t.v2);
		auto& v3 = assureVertex(t.v3);
		assureConnection(&v1, &v2);
		assureConnection(&v2, &v3);
		assureConnection(&v3, &v1);
	}
	auto t2 = Clock::now();
	log() << "Conversion time: " << millisecondsBetweenTimes(t2, t1) << "ms\n";

	auto itBegin = std::find_if(vertexes.begin(), vertexes.end(), [&](auto t) -> bool {
		return distance(t.second.p, begin) < 0.00001; // higher precision causes floating point problems
	});
	auto itGoal = std::find_if(vertexes.begin(), vertexes.end(), [&](auto t) -> bool {
		return distance(t.second.p, goal) < 0.00001; // higher precision causes floating point problems
	});
	assert(itBegin != vertexes.end());
	assert(itGoal != vertexes.end());

	auto vertexVec = std::vector<GraphVertex*>(vertexes.size());
	for (auto& vert : vertexes) {
		vertexVec[vert.second.number] = &vert.second;
	}
	//auto graphviz = MakeGraphviz(vertexVec);

	auto tDijkstra1 = Clock::now();
	auto result = dijkstra(&(itBegin->second), &(itGoal->second), vertexVec);
	//auto path = aStar(&(itBegin->second), &(itGoal->second), vertexVec);
	auto tDijkstra2 = Clock::now();
	log() << "Dijkstra time: " << millisecondsBetweenTimes(tDijkstra2, tDijkstra1) << "ms\n";

	return result;
}
