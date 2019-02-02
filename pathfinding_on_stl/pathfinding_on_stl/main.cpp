#include <cassert>
#include <iostream>
#include <vector>
#include <set>
#include <string>
#include <sstream>
#include <algorithm>
#include <map>
#include <fstream>

#include "stl_parser/parse_stl.h"
#include "Vertex.h"
#include "OpenNodesHeap.h"



bool operator<(const Vertex& l, const Vertex& r) {
	return l.number < r.number;
}

float distance(const stl::point& a, const stl::point& b) {
	auto dx = a.x - b.x;
	auto dy = a.y - b.y;
	auto dz = a.z - b.z;
	return sqrt(dx*dx + dy * dy + dz * dz);
}

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

inline LogStream& log() { static LogStream l; return l; }


std::string debugOutput(const std::vector<Vertex*>& vertexes) {
	std::stringstream ss;

	for (auto vert : vertexes)
	{
		ss << vert->number << "  " << vert->dist << "   " << vert->prev << "\n";
	}
	ss << "\n";
	return ss.str();
};

struct DijkstraResult
{
public:
	DijkstraResult(std::vector<Vertex*> path, double length, bool pathFound) :
		path(path), length(length), pathFound(pathFound)
	{
	}
	std::vector<Vertex*> path;
	double length;
	bool pathFound;
};

std::ostream& operator<<(std::ostream& out, const DijkstraResult& result) {
	out << "pathFound: " << result.pathFound << "\n";
	if (result.pathFound) {
		out << "Length: " << result.length << "\n";
		out << "Path: ";
		for (auto& vert : result.path) out << vert->number << " ";
		out << "\n";
	}
	return out;
}

// Based on: https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
DijkstraResult dijkstra(Vertex* begin, Vertex* goal, std::vector<Vertex*>& vertexes)
{
	if (begin == goal) {
		DijkstraResult(std::vector<Vertex*> { begin }, 0, true); // no path needed
	}

	begin->dist = 0;
	OpenNodesHeap vertexSet(vertexes);



	while (!vertexSet.empty()) {

		auto u = vertexSet.pop();

		if (u == goal) {
			// Chances are slim that we would find 2 paths with the same length, so we only return 1.
			auto s = std::vector<Vertex*>();
			if (u->prev != nullptr) {
				while (u != nullptr) {
					s.push_back(u);
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

	return DijkstraResult(std::vector<Vertex*>(), -1, false); // no path found
}


/*

bool compareFScore(const Vertex* const v1, const Vertex* const v2) {
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

std::string MakePlainTextStlFromGraph(std::vector<Vertex*>& path)
{
	std::stringstream str;
	str << "solid PathScene\n";

	for (auto vert : path) {
		auto v1 = vert->p; v1.x += 0.5;
		auto v2 = vert->p; v2.y += 0.5;
		auto v3 = vert->p; v3.z += 0.5;
		str << " facet normal 0 1 0\n";
		str << "  outer loop\n";
		str << "  vertex " << v1;
		str << "  vertex " << v2;
		str << "  vertex " << v3;
		str << "  endloop\n";
		str << " endfacet\n";
	}
	str << "endsolid PathScene\n";
	return str.str();
}

// For debuging small meshes.
// Vizualize on http://webgraphviz.com/
std::string MakeGraphviz(std::map<stl::point, Vertex>& const vertexes)
{
	std::stringstream str;
	str << "# You can visualise this file here: http://webgraphviz.com\n";
	str << "digraph G {\n";
	str << "   concentrate = true;\n";
	for (auto& vert : vertexes) {
		str << "   " << vert.second.number << ";\n";
		for (auto neighbor : vert.second.neigbours)
		{
			str << "   " << vert.second.number << " -> " << neighbor->number << ";\n";
		}
	}
	str << "};\n";
	return str.str();
}

std::vector<Vertex*> calculatePath(int begin, int goal, std::string stlFileName)
{
	auto info = stl::parse_stl(stlFileName);

	std::vector<stl::triangle> triangles = info.triangles;
	log() << "\nstlFileName: " << stlFileName << "\n";
	log() << "#triangles = " << triangles.size() << "\n";

	auto vertexes = std::map<stl::point, Vertex>();

	int vertexCount = 0;
	auto assureVertex = [&](stl::point& pt) -> Vertex& {
		auto find = vertexes.find(pt);
		if (find == vertexes.end()) {
			auto v = Vertex(vertexCount, pt);
			++vertexCount;
			vertexes.insert(std::map<stl::point, Vertex>::value_type(pt, v));
			// Pointers to elements in a map stay valid
			return vertexes.find(pt)->second; // search again to return the new copy
		}
		else {
			return find->second;
		}
	};
	auto assureConnection = [&](Vertex* v1, Vertex* v2) {
		if (v1->neigbours.find(v2) == v1->neigbours.end()) {
			v1->neigbours.insert(v2);
			v2->neigbours.insert(v1);
		}
	};
	for (auto t : info.triangles) {
		auto& v1 = assureVertex(t.v1);
		auto& v2 = assureVertex(t.v2);
		auto& v3 = assureVertex(t.v3);
		assureConnection(&v1, &v2);
		assureConnection(&v2, &v3);
		assureConnection(&v3, &v1);
	}
	// auto graphviz = MakeGraphviz(vertexes);
	auto itBegin = std::find_if(vertexes.begin(), vertexes.end(), [&](auto t) -> bool {
		return t.second.number == begin;
	});
	auto itGoal = std::find_if(vertexes.begin(), vertexes.end(), [&](auto t) -> bool {
		return t.second.number == goal;
	});

	auto vertexVec = std::vector<Vertex*>(vertexes.size());
	for (auto& vert : vertexes) {
		vertexVec[vert.second.number] = &vert.second;
	}

	auto result = dijkstra(&(itBegin->second), &(itGoal->second), vertexVec);
	//auto path = aStar(&(itBegin->second), &(itGoal->second), vertexVec);

	log() << result << "\n";

	// For inspecting the resulting path.
	auto stl = MakePlainTextStlFromGraph(result.path);
	std::ofstream fs;
	fs.open(stlFileName + "_path.stl");
	fs << stl;
	fs.flush();

	return result.path;
}


int main(int argc, char* argv[]) {

	std::vector<Vertex*> p;
	p = calculatePath(2, 6, "../../test_models/Box1x1x1.stl");
	p = calculatePath(3, 6, "../../test_models/Box1x1x1.stl");
	p = calculatePath(4, 4, "../../test_models/Box1x1x1.stl");

	p = calculatePath(1, 8, "../../test_models/1_sided_ampr_and_text_alt.stl");
	p = calculatePath(1, 8, "../../test_models/BRACKET_EVO_II.STL");
	p = calculatePath(1, 8, "../../test_models/stanford_dragon_flat_base.stl");
	p = calculatePath(1, 90, "../../test_models/voronoi_bunny_with_loop.stl");

	std::cin.get();
}
