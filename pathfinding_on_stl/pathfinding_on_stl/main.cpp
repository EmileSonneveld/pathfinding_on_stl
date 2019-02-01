#include <cassert>
#include <iostream>
#include <vector>
#include <set>
#include <string>
#include <sstream>
#include <array>
#include <algorithm> 

#include "stl_parser/parse_stl.h"


struct vertex {
	vertex(int num, stl::point pt) : number(num), p(pt)
	{
	}
	int number;
	stl::point p;
	std::set<vertex*> neigbours = std::set<vertex*>();
};

bool operator<(const vertex& l, const vertex& r) {
	return l.number < r.number;
}

#include <map>

float distance(stl::point a, stl::point b) {
	auto dx = a.x - b.x;
	auto dy = a.y - b.y;
	auto dz = a.z - b.z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

// Based on: https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
std::vector<vertex*> dijkstra(vertex* begin, vertex* goal, std::map<stl::point, vertex>& vertexes)
{
	if (begin == goal) {
		return std::vector<vertex*> { begin }; // no path needed
	}

	auto vertex_set = std::vector<vertex*>();
	auto dist = std::map<vertex*, double>();
	auto prev = std::map<vertex*, vertex*>();

	for (auto& vert : vertexes) {
		dist.insert(std::map<vertex*, double>::value_type(&vert.second, std::numeric_limits<double>::infinity()));
		prev.insert(std::map<vertex*, vertex*>::value_type(&vert.second, nullptr));
		vertex_set.push_back(&vert.second);
	}
	dist.find(begin)->second = 0;



	auto debugOutput = [&]() {
		std::stringstream ss;

		for (auto vert : vertex_set)
		{
			ss << vert->number << "  " << dist.at(vert) << "   " << prev.at(vert) << "\n";
		}
		ss << "\n";
		return ss.str();
	};

	auto sortOnDist = [&](vertex* v1, vertex* v2) {
		return dist.at(v1) > dist.at(v2);
	};
	std::make_heap(vertex_set.begin(), vertex_set.end(), sortOnDist);
	std::cout << debugOutput();

	while (!vertex_set.empty()) {

		std::cout << "== while loop =======================\n";
		std::cout << debugOutput();

		auto u = vertex_set.front();
		std::pop_heap(vertex_set.begin(), vertex_set.end(), sortOnDist); vertex_set.pop_back();
		std::cout << "u    : " << u->number << "  " << dist.at(u) << "   " << prev.at(u) << "\n";

		if (u == goal) {
			auto s = std::vector<vertex*>();
			if (prev.at(u) != nullptr) {
				while (u != nullptr) {
					s.push_back(u);
					u = prev.at(u);
				}
			}
			return s;
		}

		for (auto v : u->neigbours)
		{
			auto alt = dist.at(u) + distance(u->p, v->p);
			if (alt < dist.at(v)) {
				dist.find(v)->second = alt;
				std::make_heap(vertex_set.begin(), vertex_set.end(), sortOnDist);
				prev.find(v)->second = u;
			}
		}
	}

	return std::vector<vertex*>(); // nothing found
}

// Vizualize on http://webgraphviz.com/
std::string MakeGraphviz(std::map<stl::point, vertex> vertexes)
{
	std::stringstream str;
	str << "# You can visualise this file here: http://webgraphviz.com\n";
	str << "digraph G {\n";
	str << "   concentrate = true;\n";
	for (auto vert : vertexes) {
		str << "   " << vert.second.number << ";\n";
		for (auto neighbor : vert.second.neigbours)
		{
			str << "   " << vert.second.number << " -> " << neighbor->number << ";\n";
		}
	}
	str << "};\n";
	return str.str();
}

int main(int argc, char* argv[]) {
	std::string stl_file_name = "./stl_parser/Box1x1x1.stl";

	if (argc == 2) {
		stl_file_name = argv[1];
	}
	else if (argc > 2) {
		std::cout << "ERROR: Too many command line arguments" << std::endl;
	}

	auto info = stl::parse_stl(stl_file_name);

	std::vector<stl::triangle> triangles = info.triangles;
	std::cout << "STL HEADER = " << info.name << std::endl;
	std::cout << "# triangles = " << triangles.size() << std::endl;

	auto vertexes = std::map<stl::point, vertex>();

	int vertexCount = 0;
	auto assureVertex = [&](stl::point& pt) -> vertex& {
		auto find = vertexes.find(pt);
		if (find == vertexes.end()) {
			++vertexCount;
			auto v = vertex(vertexCount, pt);
			//vertexes[pt] = v;
			vertexes.insert(std::map<stl::point, vertex>::value_type(pt, v));
			//return v; Would pass a reference to a local declared variable that will be cleaned up after this function call
			return vertexes.find(pt)->second; // search again to return the new copy
		}
		else {
			return find->second;
		}
	};
	auto assureConnection = [&](vertex* v1, vertex* v2) {
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

		//std::cout << t << std::endl;
	}
	auto graphviz = MakeGraphviz(vertexes);

	int count = 0;
	vertex* start = nullptr;
	for (auto& pair : vertexes) {
		++count;
		if (count == 2)
			start = &pair.second;
	}

	auto path = dijkstra(start, &vertexes.rbegin()->second, vertexes); // choose 2 arbitrary points
	for (auto& vert : path) std::cout << vert->number << " ";
	std::cout << "\n";

	std::cin.get();
}
