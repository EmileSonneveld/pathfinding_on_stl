#include <cassert>
#include <iostream>
#include <vector>
#include <set>
#include <string>
#include <sstream>
#include <array>
#include <algorithm>
#include <map>
#include <fstream>

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


float distance(stl::point a, stl::point b) {
	auto dx = a.x - b.x;
	auto dy = a.y - b.y;
	auto dz = a.z - b.z;
	return sqrt(dx*dx + dy*dy + dz*dz);
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
//LogStream l = LogStream();
inline LogStream& log() { static LogStream l; return l; }


// Based on: https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
std::vector<vertex*> dijkstra(vertex* begin, vertex* goal, std::vector<vertex*>& vertexes)
{
	if (begin == goal) {
		return std::vector<vertex*> { begin }; // no path needed
	}

	auto vertex_set = std::vector<vertex*>();
	auto dist = std::vector<double>(vertexes.size());
	auto prev = std::vector<vertex*>(vertexes.size());

	for (auto vert : vertexes) {
		dist[vert->number] = std::numeric_limits<double>::infinity();
		prev[vert->number] = nullptr;
		vertex_set.push_back(vert);
	}
	dist[begin->number] = 0;



	auto debugOutput = [&]() {
		std::stringstream ss;

		for (auto vert : vertex_set)
		{
			ss << vert->number << "  " << dist[vert->number] << "   " << prev[vert->number] << "\n";
		}
		ss << "\n";
		return ss.str();
	};

	auto sortOnDist = [&](vertex* v1, vertex* v2) {
		return dist[v1->number] > dist[v2->number];
	};
	std::make_heap(vertex_set.begin(), vertex_set.end(), sortOnDist); // O(3n)
	//log() << debugOutput();

	while (!vertex_set.empty()) {

		auto u = vertex_set.front();
		if (u == nullptr) continue; // invalidated element
		std::pop_heap(vertex_set.begin(), vertex_set.end(), sortOnDist); // O(2 lg(n))
		vertex_set.pop_back();
		//log() << "u    : " << u->number << "  " << dist[u->number] << "   " << prev[u->number] << "\n";

		if (u == goal) {
			auto s = std::vector<vertex*>();
			if (prev[u->number] != nullptr) {
				while (u != nullptr) {
					s.push_back(u);
					u = prev[u->number];
				}
			}
			return s;
		}

		for (auto v : u->neigbours)
		{
			// map::at O(log(n))
			auto alt = dist[u->number] + distance(u->p, v->p);
			if (alt < dist[v->number]) {
				dist[v->number] = alt;
				std::make_heap(vertex_set.begin(), vertex_set.end(), sortOnDist);
				prev[v->number] = u;
			}
		}
	}

	return std::vector<vertex*>(); // nothing found
}

// Vizualize on http://webgraphviz.com/
std::string MakeGraphviz(std::map<stl::point, vertex>& const vertexes)
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


std::vector<vertex*> calculatePath(int begin, int goal, std::string stl_file_name)
{
	auto info = stl::parse_stl(stl_file_name);

	std::vector<stl::triangle> triangles = info.triangles;
	log() << "\nstl_file_name: " << stl_file_name << "\n";
	log() << "#triangles = " << triangles.size() << "\n";

	auto vertexes = std::map<stl::point, vertex>();

	int vertexCount = 0;
	auto assureVertex = [&](stl::point& pt) -> vertex& {
		auto find = vertexes.find(pt);
		if (find == vertexes.end()) {
			auto v = vertex(vertexCount, pt);
			++vertexCount;
			//vertexes[pt] = v;
			vertexes.insert(std::map<stl::point, vertex>::value_type(pt, v));
			//return v; Would pass a reference to a local declared variable that will be cleaned up after this function call
			// Pointers to elements in a map stay valid
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
	}
	auto graphviz = MakeGraphviz(vertexes);
	auto it_begin = std::find_if(vertexes.begin(), vertexes.end(), [&](auto t) -> bool {
		return t.second.number == begin;
	});
	auto it_goal = std::find_if(vertexes.begin(), vertexes.end(), [&](auto t) -> bool {
		return t.second.number == goal;
	});

	auto vertexVec = std::vector<vertex*>(vertexes.size());
	for (auto& vert : vertexes) {
		vertexVec[vert.second.number] = &vert.second;
	}

	auto path = dijkstra(&(it_begin->second), &(it_goal->second), vertexVec); // choose 2 arbitrary points
	log() << "Result: ";
	for (auto& vert : path) log() << vert->number << " ";
	log() << "\n";
	return path;
}



int main(int argc, char* argv[]) {
	if (argc == 2) {
		//stl_file_name = argv[1];
		calculatePath(3, 7, argv[1]); // random path in figure
	}
	else if (argc > 2) {
		std::cout << "ERROR: Too many command line arguments" << std::endl;
	}
	else {
		calculatePath(2, 6, "./stl_parser/Box1x1x1.stl");
		calculatePath(3, 6, "./stl_parser/Box1x1x1.stl");
		calculatePath(4, 4, "./stl_parser/Box1x1x1.stl");

		calculatePath(1, 8, "../../test_models/1_sided_ampr_and_text_alt.stl");
		calculatePath(1, 8, "../../test_models/BRACKET_EVO_II.STL");
		calculatePath(1, 8, "../../test_models/stanford_dragon_flat_base.stl");
		calculatePath(1, 8, "../../test_models/voronoi_bunny_with_loop.stl");
	}
	std::cin.get();

}
