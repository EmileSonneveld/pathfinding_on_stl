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
	vertex* prev = nullptr;
	double dist = std::numeric_limits<double>::infinity();
	double fScore = std::numeric_limits<double>::infinity(); // only for A*
};

bool operator<(const vertex& l, const vertex& r) {
	return l.number < r.number;
}


float distance(const stl::point& a, const stl::point& b) {
	auto dx = a.x - b.x;
	auto dy = a.y - b.y;
	auto dz = a.z - b.z;
	return sqrt(dx*dx + dy * dy + dz * dz);
}


struct compareOn_fScore_struct {
	bool operator()(const vertex* const v1, const vertex* const v2) const {
		return v1->fScore < v2->fScore;
	}
};


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


unsigned int heapGetParentIndex(unsigned int i) {
	return (int)floor((i - 1) / 2);
}
unsigned int heapGetLeftChild(unsigned int i) {
	return (i * 2) + 1;
}
unsigned int heapGetRightChild(unsigned int i) {
	return (i * 2) + 2;
}
void heapSwap(std::vector<vertex*>& vec, unsigned int ia, unsigned int ib) {
	auto tmp = vec[ia];
	vec[ia] = vec[ib];
	vec[ib] = tmp;
}
bool sortOnDist(vertex* v1, vertex* v2) {
	if (v1 == nullptr) return false;
	if (v2 == nullptr) return true;
	return v1->dist > v2->dist;
};
void heapSiftUp(std::vector<vertex*>& vec, int index)
{
	if (index == 0) return;
	auto val = vec[index];
	auto parentI = heapGetParentIndex(index);
	auto parent = vec[parentI];
	assert(parent >= 0);
	if (parent == 0) return;
	if (sortOnDist(parent, val)) {
		heapSwap(vec, index, parentI);
		heapSiftUp(vec, parentI);
	}
}
void heapSiftDown(std::vector<vertex*>& vec, int index)
{
	auto li = heapGetLeftChild(index);
	auto ri = heapGetRightChild(index);
	if (li >= vec.size()) return; // no child nodes
	int maxChildI = -1;
	if (ri >= vec.size()) // only left child
		maxChildI = li;
	else {
		auto vl = vec[li];
		auto vr = vec[ri];
		maxChildI = sortOnDist(vl, vr) ? ri : li;
	}
	if (sortOnDist(vec[index], vec[maxChildI])) {
		heapSwap(vec, index, maxChildI);
		heapSiftDown(vec, maxChildI);
	}
}
// O( log(n) )
void heapRevalidateElement(std::vector<vertex*>& vec, int index) {
	// only one of these should have effect
	heapSiftUp(vec, index);
	heapSiftDown(vec, index);
}


int heapSearchElementIndex(std::vector<vertex*>& vec, vertex* needle, unsigned int finger = 0) {
	if (finger >= vec.size())
		return -1;
	auto el = vec[finger];

	if (el == needle) return finger;
	if (sortOnDist(el, needle)) // maybe swap arguments?
		return -1;
	auto l = heapSearchElementIndex(vec, needle, heapGetLeftChild(finger));
	if (l != -1) return l;
	auto r = heapSearchElementIndex(vec, needle, heapGetRightChild(finger));
	if (r != -1) return r;
	return -1;
}


std::string debugOutput(const std::vector<vertex*>& vertexes) {
	std::stringstream ss;

	for (auto vert : vertexes)
	{
		ss << vert->number << "  " << vert->dist << "   " << vert->prev << "\n";
	}
	ss << "\n";
	return ss.str();
};

std::string debugOutputSet(const std::set<vertex*, compareOn_fScore_struct>& vertexes) {
	std::stringstream ss;

	for (auto vert : vertexes)
	{
		ss << vert->number << "  fScore: " << vert->fScore << "  " << vert->dist << "   " << vert->prev << "\n";
	}
	ss << "\n";
	return ss.str();
};

// Based on: https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
std::vector<vertex*> dijkstra(vertex* begin, vertex* goal, std::vector<vertex*>& vertexes)
{
	if (begin == goal) {
		return std::vector<vertex*> { begin }; // no path needed
	}

	auto vertex_set(vertexes);
	begin->dist = 0;


	std::make_heap(vertex_set.begin(), vertex_set.end(), sortOnDist); // O(3n)

	while (!vertex_set.empty()) {

		auto u = vertex_set.front();
		std::pop_heap(vertex_set.begin(), vertex_set.end(), sortOnDist); // O(2 lg(n))
		vertex_set.pop_back();

		if (u == goal) {
			auto s = std::vector<vertex*>();
			if (u->prev != nullptr) {
				while (u != nullptr) {
					s.push_back(u);
					u = u->prev;
				}
			}
			return s;
		}

		for (auto v : u->neigbours)
		{
			auto alt = u->dist + distance(u->p, v->p);
			if (alt < v->dist) {
				auto vIndex = heapSearchElementIndex(vertex_set, v);
				v->dist = alt;
				heapRevalidateElement(vertex_set, vIndex);
				v->prev = u;
			}
		}
	}

	return std::vector<vertex*>(); // no path found
}


bool compareOn_fScore(const vertex* const v1, const vertex* const v2) {
	if (v1->fScore != v2->fScore)
		return v1->fScore < v2->fScore;
	return v1 < v2; // Arbitrary comparison if floating point failed.
};
// Based on https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
std::vector<vertex*> aStar(vertex* begin, vertex* goal, std::vector<vertex*>& vertexes)
{
	if (begin == goal) {
		return std::vector<vertex*> { begin }; // no path needed
	}

	//std::map<vertex*> cameFrom; stored in vertex
	//gScore stored as dist in vertex
	begin->dist = 0;
	//std::map<const vertex*, double> fScore;
	begin->fScore = distance(begin->p, goal->p);
	//fScore.insert(std::pair<vertex*, double>(begin, distance(begin->p, goal->p)));

	std::set<vertex*> closed;
	//std::set<vertex*, bool(*)(const vertex* const, const vertex* const)> open(compareOn_fScore);
	std::set<vertex*, compareOn_fScore_struct> open;
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

std::string MakePlainTextStlFromGraph(std::vector<vertex*>& path)
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

	auto path = dijkstra(&(it_begin->second), &(it_goal->second), vertexVec);
	//auto path = aStar(&(it_begin->second), &(it_goal->second), vertexVec);
	log() << "Result: ";
	for (auto& vert : path) log() << vert->number << " ";
	log() << "\n";

	auto stl = MakePlainTextStlFromGraph(path);
	std::ofstream fs;
	fs.open(stl_file_name + "_path.stl");
	fs << stl;
	fs.flush();

	return path;
}



int main(int argc, char* argv[]) {

	std::vector<vertex*> p;
	p = calculatePath(2, 6, "../../test_models/Box1x1x1.stl");
	p = calculatePath(3, 6, "../../test_models/Box1x1x1.stl");
	p = calculatePath(4, 4, "../../test_models/Box1x1x1.stl");

	p = calculatePath(1, 8, "../../test_models/1_sided_ampr_and_text_alt.stl");
	p = calculatePath(1, 8, "../../test_models/BRACKET_EVO_II.STL");
	p = calculatePath(1, 8, "../../test_models/stanford_dragon_flat_base.stl");
	p = calculatePath(1, 90, "../../test_models/voronoi_bunny_with_loop.stl");

	std::cin.get();
}
