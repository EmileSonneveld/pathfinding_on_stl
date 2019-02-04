#include <sstream>

#include "common.h"

bool sortOnDist(const GraphVertex* v1, const GraphVertex* v2) {
	//if (v1 == nullptr) return false;
	//if (v2 == nullptr) return true;
	return v1->dist > v2->dist;
};
//
//Vertex::Vertex : number(num), p(pt)
//{
//}

std::string DijkstraResult::MakePlainTextStlFromGraph() const
{
	std::stringstream str;
	str << "solid PathScene\n";

	for (auto vert : path) {
		auto v1 = vert.p; v1.x += 0.5;
		auto v2 = vert.p; v2.y += 0.5;
		auto v3 = vert.p; v3.z += 0.5;
		str << " facet normal 0 1 0\n"; // normal is not important for debugging.
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

bool DijkstraResult::pathFound() const
{
	return this->length != std::numeric_limits<double>::infinity();
}

std::ostream& operator<<(std::ostream& out, const DijkstraResult& result) {
	auto pathFound = result.pathFound();
	out << "pathFound: " << (pathFound ? "true" : "false") << "\n";
	if (pathFound) {
		out << "Length: " << result.length << "\n";
		out << "Path: ";
		for (auto& vert : result.path) out << vert.number << " ";
		out << "\n";
	}
	return out;
}

LogStream& log() { static LogStream l; return l; }
