#include "Vertex.h"

bool sortOnDist(Vertex* v1, Vertex* v2) {
	if (v1 == nullptr) return false;
	if (v2 == nullptr) return true;
	return v1->dist > v2->dist;
};
//
//Vertex::Vertex : number(num), p(pt)
//{
//}