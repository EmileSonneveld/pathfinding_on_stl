#pragma once

#include <cassert>
#include <vector>
#include <string>
#include <algorithm>

#include "Vertex.h"

class OpenNodesHeap {

	std::vector<Vertex*>& vertexSet;

public:
	OpenNodesHeap(std::vector<Vertex*>& vertexes);

	bool empty();
	Vertex* pop();

	unsigned int heapGetParentIndex(unsigned int i);
	unsigned int heapGetLeftChild(unsigned int i);
	unsigned int heapGetRightChild(unsigned int i);
	void heapSwap(unsigned int ia, unsigned int ib);
	void heapSiftUp(int index);
	void heapSiftDown(int index);
	void heapRevalidateElement(int index); // O( log(n) )
	
	int heapSearchElementIndex(Vertex* needle, unsigned int finger = 0);

};