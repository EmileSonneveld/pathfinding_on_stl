#pragma once

#include <cassert>
#include <vector>
#include <string>
#include <algorithm>

#include "common.h"

class OpenNodesHeap {

	std::vector<GraphVertex*>& vertexSet;

public:
	OpenNodesHeap(std::vector<GraphVertex*>& vertexes);

	bool empty();
	GraphVertex* pop();

	unsigned int heapGetParentIndex(unsigned int i);
	unsigned int heapGetLeftChild(unsigned int i);
	unsigned int heapGetRightChild(unsigned int i);
	void heapSwap(unsigned int ia, unsigned int ib);
	void heapSiftUp(int index);
	void heapSiftDown(int index);
	void heapRevalidateElement(int index); // O( log(n) )
	
	int heapSearchElementIndex(GraphVertex* needle, unsigned int finger = 0);

};