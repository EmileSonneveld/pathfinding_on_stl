#pragma once

#include <vector>
#include <string>

#include "common.h"

class OpenNodesHeap {

	std::vector<GraphVertex*>& vertexSet;

public:
	OpenNodesHeap(std::vector<GraphVertex*>& vertexes);

	bool empty() const;
	GraphVertex* pop();

	void heapSwap(unsigned int ia, unsigned int ib);
	void heapSiftUp(int index);
	void heapSiftDown(int index);
	void heapRevalidateElement(int index); // O( log(n) )
	
	int heapSearchElementIndex(const GraphVertex* needle, unsigned int finger = 0);

};

unsigned int heapGetParentIndex(unsigned int i);
unsigned int heapGetLeftChild(unsigned int i);
unsigned int heapGetRightChild(unsigned int i);
