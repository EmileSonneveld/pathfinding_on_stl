#include "OpenNodesHeap.h"

OpenNodesHeap::OpenNodesHeap(std::vector<Vertex*>& vertexes) :vertexSet(vertexes)
{
	std::make_heap(vertexSet.begin(), vertexSet.end(), sortOnDist); // O(3n)
}

bool OpenNodesHeap::empty()
{
	return vertexSet.empty();
}

Vertex * OpenNodesHeap::pop()
{
	auto tmp = vertexSet.front();

	std::pop_heap(vertexSet.begin(), vertexSet.end(), sortOnDist); // O(2 lg(n))
	vertexSet.pop_back();
	return tmp;
}

unsigned int OpenNodesHeap::heapGetParentIndex(unsigned int i) {
	return (int)floor((i - 1) / 2);
}

unsigned int OpenNodesHeap::heapGetLeftChild(unsigned int i) {
	return (i * 2) + 1;
}

unsigned int OpenNodesHeap::heapGetRightChild(unsigned int i) {
	return (i * 2) + 2;
}

void OpenNodesHeap::heapSwap(unsigned int ia, unsigned int ib) {
	auto tmp = vertexSet[ia];
	vertexSet[ia] = vertexSet[ib];
	vertexSet[ib] = tmp;
}

void OpenNodesHeap::heapSiftUp(int index)
{
	if (index == 0) return;
	auto val = vertexSet[index];
	auto parentI = heapGetParentIndex(index);
	auto parent = vertexSet[parentI];
	assert(parent >= 0);
	if (parent == 0) return;
	if (sortOnDist(parent, val)) {
		heapSwap(index, parentI);
		heapSiftUp(parentI);
	}
}

void OpenNodesHeap::heapSiftDown(int index)
{
	auto li = heapGetLeftChild(index);
	auto ri = heapGetRightChild(index);
	if (li >= vertexSet.size()) return; // no child nodes
	int maxChildI = -1;
	if (ri >= vertexSet.size()) // only left child
		maxChildI = li;
	else {
		auto vl = vertexSet[li];
		auto vr = vertexSet[ri];
		maxChildI = sortOnDist(vl, vr) ? ri : li;
	}
	if (sortOnDist(vertexSet[index], vertexSet[maxChildI])) {
		heapSwap(index, maxChildI);
		heapSiftDown(maxChildI);
	}
}

void OpenNodesHeap::heapRevalidateElement(int index) {
	// only one of these should have effect
	heapSiftUp(index);
	heapSiftDown(index);
}

int OpenNodesHeap::heapSearchElementIndex(Vertex * needle, unsigned int finger) {
	if (finger >= vertexSet.size())
		return -1;
	auto el = vertexSet[finger];

	if (el == needle) return finger;
	if (sortOnDist(el, needle)) // maybe swap arguments?
		return -1;
	auto l = heapSearchElementIndex(needle, heapGetLeftChild(finger));
	if (l != -1) return l;
	auto r = heapSearchElementIndex(needle, heapGetRightChild(finger));
	if (r != -1) return r;
	return -1;
}
