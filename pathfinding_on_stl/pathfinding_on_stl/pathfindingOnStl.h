#pragma once

#include "stl_parser/parse_stl.h"
#include "common.h"
#include "OpenNodesHeap.h"

DijkstraResult calculatePath(stl::point begin, stl::point goal, stl::stl_data stlData);
