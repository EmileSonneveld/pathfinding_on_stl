#pragma once

#include "stl_parser/parse_stl.h"
#include "common.h"
#include "OpenNodesHeap.h"

DijkstraResult calculatePath(int begin, int goal, stl::stl_data stlData);
