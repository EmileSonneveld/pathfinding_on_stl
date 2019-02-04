#pragma once

#include "stl_parser/parse_stl.h"
#include "common.h"

DijkstraResult calculatePath(const stl::point& begin, const stl::point& goal, const stl::stl_data& stlData);
