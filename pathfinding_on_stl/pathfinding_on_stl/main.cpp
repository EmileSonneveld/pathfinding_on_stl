#include <iostream>

#include "common.h"
#include "pathfindingOnStl.h"

#ifdef WIN32
// for using _CrtDumpMemoryLeaks
#   define _CRTDBG_MAP_ALLOC  
#   include <stdlib.h>  
#   include <crtdbg.h>  
#endif


void testCase(const stl::point& begin, const stl::point& goal, const std::string& stlFileName) // stl::point begin, 
{
	auto stlData = stl::parse_stl(stlFileName);

	std::vector<stl::triangle> triangles = stlData.triangles;
	log() << "\nstlFileName: " << stlFileName << "\n";
	log() << "#triangles = " << triangles.size() << "\n";

	auto result = calculatePath(begin, goal, stlData);

	log() << result << "\n";

	// For inspecting the resulting path.
	std::ofstream fs;
	fs.open(stlFileName + "_path.stl");
	fs << result.MakePlainTextStlFromGraph();
	fs.flush();
}


int main(int argc, char* argv[]) {

#ifdef WIN32
	//_CrtSetBreakAlloc(*place some value here to debug*);
#endif
	// The result of 'testCase' is not used
	testCase(stl::point(1.60291f, 1.36214f, 1), stl::point(0.602907f, 0.362136f, 1e-06f), "../../test_models/Box1x1x1.stl"); // 2, 6, 
	testCase(stl::point(0.602907f, 1.36214f, 1), stl::point(0.602907f, 0.362136f, 1e-06f), "../../test_models/Box1x1x1.stl"); // 3, 6, 
	testCase(stl::point(1.60291f, 0.362136f, 1e-06f), stl::point(1.60291f, 0.362136f, 1e-06f), "../../test_models/Box1x1x1.stl"); // 4, 4, 
	testCase(stl::point(2, 0, 0), stl::point(2, 3, 0), "../../test_models/test_mesh.stl"); // 0, 4, 
	testCase(stl::point(1, 0, 1), stl::point(2, 3, 2), "../../test_models/separated_triangles.stl"); // should not find a path // 1, 4, 

	//testCase(stl::point(1.60291f, 1.36214f, 1), stl::point(0, 0, 0), "../../test_models/BRACKET_EVO_II.stl"); // 1, 8, 
	testCase(stl::point(38.4102440f, 107.106583f, 87.8145447f), stl::point(42.0392914f, 109.200668f, 91.9542694f), "../../test_models/BRACKET_EVO_II.STL"); // 1, 8, 
	testCase(stl::point(3.20270014f, 4.95758295f, 5.68166208f), stl::point(3.49857974f, -2.93590450f, 7.96826077f), "../../test_models/stanford_dragon_flat_base.stl"); // 1, 8, 
	testCase(stl::point(-5.11849499f, -4.07326889f, 62.6000862f), stl::point(-5.11849451f, -6.39886713f, 64.0412750f), "../../test_models/voronoi_bunny_with_loop.stl"); // 1, 90, 

	std::cin.get();

#ifdef WIN32
	_CrtDumpMemoryLeaks(); // To verify no memory was leaked.
#endif
}
