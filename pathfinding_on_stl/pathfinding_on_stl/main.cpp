#include <iostream>

#include "common.h"
#include "pathfindingOnStl.h"

#ifdef WIN32
// for using _CrtDumpMemoryLeaks
#   define _CRTDBG_MAP_ALLOC  
#   include <stdlib.h>  
#   include <crtdbg.h>  
#endif


void testCase(int begin, int goal, std::string stlFileName)
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
		// Nothing is done with the result of 
	testCase(2, 6, "../../test_models/Box1x1x1.stl");
	testCase(3, 6, "../../test_models/Box1x1x1.stl");
	testCase(4, 4, "../../test_models/Box1x1x1.stl");
	testCase(0, 4, "../../test_models/test_mesh.stl");
	testCase(1, 4, "../../test_models/separated_triangles.stl"); // should not find a path

	testCase(1, 8, "../../test_models/BRACKET_EVO_II.stl");
	testCase(1, 8, "../../test_models/BRACKET_EVO_II.STL");
	testCase(1, 8, "../../test_models/stanford_dragon_flat_base.stl");
	testCase(1, 90, "../../test_models/voronoi_bunny_with_loop.stl");

	std::cin.get();

#ifdef WIN32
	_CrtDumpMemoryLeaks(); // To verify no memory was leaked.
#endif
}
