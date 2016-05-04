// rt_test_dll.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "../devices/renderer/YulioRT.h"

#include <thread>
#include <chrono>

using namespace Yulio;

int main()
{
	ParamsRT params;
	//params.renderer = "pathtracer";
	params.size = 128;
	params.spp = 1;
	params.jpegQuality = 90;
	params.debug = false;
	char *colladaFile = "../../models/Kia.dae";
	//char *colladaFile = "../../models/treeAlone_tempo_to_execute.dae";

	const auto nInterations = 10;
	for (auto i = 0; i < nInterations; ++i) {
		if (!StartRT(colladaFile, &params)) {
			auto error = GetLastErrorRT();
		}
#if 1
		WaitRT();
#else
		std::this_thread::sleep_for(std::chrono::seconds(5));
		StopRT(true);
#endif
	}

    return 0;
}

