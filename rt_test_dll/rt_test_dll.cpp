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
	params.size = 1536;
	params.spp = 64;
	params.jpegQuality = 90;
	params.debug = false;
	if (!StartRT("../../models/Kia.dae", &params)) {
		auto error = GetLastErrorRT();
	}

	std::this_thread::sleep_for(std::chrono::seconds(20));

	StopRT(true);

    return 0;
}

