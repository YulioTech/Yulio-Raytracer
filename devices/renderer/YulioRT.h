#pragma once

#if defined(DLL_EXPORT)
#define DllApi extern "C" __declspec(dllexport) 
#else
#define DllApi extern "C" __declspec(dllimport) 
#endif

namespace Yulio {

	enum ErrorCodeRT
	{
		RT_SUCCESS = 0,
		RT_MISSING_COLLADA,
		RT_FAILED_UNKNOWN
	};

	struct ParamsRT {
		char *renderer = "pathtracer";		// type of the renderer(don't change)
		int size = 1536; 					// single cube face image resolution(should be 1:1 ration; for testing puposes, can be reduced to speed up the rendering)
		int depth = 10; 					// max number of secondary ray bounces(will mostly affect translucent materials, e.g.glass)
		float tMaxShadowRay = 120.f; 		// max length of shadow rays(will affect how dark / bright the scene will appear in the rendered image)
		int spp = 32;						// number of samples per pixel(the higher the number, the less noisy the rendered image will be; has to be a pow of 2 - otherwise will be floored to the nearest power of 2)
		float ambientlight[3] = { .83f, .95f, .98f }; // lighter blue sky color
		float eyeSeparation = 2.5f;			// distance between the eyes in inches(2.5 is the default)
		bool toeIn = true; 					// forces a toe - in stereoscopic camera if present, with the zero parallax value specified separately
		float zeroParallax = 75.f;			// zero parallax depth from the view point in inches(only active when the toeIn flag is present)
		bool debug = false;					// if present, enables debug the debug mode (i.e. keeping the intermediate image files, more detailed logging, etc.) 
	};

	DllApi bool StartRT(const char* colladaFile, const ParamsRT* params);
	DllApi bool StopRT(bool keepResults);
	DllApi ErrorCodeRT GetLastErrorRT();
	//DllApi char* ErrorCodeToString(ErrorCodeRT errorCode);
}