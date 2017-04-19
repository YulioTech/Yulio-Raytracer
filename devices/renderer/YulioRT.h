#pragma once

#if defined(YULIO_DLL_EXPORT)
#define DllApi extern "C" __declspec(dllexport) 
#else
#define DllApi extern "C" __declspec(dllimport) 
#endif

namespace Yulio {

	enum ErrorCodeRT {
		NoError = 0,
		RenderingIsInProgress,
		MissingColladaFile,
		InvalidColladaFormat,
		UnitializedRenderer,
		FailedToPopulateStatus,
		UnknownError = 1000
	};

	enum StateRT {
		Inactive,
		Initialiazing,
		Rendering,
		Stopped,
		Done
	};

	struct StatusRT {
	
		StateRT state;
		float progress; // A relative progress value in the range [0.0:1.0]
		ErrorCodeRT lastError;
	};

	struct ParamsRT {
		char *renderer = "pathtracer";		// type of the renderer (currently supported values are "pathtracer" and "gpt"; gpt only works if the GPT code is included and GPT_RENDERER macro is defined in the device_singleray project) 
		int size = 1536; 					// single cube face image resolution (should be a 1:1 ratio; for testing purposes, can be reduced to speed up the rendering)
		int depth = 10; 					// max number of secondary ray bounces (will mostly affect translucent materials, e.g.glass)
		float tMaxShadowRay = 120.f; 		// max length of shadow rays (will affect how dark / bright the scene will appear in the rendered image)
		int spp = 256;						// number of samples per pixel(the higher the number, the less noisy the rendered image will be; has to be a pow of 2 - otherwise will be floored to the nearest power of 2)
		float ambientlight[3] = { .83f, .95f, .98f }; // lighter blue sky color
		float eyeSeparation = 2.5f;			// distance between the eyes in inches (2.5 is the default)
		bool toeIn = true; 					// forces a toe-in stereoscopic camera if present, with the zero parallax value specified separately
		float zeroParallax = 75.f;			// zero parallax depth from the view point in inches(only active when the toeIn flag is present)
		int jpegQuality = 90;				// JPEG compression quality (1-100 range)
		bool debug = false;					// if present, enables debug the debug mode (i.e. keeping the intermediate image files, more detailed logging, etc.)
		int threadsPriority = 0;			// worker threads priority as defined in Windows SetThreadPriorityfunction (i.e. 0 - is normal, -1 - below normal, etc.)
		bool waterMark = false;				// if true, adds a watermark to the rendered images
		char *faceCullingMode = "default";	// face culling mode : "default", "forcesingle" or "forcedouble"
	};

	DllApi bool StartRT(const char* colladaFile, const ParamsRT* params);
	DllApi bool WaitRT();
	DllApi bool StopRT(bool keepResults);
	DllApi ErrorCodeRT GetLastErrorRT();
	DllApi void GetCurrentStatusRT(StatusRT *status);
}