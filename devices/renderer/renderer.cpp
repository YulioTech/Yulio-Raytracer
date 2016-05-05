// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "sys/platform.h"
#include "sys/filename.h"
#include "image/image.h"
#include "lexers/streamfilters.h"
#include "lexers/parsestream.h"
#include "device/loaders/loaders.h"
#include "glutdisplay.h"

#include <iomanip>
#include <filesystem>
#include <thread>
#include <atomic>
#include <mutex>
#include "YulioRT.h" // Exported async DLL API definition 

namespace Yulio {

	using namespace embree;
	using namespace std;

	// A shared mutex used by the Yulio code
	std::mutex g_yulio_mutex;
#define YULIO_CRITIAL_SECTION lock_guard<std::mutex> guard(Yulio::g_yulio_mutex)

	class YulioStatusTracker {
	private:
		int numberOfStages = 0, currentStage = 0;
		StatusRT status;
		vector<ErrorCodeRT> allErrors;
#if defined (_DEBUG)
		const bool debug = true;
#else
		const bool debug = false;
#endif

	private:
		string stateToString() const {
			string s = "Unknown";
			switch (status.state) {
			case Inactive:
				s = "Inactive";
				break;
			case Initialiazing:
				s = "Initialiazing";
				break;
			case Rendering:
				s = "Rendering";
				break;
			case Stopped:
				s = "Stopped";
				break;
			case Done:
				s = "Done";
				break;
			}
			return s;
		}

	public:
		YulioStatusTracker(int numberOfStages = 0) {
			Init(numberOfStages);
		}

		void Reset() {
			status.lastError = NoError;
			status.progress = 0.f;
			status.state = Inactive;
			allErrors.clear();

			if (debug) cout << *this << endl;
		}

		void Init(int numberOfStages_) {
			numberOfStages = numberOfStages_;
			currentStage = 0;
			if (numberOfStages == 0) {
				Reset();
			}
			else {
				if (debug) cout << *this << endl;
			}
		}

		void SetCurrentState(StateRT state) {
			YULIO_CRITIAL_SECTION;
			status.state = state;

			switch (status.state)
			{
			case Inactive:
				Reset();
				break;
			case Stopped: case Done:
				status.progress = 1.f;
				break;
			default:
				break;
			}

			if (debug) cout << *this << endl;
		}

		void SetCurrentStage(int stage) {
			YULIO_CRITIAL_SECTION;
			currentStage = stage < numberOfStages ? stage : currentStage;

			if (debug) cout << *this << endl;
		}

		void UpdateCurrentStageProgress(float stageProgress) {
			YULIO_CRITIAL_SECTION;
			if (numberOfStages <= 0) return;

			const float baseProgress = float(currentStage) / numberOfStages;
			const float maxStageProgress = rcp(float(numberOfStages));
			status.progress = baseProgress + stageProgress * maxStageProgress;

			if (debug) cout << *this << endl;
		}

		void AddError(ErrorCodeRT error) {
			YULIO_CRITIAL_SECTION;
			status.lastError = error;
			allErrors.push_back(error);

			if (debug) cout << *this << endl;
		}

		StatusRT GetCurrentStatus() {
			YULIO_CRITIAL_SECTION;
			return status;
		}

		ErrorCodeRT GetLastError() {
			YULIO_CRITIAL_SECTION;
			return status.lastError;
		}

		friend std::ostream& operator<<(std::ostream& cout, const YulioStatusTracker &tracker);
	};

	__forceinline std::ostream& operator<<(std::ostream& cout, const YulioStatusTracker &tracker) {
		return cout << "[ State = " << tracker.stateToString() <<
			std::fixed << std::setw(10) << std::setprecision(2) << "; percentage done = " << tracker.status.progress <<
			"; last error = " << tracker.status.lastError << endl <<
			"; number of stages = " << tracker.numberOfStages <<
			"; current stage = " << tracker.currentStage <<
			" ]" << endl;
	}

	YulioStatusTracker yulioStatusTracker;

	atomic<bool>	yulioRunning = false;
	atomic<bool>	yulioStop = false;
	atomic<bool>	yulioKeepResults = false;

	void rsc(const RendererStatus &status) {
		yulioStatusTracker.UpdateCurrentStageProgress(status.progress);
	}
}

namespace embree
{
	//double upload_time = 0;
	/******************************************************************************/
	/*                                  State                                     */
	/******************************************************************************/

	/* camera settings */
	Vector3f g_camPos = Vector3f(0.0f, 0.0f, 0.0f);
	Vector3f g_camLookAt = Vector3f(1.0f, 0.0f, 0.0f);
	Vector3f g_camUp = Vector3f(0, 1, 0);
	float g_camFieldOfView = 64.0f;
	float g_camRadius = 0.0f;
	bool g_stereo = false;
	bool g_processingFprCollada = false;
	float g_eyeSeparation = 6.35f * 0.393701f;
	bool g_toeIn = false;
	float g_zeroParallaxDistance = g_eyeSeparation * 30.f;
	float g_tMaxShadowRay = inf;
	float g_sceneScale = 1.f;

	/* rendering device and global handles */
	Handle<Device::RTRenderer> g_renderer = nullptr;
	Handle<Device::RTToneMapper> g_tonemapper = nullptr;
	Handle<Device::RTFrameBuffer> g_frameBuffer = nullptr;
	Handle<Device::RTImage> g_backplate = nullptr;
	Handle<Device::RTScene> g_render_scene = nullptr;
	std::vector<Handle<Device::RTPrimitive>> g_prims;
	std::vector<Handle<Device::RTCamera>> g_stereoCubeCameras;
	size_t g_stereoCubeCameraIndex = 1 * 12; // Used for real-time rendering (i.e. for testing)

	/* rendering settings */
	std::string g_scene = "default";
	std::string g_accel = "default";
	std::string g_builder = "default";
	std::string g_traverser = "default";
	int g_depth = -1;                       //!< recursion depth
	int g_spp = 1;                          //!< samples per pixel for ordinary rendering

	/* output settings */
	int g_numBuffers = 1;                   //!< number of buffers of the framebuffer
	bool g_rendered = false;                //!< set to true after rendering
	int g_refine = 1;                       //!< refinement mode
	float g_gamma = 1.0f;
	bool g_vignetting = false;
	bool g_fullscreen = false;
	size_t g_width = 512;
	size_t g_height = 512;
	std::string g_format = "RGB8";
	std::string g_rtcore_cfg = "";
	std::string g_outFileName = "";
	std::string g_workingDirectory = "";
	std::string g_sceneFileName = "";
	size_t g_num_frames = 1; // number of frames to render in output mode
	size_t g_numThreads = 0;// 1; // 0 means auto-detect
	size_t g_verbose_output = 0;
	int g_jpegQuality = 90;

	/* regression testing mode */
	bool g_regression = false;

	/* logging settings */
	extern bool log_display;
	bool g_profiling = false;
	bool g_debugging = false;

	/******************************************************************************/
	/*                            Object Creation                                 */
	/******************************************************************************/

	Handle<Device::RTCamera> createCamera(const AffineSpace3f& space)
	{
		/*! pinhole camera */
		if (g_camRadius == 0.0f) {
			Handle<Device::RTCamera> camera = g_device->rtNewCamera("pinhole");
			g_device->rtSetTransform(camera, "local2world", copyToArray(space));
			g_device->rtSetFloat1(camera, "angle", g_camFieldOfView);
			g_device->rtSetFloat1(camera, "aspectRatio", float(g_width) / float(g_height));
			g_device->rtCommit(camera);
			return camera;
		}
		/*! depth of field camera */
		else {
			Handle<Device::RTCamera> camera = g_device->rtNewCamera("depthoffield");
			g_device->rtSetTransform(camera, "local2world", copyToArray(space));
			g_device->rtSetFloat1(camera, "angle", g_camFieldOfView);
			g_device->rtSetFloat1(camera, "aspectRatio", float(g_width) / float(g_height));
			g_device->rtSetFloat1(camera, "lensRadius", g_camRadius);
			g_device->rtSetFloat1(camera, "focalDistance", length(g_camLookAt - g_camPos));
			g_device->rtCommit(camera);
			return camera;
		}
	}

	Handle<Device::RTScene> createScene()
	{
		Handle<Device::RTScene> scene = g_device->rtNewScene(g_scene.c_str());
		g_device->rtSetString(scene, "accel", g_accel.c_str());
		g_device->rtSetString(scene, "builder", g_builder.c_str());
		g_device->rtSetString(scene, "traverser", g_traverser.c_str());
		for (size_t i = 0; i < g_prims.size(); i++) g_device->rtSetPrimitive(scene, i, g_prims[i]);
		g_device->rtCommit(scene);
		return scene;
	}

	void setLight(Handle<Device::RTPrimitive> light)
	{
		if (!g_render_scene) return;
		g_device->rtSetPrimitive(g_render_scene, g_prims.size(), light);
		g_device->rtCommit(g_render_scene);
	}

	void createGlobalObjects()
	{
		g_renderer = g_device->rtNewRenderer("pathtracer");
		if (g_depth >= 0) g_device->rtSetInt1(g_renderer, "maxDepth", g_depth);
		g_device->rtSetFloat1(g_renderer, "tMaxShadowRay", g_tMaxShadowRay);
		g_device->rtSetInt1(g_renderer, "sampler.spp", g_spp);
		g_device->rtCommit(g_renderer);

		g_tonemapper = g_device->rtNewToneMapper("default");
		g_device->rtSetFloat1(g_tonemapper, "gamma", g_gamma);
		g_device->rtSetBool1(g_tonemapper, "vignetting", g_vignetting);
		g_device->rtCommit(g_tonemapper);

		g_frameBuffer = g_device->rtNewFrameBuffer(g_format.c_str(), g_width, g_height, g_numBuffers);
		g_backplate = nullptr;
	}

	void clearGlobalObjects() {
		g_rendered = false;

		g_renderer = null;
		g_tonemapper = null;
		g_frameBuffer = null;
		g_backplate = null;
		g_prims.clear();
		g_stereoCubeCameras.clear();
		g_render_scene = null;
		rtClearTextureCache();
		rtClearImageCache();

		//std::this_thread::sleep_for(std::chrono::seconds(5));
		delete g_device;
		g_device = nullptr;
	}

	/******************************************************************************/
	/*                      Command line parsing                                  */
	/******************************************************************************/

	static void parseDebugRenderer(Ref<ParseStream> cin, const FileName& path)
	{
		g_renderer = g_device->rtNewRenderer("debug");
		if (g_depth >= 0) g_device->rtSetInt1(g_renderer, "maxDepth", g_depth);
		g_device->rtSetInt1(g_renderer, "sampler.spp", g_spp);

		if (cin->peek() != "{") goto finish;
		cin->drop();

		while (cin->peek() != "}") {
			std::string tag = cin->getString();
			cin->force("=");
			if (tag == "depth") g_device->rtSetInt1(g_renderer, "maxDepth", cin->getInt());
			else std::cout << "unknown tag \"" << tag << "\" in debug renderer parsing" << std::endl;
		}
		cin->drop();

	finish:
		g_device->rtCommit(g_renderer);
	}

	static void parsePathTracer(Ref<ParseStream> cin, const FileName& path, std::atomic<bool> *stopFlag, RendererStatusCallback *rsc)
	{
		g_renderer = g_device->rtNewRenderer("pathtracer");
		if (g_depth >= 0) g_device->rtSetInt1(g_renderer, "maxDepth", g_depth);
		g_device->rtSetFloat1(g_renderer, "tMaxShadowRay", g_tMaxShadowRay);
		g_device->rtSetInt1(g_renderer, "sampler.spp", g_spp);
		g_device->rtSetPointer(g_renderer, "stopFlag", stopFlag);
		g_device->rtSetPointer(g_renderer, "statusCallback", rsc);

		if (g_backplate) g_device->rtSetImage(g_renderer, "backplate", g_backplate);

		if (cin->peek() != "{") goto finish;
		cin->drop();

		while (cin->peek() != "}") {
			std::string tag = cin->getString();
			cin->force("=");
			if (tag == "depth") g_device->rtSetInt1(g_renderer, "maxDepth", cin->getInt());
			else if (tag == "tMaxShadowRay") g_device->rtSetFloat1(g_renderer, "tMaxShadowRay", cin->getFloat() * g_sceneScale);
			else if (tag == "spp") g_device->rtSetInt1(g_renderer, "sampler.spp", cin->getInt());
			else if (tag == "minContribution") g_device->rtSetFloat1(g_renderer, "minContribution", cin->getFloat());
			else if (tag == "backplate") g_device->rtSetImage(g_renderer, "backplate", rtLoadImage(path + cin->getFileName()));
			else std::cout << "unknown tag \"" << tag << "\" in debug renderer parsing" << std::endl;
		}
		cin->drop();

	finish:
		g_device->rtCommit(g_renderer);
	}

	static void parseGPT(Ref<ParseStream> cin, const FileName& path, std::atomic<bool> *stopFlag, RendererStatusCallback *rsc)
	{
		g_renderer = g_device->rtNewRenderer("gpt");
		if (g_depth >= 0) g_device->rtSetInt1(g_renderer, "maxDepth", g_depth);
		g_device->rtSetFloat1(g_renderer, "tMaxShadowRay", g_tMaxShadowRay);
		g_device->rtSetInt1(g_renderer, "sampler.spp", g_spp);
		g_device->rtSetPointer(g_renderer, "stopFlag", stopFlag);
		g_device->rtSetPointer(g_renderer, "statusCallback", rsc);
		if (g_backplate) g_device->rtSetImage(g_renderer, "backplate", g_backplate);

		if (cin->peek() != "{") goto finish;
		cin->drop();

		while (cin->peek() != "}") {
			std::string tag = cin->getString();
			cin->force("=");
			if (tag == "depth") g_device->rtSetInt1(g_renderer, "maxDepth", cin->getInt());
			else if (tag == "tMaxShadowRay") g_device->rtSetFloat1(g_renderer, "tMaxShadowRay", cin->getFloat() * g_sceneScale);
			else if (tag == "spp") g_device->rtSetInt1(g_renderer, "sampler.spp", cin->getInt());
			else if (tag == "minContribution") g_device->rtSetFloat1(g_renderer, "minContribution", cin->getFloat());
			else if (tag == "backplate") g_device->rtSetImage(g_renderer, "backplate", rtLoadImage(path + cin->getFileName()));
			else std::cout << "unknown tag \"" << tag << "\" in debug renderer parsing" << std::endl;
		}
		cin->drop();

	finish:
		g_device->rtCommit(g_renderer);
	}

	static void displayMode()
	{
		if (!g_renderer) throw std::runtime_error("no renderer set");

		AffineSpace3f camSpace = one;
		//if (!g_stereo) {
			//g_camPos.y = 50.f;
			//g_camPos.z -= 100.f;
			//g_camLookAt.y = 50.f;
			//g_camLookAt.z -= 100.f;

			camSpace = AffineSpace3f::lookAtPoint(g_camPos, g_camLookAt, g_camUp);
		//}
		//else {
		//	auto stereoCubeCamera = g_cameras[g_cameraIndex];

		//	g_device->rtGetFloat3(stereoCubeCamera, "origin", g_camPos.x, g_camPos.y, g_camPos.z);
		//	g_device->rtGetFloat3(stereoCubeCamera, "lookAt", g_camLookAt.x, g_camLookAt.y, g_camLookAt.z);
		//	g_device->rtGetFloat3(stereoCubeCamera, "up", g_camUp.x, g_camUp.y, g_camUp.z);

		//	g_device->rtGetTransform(stereoCubeCamera, "local2world", &camSpace);
		//	//camSpace = AffineSpace3f::lookAtPoint(g_camPos, g_camLookAt, g_camUp);
		//}

		const auto speed = fmaxf(1.0f, 0.05f * length(g_camLookAt - g_camPos));
		Handle<Device::RTScene> scene = createScene();
		GLUTDisplay(camSpace, speed, scene);
		g_rendered = true;
	}

	static std::string getBasePath(const std::string &path) {
		auto pos = path.find_last_of("\\/");
		return (std::string::npos == pos) ? "" : path.substr(0, pos + 1);
	}

	static void outputMode(const FileName &fileName)
	{
		Yulio::yulioStatusTracker.SetCurrentState(Yulio::Rendering);

		if (!g_renderer) {
			Yulio::yulioStatusTracker.AddError(Yulio::UnitializedRenderer);
			throw std::runtime_error("no renderer set");
		}

		// Special handling of the stereoscopic render mode
		if (g_stereo) {
			Handle<Device::RTScene> scene = createScene();

			if (g_stereoCubeCameras.size() > 0) {

				// Init the number of stages (i.e. camera views)
				Yulio::yulioStatusTracker.Init(g_stereoCubeCameras.size());

				std::vector<Ref<Image>> stereCubeFaceImages;
				std::vector<std::string> savedImages;

				// Make sure the cube face images are square
				if (g_width != g_height) {
					g_width = g_height = max(g_width, g_height);
					g_frameBuffer = g_device->rtNewFrameBuffer(g_format.c_str(), g_width, g_height, g_numBuffers);
				}

#if 0
				// For testing
				const size_t testCameraIndex = 2;
				for (size_t i = (testCameraIndex - 1) * 12; i < testCameraIndex * 12; ++i) {
#else
				for (size_t i = 0; i < g_stereoCubeCameras.size() && !Yulio::yulioStop; ++i) {
#endif
					// Set the current stage
					Yulio::yulioStatusTracker.SetCurrentStage(i);

					const auto stereoCubeCamera = g_stereoCubeCameras[i];

					// Update the dynamic geometry (e.g. self-aligning instances, etc.)
					{
						Vector3f camPos;
						g_device->rtGetFloat3(stereoCubeCamera, "origin", camPos.x, camPos.y, camPos.z);

						for (size_t i = 0; i < g_prims.size(); i++)
							g_device->rtUpdatePrimitive(scene, i, g_prims[i], camPos, g_camUp);

						g_device->rtCommit(scene);
					}

					std::string cameraName;
					g_device->rtGetString(stereoCubeCamera, "name", cameraName);

					const auto cameraIndex = i / 12;
					const auto cubeFaceIndex = i % 12;

					if (cubeFaceIndex == 0) stereCubeFaceImages.clear();

					bool cameraParamsChanged = false;

					if (g_toeIn) {
						g_device->rtSetBool1(stereoCubeCamera, "toeIn", g_toeIn);
						cameraParamsChanged = true;
					}

					if (cameraParamsChanged) g_device->rtCommit(stereoCubeCamera);

					g_device->rtRenderFrame(g_renderer, stereoCubeCamera, scene, g_tonemapper, g_frameBuffer, 0);

					for (int i = 0; i < g_numBuffers; i++)
						g_device->rtSwapBuffers(g_frameBuffer);

					// Override the file name
					const FileName fileName = g_sceneFileName;

					std::string cubeFaceFileName = std::string(fileName.path()) + "\\" + fileName.name() + "_" + cameraName + "_";
					const std::string eyeName = cubeFaceIndex < 6 ? "left" : "right";
					switch (cubeFaceIndex % 6) {
					case 0:
						// Front image
						cubeFaceFileName += "front_image_";
						break;

					case 1:
						// Right image
						cubeFaceFileName += "right_image_";
						break;

					case 2:
						// Back image
						cubeFaceFileName += "back_image_";
						break;

					case 3:
						// Left image
						cubeFaceFileName += "left_image_";
						break;

					case 4:
						// Up image
						cubeFaceFileName += "top_image_";
						break;

					case 5:
						// Down image
						cubeFaceFileName += "bottom_image_";
						break;
					}

					cubeFaceFileName += eyeName + ".jpg";

					// Create a cube face image
					{
						auto ptr = g_device->rtMapFrameBuffer(g_frameBuffer);
						Ref<Image> image = null;
						if (g_format == "RGB8")  image = new Image3c(g_width, g_height, (Col3c*)ptr);
						else if (g_format == "RGBA8")  image = new Image4c(g_width, g_height, (Col4c*)ptr);
						else if (g_format == "RGB_FLOAT32")  image = new Image3f(g_width, g_height, (Col3f*)ptr);
						else if (g_format == "RGBA_FLOAT32")  image = new Image4f(g_width, g_height, (Col4f*)ptr);
						else throw std::runtime_error("unsupported framebuffer format: " + g_format);
						g_device->rtUnmapFrameBuffer(g_frameBuffer);

						stereCubeFaceImages.push_back(image);

						// Store the cube face image to disk if needed (for debugging purposes)
						if (g_debugging) {
							storeImage(image, cubeFaceFileName, g_jpegQuality);
							savedImages.push_back(cubeFaceFileName);
						}
					}

					// Assemble the final cube map image
					// The order is: left right up down back front (6 images for the left eye, then 6 for the right eye respectively)
					if (cubeFaceIndex == 11) {
						Ref<Image> finalImage = null;
						if (g_format == "RGB8")  finalImage = new Image3c(g_width * 12, g_height);
						else if (g_format == "RGBA8")  finalImage = new Image4c(g_width * 12, g_height);
						else if (g_format == "RGB_FLOAT32")  finalImage = new Image3f(g_width * 12, g_height);
						else if (g_format == "RGBA_FLOAT32")  finalImage = new Image4f(g_width * 12, g_height);
						else throw std::runtime_error("unsupported framebuffer format: " + g_format);

						for (size_t y = 0, i = 0; y < finalImage->height; ++y) {
							for (size_t x = 0; x < finalImage->width; ++x) {
								const size_t finalImageSegment = x / g_width;
								const size_t xCubeFaceOffset = x % g_width; 
								const auto eyeIndex = finalImageSegment / 6 == 0 ? 1 : 0;
								size_t cubeFaceIndex = 6 * eyeIndex;
								switch (finalImageSegment % 6) {

								case 0:
									// Left image
									cubeFaceIndex += 3;
									break;

								case 1:
									// Right image
									cubeFaceIndex += 1;
									break;

								case 2:
									// Up image
									cubeFaceIndex += 4;
									break;

								case 3:
									// Down image
									cubeFaceIndex += 5;
									break;

								case 4:
									// Back image
									cubeFaceIndex += 2;
									break;

								case 5:
									// Front image
									cubeFaceIndex += 0;
									break;
								}

								const Color4 pixel = stereCubeFaceImages[cubeFaceIndex]->get(xCubeFaceOffset, y);
								finalImage->set(x, y, pixel);
							}
						}

						// Store the final image
						{
							const auto finalFileName = std::string(fileName.path()) + "\\" + fileName.name() + "_" + cameraName + "." + "jpg"; // fileName.ext();
							storeImage(finalImage, finalFileName, g_jpegQuality);
							savedImages.push_back(cubeFaceFileName);

							std::cout << "Generated stereoscopic cube map #" << (cameraIndex + 1) << " in file " << finalFileName << std::endl << std::endl;
						}
					}

					// Stop if abort is requested
					if (Yulio::yulioStop) {
						if (!Yulio::yulioKeepResults) {
							// Remove all the images written so far
							for (auto imageFile : savedImages) {
								std::experimental::filesystem::remove(imageFile);
							}
						}
						break;
					}
				}

				Yulio::yulioStatusTracker.SetCurrentState(Yulio::yulioStop ? Yulio::Stopped : Yulio::Done);

			}
			else if (!g_processingFprCollada) {
				auto camSpaceOrigin = AffineSpace3f::lookAtPoint(g_camPos, g_camLookAt, g_camUp);

				std::vector<Ref<Image>> stereCubeFaceImages;

				for (int i = 0; i < 12; ++i) {
					Handle<Device::RTCamera> stereoCubeCamera = g_device->rtNewCamera("stereo");
					g_device->rtSetTransform(stereoCubeCamera, "local2world", copyToArray(camSpaceOrigin));
					g_device->rtSetInt1(stereoCubeCamera, "cubeFaceIndex", i);
					g_device->rtSetFloat3(stereoCubeCamera, "origin", g_camPos.x, g_camPos.y, g_camPos.z);
					g_device->rtSetFloat3(stereoCubeCamera, "lookAt", g_camLookAt.x, g_camLookAt.y, g_camLookAt.z);
					g_device->rtSetFloat3(stereoCubeCamera, "up", g_camUp.x, g_camUp.y, g_camUp.z);
					//g_device->rtSetFloat1(stereoCubeCamera, "eyeSeparation", 20.f); // for testing
					g_device->rtSetBool1(stereoCubeCamera, "toeIn", g_toeIn);
					//g_device->rtSetFloat1(stereoCubeCamera, "zeroParallaxDistance", 200.f);
					g_device->rtCommit(stereoCubeCamera);

					g_device->rtRenderFrame(g_renderer, stereoCubeCamera, scene, g_tonemapper, g_frameBuffer, 0);

					for (int i = 0; i < g_numBuffers; i++)
						g_device->rtSwapBuffers(g_frameBuffer);

					std::string cubeFaceFileName = std::string(fileName.path()) + "\\" + fileName.name() + "_";
					const std::string eyeName = i < 6 ? "left" : "right";
					switch (i % 6) {
					case 0:
						// Front image
						cubeFaceFileName += "front_image_";
						break;

					case 1:
						// Right image
						cubeFaceFileName += "right_image_";
						break;

					case 2:
						// Back image
						cubeFaceFileName += "back_image_";
						break;

					case 3:
						// Left image
						cubeFaceFileName += "left_image_";
						break;

					case 4:
						// Up image
						cubeFaceFileName += "top_image_";
						break;

					case 5:
						// Down image
						cubeFaceFileName += "bottom_image_";
						break;
					}

					cubeFaceFileName += eyeName + "." + fileName.ext();

					/* store to disk */
					{
						auto ptr = g_device->rtMapFrameBuffer(g_frameBuffer);
						Ref<Image> image = null;
						if (g_format == "RGB8")  image = new Image3c(g_width, g_height, (Col3c*)ptr);
						else if (g_format == "RGBA8")  image = new Image4c(g_width, g_height, (Col4c*)ptr);
						else if (g_format == "RGB_FLOAT32")  image = new Image3f(g_width, g_height, (Col3f*)ptr);
						else if (g_format == "RGBA_FLOAT32")  image = new Image4f(g_width, g_height, (Col4f*)ptr);
						else throw std::runtime_error("unsupported framebuffer format: " + g_format);
						g_device->rtUnmapFrameBuffer(g_frameBuffer);

						stereCubeFaceImages.push_back(image);

						// Store the cube face image to disk if needed (for debugging purposes)
						if (g_debugging) {
							storeImage(image, cubeFaceFileName, g_jpegQuality);
						}
					}
				}

				// Assemble the final cube map image
				// The order is: left right up down back front (6 images for the left eye, then 6 for the right eye respectively)
				{
					//uint32_t xOffset;
					//const uint32_t xStride = g_width * 6;
					Ref<Image> finalImage = null;
					if (g_format == "RGB8")  finalImage = new Image3c(g_width * 12, g_height);
					else if (g_format == "RGBA8")  finalImage = new Image4c(g_width * 12, g_height);
					else if (g_format == "RGB_FLOAT32")  finalImage = new Image3f(g_width * 12, g_height);
					else if (g_format == "RGBA_FLOAT32")  finalImage = new Image4f(g_width * 12, g_height);
					else throw std::runtime_error("unsupported framebuffer format: " + g_format);

					for (size_t y = 0, i = 0; y < finalImage->height; ++y) {
						for (size_t x = 0; x < finalImage->width; ++x) {
							const size_t finalImageSegment = x / g_width;
							const size_t xCubeFaceOffset = x % g_width;
							const auto eyeIndex = finalImageSegment / 6 == 0 ? 1 : 0;
							size_t cubeFaceIndex = 6 * eyeIndex;
							switch (finalImageSegment % 6) {

							case 0:
								// Left image
								cubeFaceIndex += 3;
								break;

							case 1:
								// Right image
								cubeFaceIndex += 1;
								break;

							case 2:
								// Up image
								cubeFaceIndex += 4;
								break;

							case 3:
								// Down image
								cubeFaceIndex += 5;
								break;

							case 4:
								// Back image
								cubeFaceIndex += 2;
								break;

							case 5:
								// Front image
								cubeFaceIndex += 0;
								break;
							}

							const Color4 pixel = stereCubeFaceImages[cubeFaceIndex]->get(xCubeFaceOffset, y);
							finalImage->set(x, y, pixel);
						}
					}

					storeImage(finalImage, fileName, g_jpegQuality);
				}
			}

			g_rendered = true;
		}
		else {
			/* render image */
			Handle<Device::RTCamera> camera = createCamera(AffineSpace3f::lookAtPoint(g_camPos, g_camLookAt, g_camUp));
			Handle<Device::RTScene> scene = createScene();
			g_device->rtSetInt1(g_renderer, "showprogress", 1);
			g_device->rtCommit(g_renderer);
			for (size_t i = 0; i < g_num_frames; i++)
				g_device->rtRenderFrame(g_renderer, camera, scene, g_tonemapper, g_frameBuffer, 0);
			for (int i = 0; i < g_numBuffers; i++)
				g_device->rtSwapBuffers(g_frameBuffer);

			/* store to disk */
			auto ptr = g_device->rtMapFrameBuffer(g_frameBuffer);
			Ref<Image> image = null;
			if (g_format == "RGB8")  image = new Image3c(g_width, g_height, (Col3c*)ptr);
			else if (g_format == "RGBA8")  image = new Image4c(g_width, g_height, (Col4c*)ptr);
			else if (g_format == "RGB_FLOAT32")  image = new Image3f(g_width, g_height, (Col3f*)ptr);
			else if (g_format == "RGBA_FLOAT32")  image = new Image4f(g_width, g_height, (Col4f*)ptr);
			else throw std::runtime_error("unsupported framebuffer format: " + g_format);
			storeImage(image, fileName, g_jpegQuality);
			g_device->rtUnmapFrameBuffer(g_frameBuffer);
			g_rendered = true;
		}
	}

	static std::string parseList(Ref<ParseStream> cin)
	{
		std::string str;
		if (cin->peek() != "" && cin->peek()[0] != '-') {
			str += cin->getString();
			while (cin->peek() != "" && cin->peek()[0] != '-') str += " " + cin->getString();
		}
		return str;
	}

	static void parseNumThreads(Ref<ParseStream> cin)
	{
		while (true)
		{
			std::string tag = cin->peek();
			if (tag == "-rtcore") {
				cin->getString();
				g_rtcore_cfg = cin->getString();
			}
			else if (tag == "-threads") {
				cin->getString();
				g_numThreads = cin->getInt();
				g_rtcore_cfg += ",threads=" + std::stringOf(g_numThreads);
			}

			/*! enable verbose output mode */
			else if (tag == "-verbose") {
				cin->getString();
				g_verbose_output = 1;
				g_rtcore_cfg += ",verbose=" + std::stringOf(g_verbose_output);
			}
			else break;
		}
	}

	static void parseDevice(Ref<ParseStream> cin)
	{
		std::string tag = cin->peek();
		if (tag == "") return;

		/* create network device */
		if (tag == "-connect") {
			cin->getString();
			clearGlobalObjects();
			if (g_format != "RGBA8") g_format = "RGB8";
			g_numBuffers = 2;
			std::string type = "network " + parseList(cin);
			g_device = Device::rtCreateDevice(type.c_str(), g_numThreads, g_rtcore_cfg.c_str());
			createGlobalObjects();
		}

		/* parse device */
		else if (tag == "-device") {
			cin->getString();
			clearGlobalObjects();
			g_device = Device::rtCreateDevice(cin->getString().c_str(), g_numThreads, g_rtcore_cfg.c_str());
			createGlobalObjects();
		}
	}

	std::string makeFileName(const std::string path, const std::string fileName)
	{
		if (fileName[0] == '/') return fileName;
		if (path == "") return fileName;
		return path + "/" + fileName;
	}

	static void parseCommandLine(Ref<ParseStream> cin, const FileName &path)
	{
		// file name to write to -- "" means "display mode"
		while (true)
		{
			std::string tag = cin->getString();
			if (tag == "") return;

			/* parse command line parameters from a file */
			if (tag == "-c") {
				const FileName file = makeFileName(path, cin->getFileName());
				parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
			}

			/* turn off logging */
			else if (tag == "--no-logging") {
				log_display = false;
			}

			else if (tag == "-profiling") {
				g_profiling = true;
			}

			else if (tag == "-debug") {
				g_debugging = true;
			}

			/* read model from file */
			else if (tag == "-i") {
				g_sceneFileName = makeFileName(path, cin->getFileName());
				std::vector<Handle<Device::RTPrimitive>> prims = rtLoadScene(g_sceneFileName, &g_stereoCubeCameras);
				g_prims.insert(g_prims.end(), prims.begin(), prims.end());
			}

			/* triangulated sphere */
			else if (tag == "-trisphere")
			{
				Handle<Device::RTShape> sphere = g_device->rtNewShape("sphere");
				const Vector3f P = cin->getVector3f();
				g_device->rtSetFloat3(sphere, "P", P.x, P.y, P.z);
				g_device->rtSetFloat1(sphere, "r", cin->getFloat());
				g_device->rtSetInt1(sphere, "numTheta", cin->getInt());
				g_device->rtSetInt1(sphere, "numPhi", cin->getInt());
				g_device->rtCommit(sphere);

				Handle<Device::RTMaterial> material = g_device->rtNewMaterial("matte");
				g_device->rtSetFloat3(material, "reflection", 1.0f, 0.0f, 0.0f);
				g_device->rtCommit(material);
				g_prims.push_back(g_device->rtNewShapePrimitive(sphere, material, nullptr));
			}

			/* ambient light source */
			else if (tag == "-ambientlight") {
				Handle<Device::RTLight> light = g_device->rtNewLight("ambientlight");
				const Color L = cin->getColor();
				g_device->rtSetFloat3(light, "L", L.r, L.g, L.b);
				g_device->rtCommit(light);
				g_prims.push_back(g_device->rtNewLightPrimitive(light, nullptr, nullptr));
			}

			/* point light source */
			else if (tag == "-pointlight") {
				Handle<Device::RTLight> light = g_device->rtNewLight("pointlight");
				const Vector3f P = cin->getVector3f();
				const Color I = cin->getColor();
				g_device->rtSetFloat3(light, "P", P.x, P.y, P.z);
				g_device->rtSetFloat3(light, "I", I.r, I.g, I.b);
				g_device->rtCommit(light);
				g_prims.push_back(g_device->rtNewLightPrimitive(light, nullptr, nullptr));
			}

			else if (tag == "-masked_pointlight") {
				Handle<Device::RTLight> light = g_device->rtNewLight("pointlight");
				const Vector3f P = cin->getVector3f();
				const Color I = cin->getColor();
				int illumMask = cin->getInt();
				int shadowMask = cin->getInt();
				g_device->rtSetFloat3(light, "P", P.x, P.y, P.z);
				g_device->rtSetFloat3(light, "I", I.r, I.g, I.b);
				g_device->rtCommit(light);
				Handle<Device::RTPrimitive> prim = g_device->rtNewLightPrimitive(light, nullptr, nullptr);
				g_device->rtSetInt1(prim, "illumMask", illumMask);
				g_device->rtSetInt1(prim, "shadowMask", shadowMask);
				g_device->rtCommit(prim);
				g_prims.push_back(prim);
			}

			/* directional light source */
			else if (tag == "-directionallight" || tag == "-dirlight") {
				Handle<Device::RTLight> light = g_device->rtNewLight("directionallight");
				const Vector3f D = cin->getVector3f();
				const Color E = cin->getColor();
				g_device->rtSetFloat3(light, "D", D.x, D.y, D.z);
				g_device->rtSetFloat3(light, "E", E.r, E.g, E.b);
				g_device->rtCommit(light);
				g_prims.push_back(g_device->rtNewLightPrimitive(light, nullptr, nullptr));
			}

			/* distant light source */
			else if (tag == "-distantlight") {
				Handle<Device::RTLight> light = g_device->rtNewLight("distantlight");
				const Vector3f D = cin->getVector3f();
				const Color L = cin->getColor();
				g_device->rtSetFloat3(light, "D", D.x, D.y, D.z);
				g_device->rtSetFloat3(light, "L", L.r, L.g, L.b);
				g_device->rtSetFloat1(light, "halfAngle", cin->getFloat());
				g_device->rtCommit(light);
				g_prims.push_back(g_device->rtNewLightPrimitive(light, nullptr, nullptr));
			}

			/* spot light source */
			else if (tag == "-spotlight") {
				Handle<Device::RTLight> light = g_device->rtNewLight("spotlight");
				const Vector3f P = cin->getVector3f();
				const Vector3f D = cin->getVector3f();
				const Color I = cin->getColor();
				const float angleMin = cin->getFloat();
				const float angleMax = cin->getFloat();
				g_device->rtSetFloat3(light, "P", P.x, P.y, P.z);
				g_device->rtSetFloat3(light, "D", D.x, D.y, D.z);
				g_device->rtSetFloat3(light, "I", I.r, I.g, I.b);
				g_device->rtSetFloat1(light, "angleMin", angleMin);
				g_device->rtSetFloat1(light, "angleMax", angleMax);
				g_device->rtCommit(light);
				g_prims.push_back(g_device->rtNewLightPrimitive(light, nullptr, nullptr));
			}

			/* triangular light source */
			else if (tag == "-trianglelight") {
				Vector3f P = cin->getVector3f();
				Vector3f U = cin->getVector3f();
				Vector3f V = cin->getVector3f();
				Vector3f L = cin->getVector3f();

				Handle<Device::RTLight> light = g_device->rtNewLight("trianglelight");
				g_device->rtSetFloat3(light, "v0", P.x, P.y, P.z);
				g_device->rtSetFloat3(light, "v1", P.x + U.x, P.y + U.y, P.z + U.z);
				g_device->rtSetFloat3(light, "v2", P.x + V.x, P.y + V.y, P.z + V.z);
				g_device->rtSetFloat3(light, "L", L.x, L.y, L.z);
				g_device->rtCommit(light);
				g_prims.push_back(g_device->rtNewLightPrimitive(light, nullptr, nullptr));
			}

			/* quad light source */
			else if (tag == "-quadlight")
			{
				Vector3f P = cin->getVector3f();
				Vector3f U = cin->getVector3f();
				Vector3f V = cin->getVector3f();
				Vector3f L = cin->getVector3f();

				Handle<Device::RTLight> light0 = g_device->rtNewLight("trianglelight");
				g_device->rtSetFloat3(light0, "v0", P.x + U.x + V.x, P.y + U.y + V.y, P.z + U.z + V.z);
				g_device->rtSetFloat3(light0, "v1", P.x + U.x, P.y + U.y, P.z + U.z);
				g_device->rtSetFloat3(light0, "v2", P.x, P.y, P.z);
				g_device->rtSetFloat3(light0, "L", L.x, L.y, L.z);
				g_device->rtCommit(light0);
				g_prims.push_back(g_device->rtNewLightPrimitive(light0, nullptr, nullptr));

				Handle<Device::RTLight> light1 = g_device->rtNewLight("trianglelight");
				g_device->rtSetFloat3(light1, "v0", P.x + U.x + V.x, P.y + U.y + V.y, P.z + U.z + V.z);
				g_device->rtSetFloat3(light1, "v1", P.x, P.y, P.z);
				g_device->rtSetFloat3(light1, "v2", P.x + V.x, P.y + V.y, P.z + V.z);
				g_device->rtSetFloat3(light1, "L", L.x, L.y, L.z);
				g_device->rtCommit(light1);
				g_prims.push_back(g_device->rtNewLightPrimitive(light1, nullptr, nullptr));
			}

			/* HDRI light source */
			else if (tag == "-hdrilight")
			{
				Handle<Device::RTLight> light = g_device->rtNewLight("hdrilight");
				const Color L = cin->getColor();
				g_device->rtSetFloat3(light, "L", L.r, L.g, L.b);
				g_device->rtSetImage(light, "image", rtLoadImage(path + cin->getFileName()));
				g_device->rtCommit(light);
				g_prims.push_back(g_device->rtNewLightPrimitive(light, nullptr, nullptr));
			}

			/* parse camera parameters */
			else if (tag == "-vp")     g_camPos = Vector3f(cin->getVector3f());
			else if (tag == "-vi")     g_camLookAt = Vector3f(cin->getVector3f());
			else if (tag == "-vd")     g_camLookAt = g_camPos + cin->getVector3f();
			else if (tag == "-vu")     g_camUp = cin->getVector3f();
			else if (tag == "-angle")  g_camFieldOfView = cin->getFloat();
			else if (tag == "-fov")    g_camFieldOfView = cin->getFloat();
			else if (tag == "-radius") g_camRadius = cin->getFloat();
			else if (tag == "-stereo") g_stereo = true;
			else if (tag == "-toeIn")  g_toeIn = true;
			else if (tag == "-eyeSeparation") g_eyeSeparation = cin->getFloat();
			else if (tag == "-zeroParallax")    g_zeroParallaxDistance = cin->getFloat();

			/* frame buffer size */
			else if (tag == "-size") {
				g_width = cin->getInt();
				g_height = cin->getInt();
				g_frameBuffer = g_device->rtNewFrameBuffer(g_format.c_str(), g_width, g_height, g_numBuffers);
			}

			/* JPEG compression quality (1-100 range) */
			else if (tag == "-jpegQuality") {
				g_jpegQuality = clamp(cin->getInt(), 1, 100);
			}

			/* set framebuffer format */
			else if (tag == "-framebuffer" || tag == "-fb") {
				g_format = cin->getString();
				g_frameBuffer = g_device->rtNewFrameBuffer(g_format.c_str(), g_width, g_height, g_numBuffers);
			}

			/* full screen mode */
			else if (tag == "-fullscreen") g_fullscreen = true;

			/* refine rendering when not moving */
			else if (tag == "-refine") g_refine = cin->getInt();

			/* scene type to use */
			else if (tag == "-scene") g_scene = cin->getString();

			/* acceleration structure to use */
			else if (tag == "-accel") {
				g_accel = g_mesh_accel = cin->getString();
			}

			/* builder to use */
			else if (tag == "-builder") {
				g_builder = g_mesh_builder = cin->getString();
			}

			/* traverser to use */
			else if (tag == "-traverser") {
				g_traverser = g_mesh_traverser = cin->getString();
			}

			/* set renderer */
			else if (tag == "-renderer") {
				const auto renderer = cin->getString();
				if (renderer == "debug") parseDebugRenderer(cin, path);
				else if (renderer == "pt" || renderer == "pathtracer") parsePathTracer(cin, path, &Yulio::yulioStop, &Yulio::rsc);
				else if (renderer == "gpt") parseGPT(cin, path, &Yulio::yulioStop, &Yulio::rsc);
				else throw std::runtime_error("(when parsing -renderer) : unknown renderer: " + renderer);
			}

			/* set gamma */
			else if (tag == "-gamma") {
				g_device->rtSetFloat1(g_tonemapper, "gamma", g_gamma = cin->getFloat());
				g_device->rtCommit(g_tonemapper);
			}

			/* set gamma */
			else if (tag == "-vignetting") {
				g_device->rtSetBool1(g_tonemapper, "vignetting", g_vignetting = cin->getInt());
				g_device->rtCommit(g_tonemapper);
			}

			/* set recursion depth */
			else if (tag == "-depth") {
				g_device->rtSetInt1(g_renderer, "maxDepth", g_depth = cin->getInt());
				g_device->rtCommit(g_renderer);
			}

			/* set recursion depth */
			else if (tag == "-tMaxShadowRay") {
				g_device->rtSetFloat1(g_renderer, "tMaxShadowRay", g_tMaxShadowRay = cin->getInt() * g_sceneScale);
				g_device->rtCommit(g_renderer);
			}

			/* set samples per pixel */
			else if (tag == "-spp") {
				g_device->rtSetInt1(g_renderer, "sampler.spp", g_spp = cin->getInt());
				g_device->rtCommit(g_renderer);
			}

			/* set the backplate */
			else if (tag == "-backplate") {
				g_device->rtSetImage(g_renderer, "backplate", g_backplate = rtLoadImage(path + cin->getFileName()));
				g_device->rtCommit(g_renderer);
			}

			/* number of frames to render in output mode (used for benchmarking) */
			else if (tag == "-frames")
				g_num_frames = cin->getInt();

			/* render frame */
			else if (tag == "-o") {
				std::string fn = cin->getFileName();
				if (fn[0] == '/')
					g_outFileName = fn; //outputMode(path + cin->getFileName());
				else
					g_outFileName = path + fn; //outputMode(path + cin->getFileName());
			}

			/* display image */
			else if (tag == "-display")
				g_outFileName = ""; //displayMode();

			  /* regression testing */
			else if (tag == "-regression")
			{
				g_refine = false;
				g_regression = true;
				Handle<Device::RTScene> scene;
				GLUTDisplay(AffineSpace3f::lookAtPoint(g_camPos, g_camLookAt, g_camUp), 0.01f, scene);
			}

			else if (tag == "-version") {
				std::cout << "Embree renderer version 2.0" << std::endl;
				exit(1);
			}

			else if (tag == "-h" || tag == "-?" || tag == "-help" || tag == "--help")
			{
				std::cout << std::endl;
				std::cout << "Embree Version 2.0" << std::endl;
				std::cout << std::endl;
				std::cout << "  usage: embree -i model.obj -renderer debug -display" << std::endl;
				std::cout << "         embree -i model.obj -renderer pathtracer -o out.tga" << std::endl;
				std::cout << "         embree -c model.ecs -display" << std::endl;
				std::cout << std::endl;
				std::cout << "-renderer [debug,profile,pathtracer]" << std::endl;
				std::cout << "  Sets the renderer to use." << std::endl;
				std::cout << std::endl;
				std::cout << "-c file" << std::endl;
				std::cout << "  Parses command line parameters from file." << std::endl;
				std::cout << std::endl;
				std::cout << "-i file" << std::endl;
				std::cout << "  Loads a scene from file." << std::endl;
				std::cout << std::endl;
				std::cout << "-o file" << std::endl;
				std::cout << "  Renders and outputs the image to the file." << std::endl;
				std::cout << std::endl;
				std::cout << "-display" << std::endl;
				std::cout << "  Interactively displays the rendering into a window." << std::endl;
				std::cout << std::endl;
				std::cout << "-vp x y z" << std::endl;
				std::cout << "  Sets camera position to the location (x,y,z)." << std::endl;
				std::cout << std::endl;
				std::cout << "-vi x y z" << std::endl;
				std::cout << "  Sets camera lookat point to the location (x,y,z)." << std::endl;
				std::cout << std::endl;
				std::cout << "-vd x y z" << std::endl;
				std::cout << "  Sets camera viewing direction to (x,y,z)." << std::endl;
				std::cout << std::endl;
				std::cout << "-vu x y z" << std::endl;
				std::cout << "  Sets camera up direction to (x,y,z)." << std::endl;
				std::cout << std::endl;
				std::cout << "-fov angle" << std::endl;
				std::cout << "  Sets camera field of view in y direction to angle." << std::endl;
				std::cout << std::endl;
				std::cout << "-size width height" << std::endl;
				std::cout << "  Sets the width and height of image to render." << std::endl;
				std::cout << std::endl;
				std::cout << "-fullscreen" << std::endl;
				std::cout << "  Enables full screen display mode." << std::endl;
				std::cout << std::endl;
				std::cout << "-accel [bvh2,bvh4,bvh4.spatial].[triangle1,triangle1i,triangle4,...]" << std::endl;
				std::cout << "  Sets the spatial index structure to use." << std::endl;
				std::cout << std::endl;
				std::cout << "-gamma v" << std::endl;
				std::cout << "  Sets gamma correction to v (only pathtracer)." << std::endl;
				std::cout << std::endl;
				std::cout << "-depth i" << std::endl;
				std::cout << "  Sets the recursion depth to i (default 16)" << std::endl;
				std::cout << std::endl;
				std::cout << "-spp i" << std::endl;
				std::cout << "  Sets the number of samples per pixel to i (default 1) (only pathtracer)." << std::endl;
				std::cout << std::endl;
				std::cout << "-backplate" << std::endl;
				std::cout << "  Sets a high resolution back ground image. (default none) (only pathtracer)." << std::endl;
				std::cout << std::endl;
				std::cout << "-ambientlight r g b" << std::endl;
				std::cout << "  Creates an ambient light with intensity (r,g,b)." << std::endl;
				std::cout << std::endl;
				std::cout << "-pointlight px py pz r g b" << std::endl;
				std::cout << "  Creates a point light with intensity (r,g,b) at position (px,py,pz)." << std::endl;
				std::cout << std::endl;
				std::cout << "-distantlight dx dy dz r g b halfAngle" << std::endl;
				std::cout << "  Creates a distant sun light with intensity (r,g,b) shining into " << std::endl;
				std::cout << "  direction (dx,dy,dz) from the cone spanned by halfAngle." << std::endl;
				std::cout << std::endl;
				std::cout << "-trianglelight px py pz ux uy uz vx vy vz r g b" << std::endl;
				std::cout << "  Creates a triangle-light with intensity (r,g,b) spanned by the point " << std::endl;
				std::cout << "  (px,py,pz) and the vectors (vx,vy,vz) and (ux,uy,uz)." << std::endl;
				std::cout << std::endl;
				std::cout << "-quadlight px py pz ux uy uz vx vy vz r g b" << std::endl;
				std::cout << "  Creates a quad-light with intensity (r,g,b) spanned by the point " << std::endl;
				std::cout << "  (px,py,pz) and the vectors (vx,vy,vz) and (ux,uy,uz)." << std::endl;
				std::cout << std::endl;
				std::cout << "-hdrilight r g b file" << std::endl;
				std::cout << "  Creates a high dynamic range environment light from the image " << std::endl;
				std::cout << "  file. The intensities are multiplies by (r,g,b)." << std::endl;
				std::cout << std::endl;
				std::cout << "-trisphere px py pz r theta phi" << std::endl;
				std::cout << "  Creates a triangulated sphere with radius r at location (px,py,pz) " << std::endl;
				std::cout << "  and triangulation rates theta and phi." << std::endl;
				std::cout << std::endl;
				std::cout << "-[no]refine" << std::endl;
				std::cout << "  Enables (default) or disables the refinement display mode." << std::endl;
				std::cout << std::endl;
				std::cout << "-regression" << std::endl;
				std::cout << "  Runs a stress test of the system." << std::endl;
				std::cout << std::endl;
				std::cout << "-version" << std::endl;
				std::cout << "  Prints version number." << std::endl;
				std::cout << std::endl;
				std::cout << "-h, -?, -help, --help" << std::endl;
				std::cout << "  Prints this help." << std::endl;
				exit(1);
			}

			/* skip unknown command line parameter */
			else {
				std::cerr << "unknown command line parameter: " << tag << " ";
				while (cin->peek() != "" && cin->peek()[0] != '-') std::cerr << cin->getString() << " ";
				std::cerr << std::endl;
			}
		}
	}

	/* main function in embree namespace */
	int main(int argc, char** argv)
	{
		Ref<ParseStream> stream;

		// Lev: special handling for the purposes of FPR rendering and Collada processing
		{
			std::vector<std::string> args;
			for (ssize_t k = 0; k < argc; k++)
				args.push_back(argv[k]);

			g_workingDirectory = FileName(args[0]).path();

			if (args.size() == 2) {
				FileName colladaFile = args[1];

				const auto extension = colladaFile.ext();
				if (extension == "dae") {
					g_sceneFileName = colladaFile;
					g_processingFprCollada = true;
				}
			}
		}

		/*! create stream for parsing */
		if (g_processingFprCollada) {
			const std::vector<std::string> argv = { "dummy_param", "-c", "renderer_settings" };
			stream = new ParseStream(new CommandLineStream(argv));
		}
		else {
			stream = new ParseStream(new CommandLineStream(argc, argv));
		}

		/*! parse device to use */
		parseNumThreads(stream);
		parseDevice(stream);

		/*! create embree device */
		if (!g_device) {
			g_device = Device::rtCreateDevice("default", g_numThreads, g_rtcore_cfg.c_str());
		}

		createGlobalObjects();

		if (g_processingFprCollada) {
			std::vector<Handle<Device::RTPrimitive>> prims = rtLoadScene(g_sceneFileName, &g_stereoCubeCameras);
			g_prims.insert(g_prims.end(), prims.begin(), prims.end());
		}

		if (g_stereoCubeCameras.size()) {
			g_device->rtGetFloat1(g_stereoCubeCameras[0], "sceneScale", g_sceneScale);
		}

		/*! parse command line */
		parseCommandLine(stream, g_workingDirectory);

		//upload_time = getSeconds();
		//PRINT(upload_time);

		/*! if we did no render yet but have loaded a scene, switch to display mode */
		if (!g_rendered && g_prims.size()) {
			if (g_outFileName != ""
				|| g_processingFprCollada)
				outputMode(g_outFileName);
			else
				displayMode();
		}

		/*! cleanup */
		clearGlobalObjects();

		return(0);
	}

} // namespace embree

namespace Yulio {

	//using namespace embree;
	//using namespace std;

	static thread workerThread;

	const char *errorStrings[] = {
		"OK"
	};


	void workerThreadRT(const string &file, const ParamsRT *params) {
		
		yulioStatusTracker.SetCurrentState(Initialiazing);

		const FileName colladaFile = file;

		// Lev: special handling for the purposes of FPR rendering and Collada processing
		const auto extension = colladaFile.ext();
		if (extension != "dae") {
			yulioStatusTracker.AddError(MissingColladaFile);
			return;
		}

		g_sceneFileName = colladaFile;
		g_workingDirectory = FileName(colladaFile).path();
		g_processingFprCollada = true;

		ParamsRT currentParams; // Initialized with the defaults
		if (params) currentParams = *params;

		/*! create stream for parsing */
		std::vector<std::string> argv = { "dummy_param" };
		// stereo (HAS to be present)
		{ argv.push_back("-stereo"); }
		// renderer (i.e the integrator type)
		{ argv.push_back("-renderer"); argv.push_back(currentParams.renderer ? currentParams.renderer : "pathtracer"); }
		// spp
		{ argv.push_back("-spp"); argv.push_back(to_string(currentParams.spp)); }
		// size
		{ argv.push_back("-size"); argv.push_back(to_string(currentParams.size)); argv.push_back(to_string(currentParams.size)); }
		// depth
		{ argv.push_back("-depth"); argv.push_back(to_string(currentParams.depth)); }
		// jpegQuality
		{ argv.push_back("-jpegQuality"); argv.push_back(to_string(currentParams.jpegQuality)); }
		// tMaxShadowRay
		{ argv.push_back("-tMaxShadowRay"); argv.push_back(to_string(currentParams.tMaxShadowRay)); }
		// ambientlight
		{ argv.push_back("-ambientlight"); argv.push_back(to_string(currentParams.ambientlight[0]));  argv.push_back(to_string(currentParams.ambientlight[1]));  argv.push_back(to_string(currentParams.ambientlight[2])); }
		// eyeSeparation
		{ argv.push_back("-eyeSeparation"); argv.push_back(to_string(currentParams.eyeSeparation)); }
		// toeIn
		{ if (currentParams.toeIn) argv.push_back("-toeIn"); }
		// eyeSeparation
		{ argv.push_back("-zeroParallax"); argv.push_back(to_string(currentParams.zeroParallax)); }
		// debug
		{ if (currentParams.debug) argv.push_back("-debug"); }

		Ref<ParseStream> stream = new ParseStream(new CommandLineStream(argv));

		/*! parse device to use */
		parseNumThreads(stream);
		parseDevice(stream);

		/*! create embree device */
		if (!g_device) {
			g_device = Device::rtCreateDevice("default", g_numThreads, g_rtcore_cfg.c_str());
		}

		createGlobalObjects();

		// Note, local prims variable needs to be destroyed (i.e. the vector needs to be cleaned) before clearGlobalObjects is called (i.e. before g_device is deleted),
		// so we create a local scope for that.
		{
			vector<Handle<Device::RTPrimitive>> prims = rtLoadScene(g_sceneFileName, &g_stereoCubeCameras);
			g_prims.insert(g_prims.end(), prims.begin(), prims.end());
		}

		if (!g_stereoCubeCameras.size()) {
			yulioStatusTracker.AddError(InvalidColladaFormat);
			goto cleanup;
		}
		
		g_device->rtGetFloat1(g_stereoCubeCameras[0], "sceneScale", g_sceneScale);

		/*! parse command line */
		parseCommandLine(stream, g_workingDirectory);

		/*! if we did no render yet but have loaded a scene, switch to display mode */
		if (//g_rendered ||
			!g_prims.size()) {
			yulioStatusTracker.AddError(InvalidColladaFormat);
			goto cleanup;
		}
		
		outputMode(g_outFileName);

	cleanup:
		/*! cleanup */
		clearGlobalObjects();
	}

	DllApi bool StartRT(const char* colladaFile, const ParamsRT* params) {
		if (yulioRunning)
			return false;

		yulioStatusTracker.Reset();

		if (!colladaFile) {
			yulioStatusTracker.AddError(MissingColladaFile);
			return false;
		}

		workerThread = std::thread(workerThreadRT, string(colladaFile), params);
		if (workerThread.joinable()) {
			yulioRunning = true;
		}

		return yulioRunning;
	}

	DllApi bool WaitRT() {
		if (!yulioRunning)
			return false;

		// Wait for the thread to finish
		workerThread.join();
		yulioRunning = false;
		yulioStop = false;

		return true;
	}

	DllApi bool StopRT(bool keepResults) {
		if (!yulioRunning)
			return false;

		yulioKeepResults = keepResults;

		// Signal to all the listening parties to stop
		yulioStop = true;

		// Wait for the thread to finish
		workerThread.join();
		yulioRunning = false;
		yulioStop = false;
	
		return true;
	}

	DllApi ErrorCodeRT GetLastErrorRT() {
		return yulioStatusTracker.GetLastError();
	}

	DllApi StatusRT GetCurrentStatusRT() {
		return yulioStatusTracker.GetCurrentStatus();
	}


}
/******************************************************************************/
/*                               Main Function                                */
/******************************************************************************/

int main(int argc, char** argv)
{
	try {
		return embree::main(argc, argv);
	}
	catch (const std::exception& e) {
		embree::clearGlobalObjects();
		std::cout << "Error: " << e.what() << std::endl;
		return 1;
	}
	catch (...) {
		embree::clearGlobalObjects();
		return 1;
	}
}
