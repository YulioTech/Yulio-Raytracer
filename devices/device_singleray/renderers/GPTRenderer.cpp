#include "renderers/GPTRenderer.h"

/* include GPT integrator */
#include "integrators/GPTIntegrator.h"
#include "renderers/GPTPoissonSolver/Solver.hpp"

/// If defined, applies reconstruction after rendering.
#define RECONSTRUCT

/* include all samplers */
#include "samplers/sampler.h"

/* include all image filters */
#include "filters/boxfilter.h"
#include "filters/bsplinefilter.h"

#include <algorithm>

namespace embree
{
	GPTRenderer::GPTRenderer(const Parms& parms)
		: iteration(0)
	{
		/*! create GPT integrator - the only currently supported by the GPT renderer */
		integrator = new GPTIntegrator(parms);

		shiftThreshold = parms.getFloat("shiftThreshold", .001f);
		reconstructL1 = parms.getBool("reconstructL1", true);
		reconstructL2 = parms.getBool("reconstructL2", false);
		reconstructAlpha = parms.getFloat("reconstructAlpha", .2f);

		if (reconstructL1 && reconstructL2)
			throw std::runtime_error("Disable 'reconstructL1' or 'reconstructL2': Cannot display two reconstructions at a time!");

		if (reconstructAlpha <= 0.f)
			throw std::runtime_error("'reconstructAlpha' must be set to a value greater than zero!");

		/*! create sampler to use */
		const std::string _samplers = parms.getString("sampler", "multijittered");
		if (_samplers == "multijittered") samplers = new SamplerFactory(parms);
		else throw std::runtime_error("unknown sampler type: " + _samplers);

		/*! create pixel filter to use */
		// Lev: sample-shifting filters do not appear to work well with GPT. We should look into post-processing filters instead.
		const std::string _filter = parms.getString("filter", "none");
		if (_filter == "none") filter = NULL;
		else if (_filter == "box") filter = new BoxFilter;
		else if (_filter == "bspline") filter = new BSplineFilter;
		else throw std::runtime_error("unknown filter type: " + _filter);

		/*! get framebuffer configuration */
		gamma = parms.getFloat("gamma", 1.f);

		/*! show progress to the user */
		showProgress = parms.getInt("showprogress", 0);
	}

	void GPTRenderer::renderFrame(const Ref<Camera>& camera, const Ref<BackendScene>& scene, const Ref<ToneMapper>& toneMapper, Ref<SwapChain> swapchain, int accumulate)
	{
		if (accumulate == 0) iteration = 0;
		renderJob = new RenderJob(this, camera, scene, toneMapper, swapchain, accumulate, iteration);
		iteration++;
	}

	GPTRenderer::RenderJob::RenderJob(Ref<GPTRenderer> renderer, const Ref<Camera>& camera, const Ref<BackendScene>& scene,
		const Ref<ToneMapper>& toneMapper, Ref<SwapChain> swapchain, int accumulate, int iteration)
		: renderer(renderer), camera(camera), scene(scene), toneMapper(toneMapper), swapchain(swapchain),
		accumulate(accumulate), iteration(iteration), tileID(0), atomicNumRays(0)
	{
		const auto width = swapchain->getWidth();
		const auto height = swapchain->getHeight();
		numTilesX = ((int)width + TILE_SIZE - 1) / TILE_SIZE;
		numTilesY = ((int)height + TILE_SIZE - 1) / TILE_SIZE;
		rcpWidth = rcp(float(width));
		rcpHeight = rcp(float(height));
		this->framebuffer = swapchain->buffer();
		this->framebuffer->startRendering(numTilesX*numTilesY);
		if (renderer->showProgress) new (&progress) Progress(numTilesX*numTilesY);

		if (renderer->showProgress) progress.start();
		renderer->samplers->reset();
		renderer->integrator->requestSamples(renderer->samplers, scene);
		renderer->samplers->init(iteration, renderer->filter);

		// Init the accumulation buffers
		buffers[BUFFER_THROUGHPUT] = new AccuBuffer(width, height);
		buffers[BUFFER_DX] = new AccuBuffer(width, height);
		buffers[BUFFER_DY] = new AccuBuffer(width, height);
		buffers[BUFFER_VERY_DIRECT] = new AccuBuffer(width, height);
#if 1
		TaskScheduler::EventSync event;
		TaskScheduler::Task task(&event, _renderTile, this, TaskScheduler::getNumThreads(), _finish, this, "render::tile");
		TaskScheduler::addTask(-1, TaskScheduler::GLOBAL_BACK, &task);
		event.sync();
#else
		new (&task) TaskScheduler::Task(NULL, _renderTile, this, TaskScheduler::getNumThreads(), _finish, this, "render::tile");
		TaskScheduler::addTask(-1, TaskScheduler::GLOBAL_BACK, &task);
#endif
	}

	void GPTRenderer::RenderJob::finish(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event)
	{
#if defined(RECONSTRUCT)
		if (renderer->reconstructL1 || renderer->reconstructL2) {

			/* Reconstruct. */
			poisson::Solver::Params params;

			if (renderer->reconstructL1) {
				params.setConfigPreset("L1D");
			}
			else if (renderer->reconstructL2) {
				params.setConfigPreset("L2D");
			}

			const auto width = swapchain->getWidth();
			const auto height = swapchain->getHeight();
			const auto pixelCount = width * height;
			std::vector<Vec3f> throughputVector(pixelCount);
			std::vector<Vec3f> dxVector(pixelCount);
			std::vector<Vec3f> dyVector(pixelCount);
			std::vector<Vec3f> directVector(pixelCount);
			std::vector<Vec3f> reconstructionVector(pixelCount);

			//std::transform(reinterpret_cast<Vec4f *>(buffers[BUFFER_THROUGHPUT]->getData()), reinterpret_cast<Vec4f *>(buffers[BUFFER_THROUGHPUT]->getData()) + pixelCount,
			//	throughputVector.begin(), [](const Vec4f &src)  { return Vec3f(src.x, src.y, src.z); });
			//std::transform(reinterpret_cast<Vec4f *>(buffers[BUFFER_DX]->getData()), reinterpret_cast<Vec4f *>(buffers[BUFFER_DX]->getData()) + pixelCount,
			//	dxVector.begin(), [](Vec4f src) -> Vec3f { return Vec3f(src.x, src.y, src.z); });
			//std::transform(reinterpret_cast<Vec4f *>(buffers[BUFFER_DY]->getData()), reinterpret_cast<Vec4f *>(buffers[BUFFER_DY]->getData()) + pixelCount,
			//	dyVector.begin(), [](Vec4f src) -> Vec3f { return Vec3f(src.x, src.y, src.z); });
			//std::transform(reinterpret_cast<Vec4f *>(buffers[BUFFER_VERY_DIRECT]->getData()), reinterpret_cast<Vec4f *>(buffers[BUFFER_VERY_DIRECT]->getData()) + pixelCount,
			//	directVector.begin(), [](Vec4f src) -> Vec3f { return Vec3f(src.x, src.y, src.z); });

			Color c = zero;
			for (int y = 0, p = 0; y < height; ++y) {
				for (int x = 0; x < width; ++x, ++p) {
					c = buffers[BUFFER_THROUGHPUT]->get(x, y);
					throughputVector[p] = Vec3f(c.r, c.g, c.b);
					
					c = buffers[BUFFER_DX]->get(x, y);
					dxVector[p] = Vec3f(c.r, c.g, c.b);
					
					c = buffers[BUFFER_DY]->get(x, y);
					dyVector[p] = Vec3f(c.r, c.g, c.b);
					
					c = buffers[BUFFER_VERY_DIRECT]->get(x, y);
					directVector[p] = Vec3f(c.r, c.g, c.b);
				}
			}

			params.alpha = renderer->reconstructAlpha;
			///params.setLogFunction(poisson::Solver::Params::LogFunction([](const std::string& message) { SLog(EInfo, "%s", message.c_str()); }));

			poisson::Solver solver(params);
			solver.importImagesMTS(
				reinterpret_cast<float *>(dxVector.data()),
				reinterpret_cast<float *>(dyVector.data()),
				reinterpret_cast<float *>(throughputVector.data()),
				reinterpret_cast<float *>(directVector.data()),
				width, height);

			solver.setupBackend();
			solver.solveIndirect();

			solver.exportImagesMTS(reinterpret_cast<float *>(reconstructionVector.data()));

			// Update the main framebuffer with the results
			for (int y = 0, p = 0; y < height; ++y) {
				const size_t _y = swapchain->raster2buffer(y);
				for (int x = 0; x < width; ++x, ++p) {
					Color c(reconstructionVector[p].x, reconstructionVector[p].y, reconstructionVector[p].z);

					//Color c(throughputVector[p].x, throughputVector[p].y, throughputVector[p].z);
					//Color c(dxVector[p].x, dxVector[p].y, dxVector[p].z);
					//Color c(dyVector[p].x, dyVector[p].y, dyVector[p].z);
					//Color c(directVector[p].x, directVector[p].y, directVector[p].z);
					//Color c = buffers[BUFFER_THROUGHPUT]->get(x, y);
					//Color c = buffers[BUFFER_DX]->get(x, y);
					//framebuffer->set(x, y, c);

					const Color L0 = swapchain->update(x, _y, c, 1.f, false);
					const Color L1 = toneMapper->eval(L0, x, y, swapchain);
					framebuffer->set(x, _y, L1);
				}
			}

		}

#endif //RECONSTRUCT

		if (renderer->showProgress) progress.end();
		double dt = getSeconds() - t0;

		/*! print fps, render time, and rays per second */
		std::ostringstream stream;
		stream << "render  ";
		stream.setf(std::ios::fixed, std::ios::floatfield);
		stream.precision(2);
		stream << 1.0f / dt << " fps, ";
		stream.precision(0);
		stream << dt*1000.0f << " ms, ";
		stream.precision(3);
		stream << atomicNumRays / dt*1E-6 << " mrps";
		std::cout << stream.str() << std::endl;

		rtcDebug();

		delete this;
	}

	void GPTRenderer::RenderJob::renderTile(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event)
	{
		/*! create a new sampler */
		IntegratorState state;
		if (taskIndex == taskCount - 1) t0 = getSeconds();

		/*! tile pick loop */
		while (true)
		{
			/*! pick a new tile */
			size_t tile = tileID++;
			if (tile >= numTilesX*numTilesY) break;

			/*! process all tile samples */
			const int tile_x = (tile%numTilesX)*TILE_SIZE;
			const int tile_y = (tile / numTilesX)*TILE_SIZE;
			Random randomNumberGenerator(tile_x * 91711 + tile_y * 81551 + 3433 * swapchain->firstActiveLine());

			for (size_t dy = 0; dy < TILE_SIZE; dy++)
			{
				size_t y = tile_y + dy;
				if (y >= swapchain->getHeight()) continue;
				if (!swapchain->activeLine(y)) continue;

				const size_t _y = swapchain->raster2buffer(y);

				for (size_t dx = 0; dx < TILE_SIZE; dx++)
				{
					const size_t x = tile_x + dx;
					if (x >= swapchain->getWidth()) continue;

					const int set = randomNumberGenerator.getInt(renderer->samplers->sampleSets);

					Color centralVeryDirect = zero, centralThroughput = zero;
					Color gradients[4] = { zero, zero, zero, zero }, shiftedThroughputs[4] = { zero, zero, zero, zero };

					// Pixel shifts for offset rays
					static const Vec2f pixelShift[4]{
						Vec2f(1.f, 0.f),
						Vec2f(0.f, 1.f),
						Vec2f(-1.f, 0.f),
						Vec2f(0.f, -1.f)
					};

					const size_t spp = renderer->samplers->samplesPerPixel;
					for (size_t s = 0; s < spp; s++)
					{
						PrecomputedSample &sample = renderer->samplers->samples[set][s];
						const float fx = (float(x) + sample.pixel.x) * rcpWidth;
						const float fy = (float(y) + sample.pixel.y) * rcpHeight;

						// Base ray
						Ray baseRay; camera->ray(Vec2f(fx, fy), sample.getLens(), baseRay);
						baseRay.time = sample.getTime();

						Ray offsetRays[4];
						for (int i = 0; i < 4; ++i) {
							const float fx = (float(x) + pixelShift[i].x + sample.pixel.x) * rcpWidth;
							const float fy = (float(y) + pixelShift[i].y + sample.pixel.y) * rcpHeight;
							camera->ray(Vec2f(fx, fy), sample.getLens(), offsetRays[i]);
							offsetRays[i].time = sample.getTime();
						}

						state.sample = &sample;
						state.pixel = Vec2f(fx, fy);

						renderer->integrator->EvaluatePoint(baseRay, offsetRays,
							centralVeryDirect, centralThroughput,
							gradients, shiftedThroughputs,
							scene, state);
					}

					//const Color L0 = swapchain->update(x, _y, (8 * centralVeryDirect) + (2 * centralThroughput), spp * 4, accumulate);
					//const Color L1 = toneMapper->eval(L0, x, y, swapchain);
					//framebuffer->set(x, _y, L0);

					// Accumulate the results
					{
						static const bool accumulate = true;

						static const int RIGHT = 0;
						static const int BOTTOM = 1;
						static const int LEFT = 2;
						static const int TOP = 3;
						
						const Vec2f center_pixel(x, y);
						const Vec2f right_pixel = center_pixel + pixelShift[RIGHT];
						const Vec2f bottom_pixel = center_pixel + pixelShift[BOTTOM];
						const Vec2f left_pixel = center_pixel + pixelShift[LEFT];
						const Vec2f top_pixel = center_pixel + pixelShift[TOP];

						// Actual throughputs, with MIS between central and neighbor pixels for all neighbors.
						// This can be replaced with a standard throughput sample without much loss of quality in most cases.
						{
							//block->put(center_pixel, (2 * centralThroughput), 4.0f, 4.0f, BUFFER_THROUGHPUT); // Central throughput.

							//block->put(left_pixel, (2 * shiftedThroughputs[LEFT]), 1.0f, 1.0f, BUFFER_THROUGHPUT);     // Negative x throughput.
							//block->put(right_pixel, (2 * shiftedThroughputs[RIGHT]), 1.0f, 1.0f, BUFFER_THROUGHPUT);   // Positive x throughput.
							//block->put(top_pixel, (2 * shiftedThroughputs[TOP]), 1.0f, 1.0f, BUFFER_THROUGHPUT);       // Negative y throughput.
							//block->put(bottom_pixel, (2 * shiftedThroughputs[BOTTOM]), 1.0f, 1.0f, BUFFER_THROUGHPUT); // Positive y throughput.

							buffers[BUFFER_THROUGHPUT]->update(center_pixel.x, center_pixel.y, 2 * centralThroughput, 4.f * spp, accumulate);			// Central throughput.

							buffers[BUFFER_THROUGHPUT]->update(left_pixel.x, left_pixel.y, 2 * shiftedThroughputs[LEFT], 1.f * spp, accumulate);		// Negative x throughput.
							buffers[BUFFER_THROUGHPUT]->update(right_pixel.x, right_pixel.y, 2 * shiftedThroughputs[RIGHT], 1.f * spp, accumulate);		// Positive x throughput.
							buffers[BUFFER_THROUGHPUT]->update(top_pixel.x, top_pixel.y, 2 * shiftedThroughputs[TOP], 1.f * spp, accumulate);			// Negative y throughput.
							buffers[BUFFER_THROUGHPUT]->update(bottom_pixel.x, bottom_pixel.y, 2 * shiftedThroughputs[BOTTOM], 1.f * spp, accumulate);	// Positive y throughput.
						}

						// Gradients.
						{
							//block->put(left_pixel, -(2 * gradients[LEFT]), 1.0f, 1.0f, BUFFER_DX);    // Negative x gradient.
							//block->put(center_pixel, (2 * gradients[RIGHT]), 1.0f, 1.0f, BUFFER_DX);  // Positive x gradient.
							//block->put(top_pixel, -(2 * gradients[TOP]), 1.0f, 1.0f, BUFFER_DY);      // Negative y gradient.
							//block->put(center_pixel, (2 * gradients[BOTTOM]), 1.0f, 1.0f, BUFFER_DY); // Positive y gradient.

#if 1
							buffers[BUFFER_DX]->update(left_pixel.x, left_pixel.y, -2 * gradients[LEFT], 1.f * spp, accumulate);		// Negative x gradient.
							buffers[BUFFER_DX]->update(center_pixel.x, center_pixel.y, 2 * gradients[RIGHT], 1.f * spp, accumulate);		// Positive x gradient.
							buffers[BUFFER_DY]->update(top_pixel.x, top_pixel.y, -2 * gradients[TOP], 1.f * spp, accumulate);			// Negative y gradient.
							buffers[BUFFER_DY]->update(center_pixel.x, center_pixel.y, 2 * gradients[BOTTOM], 1.f * spp, accumulate);	// Positive y gradient.
#else
							// Re-balance gradients around the center
							buffers[BUFFER_DX]->update(left_pixel.x, left_pixel.y, -2 * gradients[LEFT], 1.f * spp, accumulate);		// Negative x gradient.
							buffers[BUFFER_DX]->update(center_pixel.x, center_pixel.y, -2 * gradients[LEFT], 1.f * spp, accumulate);		// Negative x gradient.
							buffers[BUFFER_DX]->update(center_pixel.x, center_pixel.y, 2 * gradients[RIGHT], 1.f * spp, accumulate);		// Positive x gradient.
							buffers[BUFFER_DX]->update(right_pixel.x, right_pixel.y, 2 * gradients[RIGHT], 1.f * spp, accumulate);		// Positive x gradient.

							buffers[BUFFER_DY]->update(top_pixel.x, top_pixel.y, -2 * gradients[TOP], 1.f * spp, accumulate);			// Negative y gradient.
							buffers[BUFFER_DY]->update(center_pixel.x, center_pixel.y, -2 * gradients[TOP], 1.f * spp, accumulate);		// Negative y gradient.
							buffers[BUFFER_DY]->update(center_pixel.x, center_pixel.y, 2 * gradients[BOTTOM], 1.f * spp, accumulate);	// Positive y gradient.
							buffers[BUFFER_DY]->update(bottom_pixel.x, bottom_pixel.y, 2 * gradients[BOTTOM], 1.f * spp, accumulate);	// Positive y gradient.
#endif
						}

						// Very direct.
						{
							//block->put(center_pixel, centralVeryDirect, 1.0f, 1.0f, BUFFER_VERY_DIRECT);

							buffers[BUFFER_VERY_DIRECT]->update(center_pixel.x, center_pixel.y, centralVeryDirect, 1.f * spp, accumulate);
						}
					}
				}
			}

			/*! print progress bar */
			if (renderer->showProgress) progress.next();

			/*! mark one more tile as finished */
			framebuffer->finishTile();
		}

		/*! we access the atomic ray counter only once per tile */
		atomicNumRays += state.numRays;
	}
}
