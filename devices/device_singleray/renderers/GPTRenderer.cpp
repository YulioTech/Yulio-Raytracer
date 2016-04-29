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
		const std::string _filter = parms.getString("filter", "bspline");
		if (_filter == "none") filter = NULL;
		else if (_filter == "box") filter = new BoxFilter;
		else if (_filter == "bspline") filter = new BSplineFilter;
		else throw std::runtime_error("unknown filter type: " + _filter);

		/*! get framebuffer configuration */
		gamma = parms.getFloat("gamma", 1.0f);

		/*! show progress to the user */
		showProgress = parms.getInt("showprogress", 0);
	}

	void GPTRenderer::stopRendering() {
		if (renderJob) {
			int n = 0;
		}
	}

	void GPTRenderer::renderFrame(const Ref<Camera>& camera, const Ref<BackendScene>& scene, const Ref<ToneMapper>& toneMapper, Ref<SwapChain> swapchain, int accumulate)
	{
		if (accumulate == 0) iteration = 0;
		renderJob = new RenderJob(this, camera, scene, toneMapper, swapchain, accumulate, iteration);
		iteration++;

#if defined(RECONSTRUCT)
		if (reconstructL1 || reconstructL2) {

			/* Reconstruct. */
			poisson::Solver::Params params;

			if (reconstructL1) {
				params.setConfigPreset("L1D");
			}
			else if (reconstructL2) {
				params.setConfigPreset("L2D");
			}

			params.alpha = reconstructAlpha;
			///params.setLogFunction(poisson::Solver::Params::LogFunction([](const std::string& message) { SLog(EInfo, "%s", message.c_str()); }));

			poisson::Solver solver(params);
			///solver.importImagesMTS(dxVector.data(), dyVector.data(), throughputVector.data(), directVector.data(), film->getCropSize().x, film->getCropSize().y);

			solver.setupBackend();
			solver.solveIndirect();

			//solver.exportImagesMTS(reconstructionVector.data());

			///* Give the solution back to Mitsuba. */
			//int w = reconstructionBitmap->getSize().x;
			//int h = reconstructionBitmap->getSize().y;

			//for (int y = 0, p = 0; y < h; ++y) {
			//	for (int x = 0; x < w; ++x, p += 3) {
			//		Float color[3] = { (Float)reconstructionVector[p], (Float)reconstructionVector[p + 1], (Float)reconstructionVector[p + 2] };
			//		reconstructionBitmap->setPixel(Point2i(x, y), Spectrum(color));
			//	}
			//}

			//film->setBitmapMulti(reconstructionBitmap, 1, BUFFER_FINAL);

		}

#endif //RECONSTRUCT
	}

	GPTRenderer::RenderJob::RenderJob(Ref<GPTRenderer> renderer, const Ref<Camera>& camera, const Ref<BackendScene>& scene,
		const Ref<ToneMapper>& toneMapper, Ref<SwapChain > swapchain, int accumulate, int iteration)
		: renderer(renderer), camera(camera), scene(scene), toneMapper(toneMapper), swapchain(swapchain),
		accumulate(accumulate), iteration(iteration), tileID(0), atomicNumRays(0)
	{
		numTilesX = ((int)swapchain->getWidth() + TILE_SIZE - 1) / TILE_SIZE;
		numTilesY = ((int)swapchain->getHeight() + TILE_SIZE - 1) / TILE_SIZE;
		rcpWidth = rcp(float(swapchain->getWidth()));
		rcpHeight = rcp(float(swapchain->getHeight()));
		this->framebuffer = swapchain->buffer();
		this->framebuffer->startRendering(numTilesX*numTilesY);
		if (renderer->showProgress) new (&progress) Progress(numTilesX*numTilesY);

		if (renderer->showProgress) progress.start();
		renderer->samplers->reset();
		renderer->integrator->requestSamples(renderer->samplers, scene);
		renderer->samplers->init(iteration, renderer->filter);

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

					Color centerVeryDirect = zero, centerThroughput = zero;
					Color gradients[4], shiftedThroughputs[4];

					const size_t spp = renderer->samplers->samplesPerPixel;
					for (size_t s = 0; s < spp; s++)
					{
						PrecomputedSample &sample = renderer->samplers->samples[set][s];
						const float fx = (float(x) + sample.pixel.x) * rcpWidth;
						const float fy = (float(y) + sample.pixel.y) * rcpHeight;

						// Base ray
						Ray baseRay; camera->ray(Vec2f(fx, fy), sample.getLens(), baseRay);
						baseRay.time = sample.getTime();

						// Pixel shifts for offset rays
						static const Vec2f pixelShift[4]{
							Vec2f(1.f, 0.f),
							Vec2f(0.f, 1.f),
							Vec2f(-1.f, 0.f),
							Vec2f(0.f, -1.f)
						};

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
							centerVeryDirect, centerThroughput,
							gradients, shiftedThroughputs,
							scene, state);
					}

					const Color L0 = swapchain->update(x, _y, centerVeryDirect, spp, accumulate);
					const Color L1 = toneMapper->eval(L0, x, y, swapchain);
					framebuffer->set(x, _y, L1);
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
