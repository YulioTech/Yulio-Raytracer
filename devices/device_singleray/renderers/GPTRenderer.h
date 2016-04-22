#pragma once

#include "../renderers/renderer.h"
#include "../integrators/integrator.h"
#include "../samplers/sampler.h"
#include "../filters/filter.h"
#include "../renderers/progress.h"
#include "common/sys/taskscheduler.h"

namespace embree
{
	/*! Renderer that uses a given integrator, sampler, and pixel
	*  filter. */
	class GPTRenderer : public Renderer
	{
	public:

		/*! Construction from parameters. */
		GPTRenderer(const Parms& parms);

		/*! Renders a single frame. */
		void renderFrame(const Ref<Camera>& camera, const Ref<BackendScene>& scene, const Ref<ToneMapper>& toneMapper, Ref<SwapChain> film, int accumulate);

	private:

		class RenderJob
		{
		public:
			RenderJob(Ref<GPTRenderer> renderer, const Ref<Camera>& camera, const Ref<BackendScene>& scene,
				const Ref<ToneMapper>& toneMapper, Ref<SwapChain> swapchain, int accumulate, int iteration);

		private:

			/*! start functon */
			TASK_RUN_FUNCTION(RenderJob, renderTile);

			/*! finish function */
			TASK_COMPLETE_FUNCTION(RenderJob, finish);

			/*! Arguments of renderFrame function */
		private:
			Ref<GPTRenderer> renderer;
			Ref<Camera> camera;            //!< Camera to render from
			Ref<BackendScene> scene;       //!< Scene to render
			Ref<ToneMapper> toneMapper;    //!< Tonemapper to use.
			Ref<FrameBuffer> framebuffer;  //!< Framebuffer to render into
			Ref<SwapChain> swapchain;		 //!< Swapchain to render into
			int accumulate;                //!< Accumulation mode
			int iteration;

			/*! Precomputations. */
		private:
			float rcpWidth;                //!< Reciprocal width of framebuffer.
			float rcpHeight;               //!< Reciprocal height of framebuffer.
			size_t numTilesX;              //!< Number of tiles in x direction.
			size_t numTilesY;              //!< Number of tiles in y direction.

		private:
			double t0;                     //!< start time of rendering
			Atomic tileID;                 //!< ID of current tile
			Atomic atomicNumRays;          //!< for counting number of shoot rays
			Progress progress;             //!< Progress printer
			TaskScheduler::Task task;
		};

		/*! Configuration */
	private:
		int maxDepth;                  //!< Maximal recursion depth.
		float gamma;                   //!< Gamma to use for framebuffer writeback.

	private:
		Ref<Integrator> integrator;    //!< Integrator to use.
		Ref<SamplerFactory> samplers;  //!< Sampler to use.
		Ref<Filter> filter;            //!< Pixel filter to use.

	private:
		int iteration;
		bool showProgress;             //!< Set to true if user wants rendering progress shown
	};
}
