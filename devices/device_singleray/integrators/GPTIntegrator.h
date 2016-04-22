#pragma once

#include "integrators/integrator.h"
#include "renderers/renderer.h"
#include "image/image.h"

namespace embree
{
	/*! Path tracer integrator. The implementation follows a single path
	*  from the camera into the scene and connect the path at each
	*  diffuse or glossy surface to all light sources. Except for this
	*  the path is never split, also not at glass surfaces. */

	class GPTIntegrator : public Integrator
	{
		/*! Tracks the state of the path. */
		class __align(16) LightPath
		{
		public:

			/*! Constructs a path. */
			__forceinline LightPath(const Ray& ray, const Medium& medium = Medium::Vacuum(), const int depth = 0,
				const Color& throughput = one, const bool ignoreVisibleLights = false, const bool unbent = true)
				: lastRay(ray), lastMedium(medium), depth(depth), throughput(throughput), ignoreVisibleLights(ignoreVisibleLights), unbent(unbent) {}

			/*! Extends a light path. */
			__forceinline LightPath extended(const Ray& nextRay, const Medium& nextMedium, const Color& weight, const bool ignoreVL) const {
				return LightPath(nextRay, nextMedium, depth + 1, throughput*weight, ignoreVL, true/*unbent && (nextRay.dir == lastRay.dir)*/);
			}

		public:
			Ray lastRay;                 /*! Last ray in the path. */
			Medium lastMedium;           /*! Medium the last ray travels inside. */
			uint32 depth;                /*! Recursion depth of path. */
			Color throughput;            /*! Determines the fraction of radiance that reaches the pixel along the path. */
			bool ignoreVisibleLights;    /*! If the previous shade point used shadow rays we have to ignore the emission
										 of geometrical lights to not double count them. */
			bool unbent;                 /*! True if the ray path is a straight line. */
		};

	public:

		/*! Construction of integrator from parameters. */
		GPTIntegrator(const Parms& parms);
		virtual ~GPTIntegrator() {}

		/*! Registers samples we need tom the sampler. */
		void requestSamples(Ref<SamplerFactory>& samplerFactory, const Ref<BackendScene>& scene);

		/*! Function that is recursively called to compute the path. */
		Color Li(LightPath& lightPath, const Ref<BackendScene>& scene, IntegratorState& state);

		/*! Computes the radiance arriving at the origin of the ray from the ray direction. */
		Color Li(Ray& ray, const Ref<BackendScene>& scene, IntegratorState& state);

		/* Configuration. */
	private:
		size_t maxDepth;               //!< Maximal recursion depth (1=primary ray only)
		float minContribution;         //!< Minimal contribution of a path to the pixel.
		float epsilon;                 //!< Epsilon to avoid self intersections.
		float tMaxShadowRay;			//!< Max length of a shadow ray (i.e. an actual ray used to test for occlusion, not just any secondary, aka "shadow", ray)
		Ref<Image> backplate;          //!< High resolution background.

		/*! Random variables. */
	private:
		int lightSampleID;            //!< 2D random variable to sample the light source.
		int firstScatterSampleID;     //!< 2D random variable to sample the BRDF.
		int firstScatterTypeSampleID; //!< 1D random variable to sample the BRDF type to choose.
		std::vector<int> precomputedLightSampleID;  //!< ID of precomputed light samples for lights that need pre-computations.
	};
}

