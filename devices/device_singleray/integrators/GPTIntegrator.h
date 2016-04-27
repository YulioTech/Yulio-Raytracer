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
		/// Classification of vertices into diffuse and glossy.
		enum VertexType {
			VERTEX_TYPE_GLOSSY,     ///< "Specular" vertex that requires the half-vector duplication shift.
			VERTEX_TYPE_DIFFUSE     ///< "Non-specular" vertex that is rough enough for the reconnection shift.
		};

		enum PathConnection {
			PATH_NOT_CONNECTED,      ///< Not yet connected - shifting in progress.
			PATH_RECENTLY_CONNECTED, ///< Connected, but different incoming direction so needs a BSDF evaluation.
			PATH_CONNECTED           ///< Connected, allows using BSDF values from the base path.
		};

		/*! Tracks the state of the path. */
		class __align(16) LightPath
		{
		public:

			/*! Constructs a path. */
			__forceinline LightPath(const Ray& ray, const Medium& medium = Medium::Vacuum(), const int depth = 0,
				const Color& throughput = one, const bool ignoreVisibleLights = false, const bool unbent = true)
				: lastRay(ray), lastMedium(medium), depth(depth), throughput(throughput), ignoreVisibleLights(ignoreVisibleLights), unbent(unbent) {}

			/*! Extends a light path. */
			__forceinline LightPath extended(const Ray& nextRay, const Medium& nextMedium, const Color& weight, float invBrdfPdf, const bool ignoreVL) const {
				LightPath lp(nextRay, nextMedium, depth + 1, throughput*weight, ignoreVL, unbent && (nextRay.dir == lastRay.dir));
				lp.misWeight = lp.throughput * invBrdfPdf;
				return lp;
			}


			/// Adds radiance to the path.
			__forceinline void addRadiance(const Color &contribution, float weight) {
				Color color = contribution * weight;
				radiance += color;
			}

			/// Adds gradient to the path.
			__forceinline void addGradient(const Color &contribution, float weight) {
				Color color = contribution * weight;
				gradient += color;
			}

		public:
			Ray lastRay;                 /*! Last ray in the path. */
			DifferentialGeometry lastDG; /*! Last differential geometry hit by the ray of the path. */
			Medium lastMedium;           /*! Medium the last ray travels inside. */
			uint32 depth = 0;                /*! Recursion depth of path. */
			Color misWeight = one;	/*! Inverse BRDF PDF for the last sample multiplied by throughput */
			Color throughput = zero;            /*! Determines the fraction of radiance that reaches the pixel along the path. */
			Color radiance = zero;				/*! Radiance accumulated so far. */
			Color gradient = zero;				/*! Gradient accumulated so far. */
			bool ignoreVisibleLights = false;    /*! If the previous shade point used shadow rays we have to ignore the emission
										 of geometrical lights to not double count them. */
			bool unbent = true;                 /*! True if the ray path is a straight line. */
			float pdf = 1.f;					/*! Current PDF of the path. */
			float eta = 1.f;					/*! Current refractive index of the ray */
			bool alive = true;					/*! Whether the path matching the ray is still good. it's an invalid offset path with zero PDF and throughput. */
			PathConnection connectionState = PATH_NOT_CONNECTED; 	/*! Whether the ray has been connected to the base path, or is in progress. */
			bool backFacing = false;
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

		void EvaluatePoint(Ray &centralRay, Ray *offsetRays,
			Color &outVeryDirect, Color &outThroughput,
			Color *outGradients, Color *outShiftedThroughputs,
			const Ref<BackendScene> &scene, IntegratorState &state);

	private:
		__forceinline VertexType getVertexTypeByRoughness(float roughness, float shiftThreshold) const;
		__forceinline VertexType getVertexType(const CompositedBRDF &brdfs, const DifferentialGeometry &dg, float shiftThreshold, unsigned int bsdfType) const;
		void evaluate(LightPath &basePath, LightPath *shiftedPaths, int shiftedCount, Color &outVeryDirect, const Ref<BackendScene>& scene, IntegratorState& state);

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

