#pragma once

#include "integrators/integrator.h"
#include "renderers/renderer.h"
#include "image/image.h"

namespace embree
{
	/// If defined, uses only the central sample for the throughput estimate. Otherwise uses offset paths for estimating throughput too.
	//#define GPT_CENTRAL_RADIANCE

	/*! Path tracer integrator. The implementation follows a single path
	*  from the camera into the scene and connect the path at each
	*  diffuse or glossy surface to all light sources. Except for this
	*  the path is never split, also not at glass surfaces. */

	class GPTIntegrator : public Integrator
	{
		/// Result of a reconnection shift.
		struct ReconnectionShiftResult {
			bool success = false;   ///< Whether the shift succeeded.
			float jacobian = 0.f; ///< Local Jacobian determinant of the shift.
			Vector3f wo = zero;     ///< World space outgoing vector for the shift.
		};

		/// Result of a half-vector duplication shift.
		struct HalfVectorShiftResult {
			bool success;   ///< Whether the shift succeeded.
			float jacobian; ///< Local Jacobian determinant of the shift.
			Vector3f wo;     ///< Tangent space outgoing vector for the shift.
		};

		/// Classification of vertices into diffuse and glossy.
		enum VertexType {
			VERTEX_TYPE_UNDEFINED,
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
				lp.lastDG = lastDG;
				lp.radiance = radiance;
				lp.gradient = gradient;
				lp.pdf = pdf;
				lp.eta = eta;
				lp.alive = alive;
				lp.connectionState = connectionState;
				lp.backFacing = backFacing;
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

			__forceinline bool isValid() const {
				return lastRay.tfar != std::numeric_limits<float>::infinity();
			}

		public:
			Ray lastRay;                 /*! Last ray in the path. */
			DifferentialGeometry lastDG; /*! Last differential geometry hit by the ray of the path. */
			Medium lastMedium;           /*! Medium the last ray travels inside. */
			uint32 depth = 0;                /*! Recursion depth of path. */
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
		__forceinline VertexType getVertexType(const CompositedBRDF &brdfs, const DifferentialGeometry &dg, float shiftThreshold, BRDFType bsdfType) const;
		__forceinline VertexType getVertexType(const LightPath &path, float shiftThreshold, BRDFType bsdfType) const;
		__forceinline ReconnectionShiftResult reconnectShift(const Ref<BackendScene> &scene, IntegratorState &state, Vector3f mainSourceVertex, Vector3f targetVertex, Vector3f shiftSourceVertex, Vector3f targetNormal, float time) const;
		__forceinline ReconnectionShiftResult environmentShift(const Ref<BackendScene> &scene, IntegratorState &state, const Ray& mainRay, Vector3f shiftSourceVertex) const;
		__forceinline HalfVectorShiftResult halfVectorShift(const Vector3f &mainWi, const Vector3f &mainWo, const DifferentialGeometry &mainDG, const Vector3f &shiftedWi, const DifferentialGeometry &shiftedDG, const BRDF *mainBrdf, const BRDF *shiftedBrdf) const;
		void evaluate(LightPath &basePath, LightPath *shiftedPaths, int shiftedCount, Color &outVeryDirect, const Ref<BackendScene>& scene, IntegratorState& state);

			/* Configuration. */
	private:
		size_t maxDepth;				//!< Maximal recursion depth (1=primary ray only)
		size_t rrDepth;					//!< Depth to begin using russian roulette
		size_t minDepth = 1;			//!< Minimal recursion depth (has to be greater than 0 to avoid "empty" images)
		float minContribution;			//!< Minimal contribution of a path to the pixel.
		float epsilon;					//!< Epsilon to avoid self intersections.
		float tMaxShadowRay;			//!< Max length of a shadow ray (i.e. an actual ray used to test for occlusion, not just any secondary, aka "shadow", ray)
		float tMaxShadowJitter;			//!< Percentage of the tMaxShadowRay that it can randomly vary by.
		Vector3f up;					//!< Up vector in world coordinates.
		Ref<Image> backplate;			//!< High resolution background.
		float glossyShiftThreshold;		//!< Threshold value below which a BRDF at a currently evaluated vertex is considered Glossy (otherwise its' considered Diffuse)

		/*! Random variables. */
	private:
		int lightSampleID;				//!< 2D random variable to sample the light source.
		int firstScatterSampleID;		//!< 2D random variable to sample the BRDF.
		int firstScatterTypeSampleID;	//!< 1D random variable to sample the BRDF type to choose.
		std::vector<int> precomputedLightSampleID;  //!< ID of precomputed light samples for lights that need pre-computations.
	};
}

