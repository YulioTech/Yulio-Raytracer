#include "GPTIntegrator.h"

#include "integrators/GPTIntegrator.h"
#include "brdfs/optics.h"

/// If defined, uses only the central sample for the throughput estimate. Otherwise uses offset paths for estimating throughput too.
//#define CENTRAL_RADIANCE

namespace embree
{
	/// A threshold to use in positive denominators to avoid division by zero.
	const float D_EPSILON = (float)(1e-14);

	GPTIntegrator::GPTIntegrator(const Parms& parms)
		: lightSampleID(-1), firstScatterSampleID(-1), firstScatterTypeSampleID(-1)
	{
		maxDepth = parms.getInt("maxDepth", 10);
		rrDepth = parms.getInt("rrDepth", max<int>(maxDepth / 2, 1));
		minContribution = parms.getFloat("minContribution", .01f);
		epsilon = parms.getFloat("epsilon", 32.f) * float(ulp);
		tMaxShadowRay = parms.getFloat("tMaxShadowRay", std::numeric_limits<float>::infinity());
		backplate = parms.getImage("backplate");
		glossyShiftThreshold = parms.getFloat("shiftThreshold", .001f);
	}

	void GPTIntegrator::requestSamples(Ref<SamplerFactory>& samplerFactory, const Ref<BackendScene>& scene)
	{
		precomputedLightSampleID.resize(scene->allLights.size());

		lightSampleID = samplerFactory->request2D();
		for (size_t i = 0; i < scene->allLights.size(); i++) {
			precomputedLightSampleID[i] = -1;
			if (scene->allLights[i]->precompute())
				precomputedLightSampleID[i] = samplerFactory->requestLightSample(lightSampleID, scene->allLights[i]);
		}
		firstScatterSampleID = samplerFactory->request2D((int)maxDepth);
		firstScatterTypeSampleID = samplerFactory->request1D((int)maxDepth);
	}

	// Non-recursive standard (one central ray) version
	Color GPTIntegrator::Li(LightPath& lightPath, const Ref<BackendScene>& scene, IntegratorState& state)
	{
#if 0
		BRDFType directLightingBRDFTypes = (BRDFType)(DIFFUSE | GLOSSY);
		BRDFType giBRDFTypes = (BRDFType)(SPECULAR);
#else
		BRDFType directLightingBRDFTypes = (BRDFType)(DIFFUSE);
		BRDFType giBRDFTypes = (BRDFType)(ALL);
#endif

		Color L = zero;
		float eta = 1.f; // Ray's relative refractive index (used by the Russian Roulette)

						 // Stop if the next ray is going to exceed the max allowed depth
		while (lightPath.depth < maxDepth) {
			if (lightPath.depth >= rrDepth - 1) {
				/* Russian roulette: try to keep path weights equal to one,
				while accounting for the solid angle compression at refractive
				index boundaries. Stop with at least some probability to avoid
				getting stuck (e.g. due to total internal reflection) */
				const float q = min(reduce_max(lightPath.throughput) * eta * eta, .95f);

				if (state.sample->getFloat(firstScatterTypeSampleID + lightPath.depth) >= q) {
					break;
				}
			}

			/*! Traverse ray. */
			DifferentialGeometry dg;
			//scene->intersector->intersect(lightPath.lastRay);
			rtcIntersect(scene->scene, (RTCRay&)lightPath.lastRay);
			scene->postIntersect(lightPath.lastRay, dg);
			state.numRays++;

			const Vector3f wo = -lightPath.lastRay.dir;

			/*! Environment shading when nothing is hit. */
			if (!lightPath.lastRay) {
				if (backplate && lightPath.unbent) {
					const int x = clamp(int(state.pixel.x * backplate->width), 0, int(backplate->width) - 1);
					const int y = clamp(int(state.pixel.y * backplate->height), 0, int(backplate->height) - 1);
					L += lightPath.throughput * backplate->get(x, y);
				}
				else {
					if (!lightPath.ignoreVisibleLights)
						for (size_t i = 0; i < scene->envLights.size(); i++)
							L += lightPath.throughput * scene->envLights[i]->Le(wo);
				}

				break;
			}

			/*! face forward normals */
			bool backfacing = false;
			if (dot(dg.Ng, lightPath.lastRay.dir) > 0.f) {
				backfacing = true; dg.Ng = -dg.Ng; dg.Ns = -dg.Ns;
			}

			//bool NsNotNg = dg.Ns != dg.Ng;
			//if (strictNormals && dot(lightPath.lastRay.dir, dg.Ng) * Frame::cosTheta(dg.wi) <= 0.f) {

			//	/* If 'strictNormals'=true, when the geometric and shading
			//	normals classify the incident direction to the same side */
			//	break;
			//}

			/*! Shade surface. */
			CompositedBRDF brdfs;
			if (dg.material)
				dg.material->shade(lightPath.lastRay, lightPath.lastMedium, dg, brdfs);

			/*! Add light emitted by hit area light source. */
			if (!lightPath.ignoreVisibleLights && dg.light && !backfacing)
				L += lightPath.throughput * dg.light->Le(dg, wo);

			/*! Check if any BRDF component uses direct lighting. */
			bool useDirectLighting = false;
			for (size_t i = 0; i < brdfs.size(); i++)
				useDirectLighting |= (brdfs[i]->type & directLightingBRDFTypes) != NONE;

			/*! Direct lighting. Shoot shadow rays to all light sources. */
			if (useDirectLighting) {
				for (size_t i = 0; i < scene->allLights.size(); i++) {
					if ((scene->allLights[i]->illumMask & dg.illumMask) == 0)
						continue;

					/*! Either use precomputed samples for the light or sample light now. */
					LightSample ls;
					if (scene->allLights[i]->precompute())
						ls = state.sample->getLightSample(precomputedLightSampleID[i]);
					else
						ls.L = scene->allLights[i]->sample(dg, ls, state.sample->getVec2f(lightSampleID));

					/*! Ignore zero radiance or illumination from the back. */
					//if (ls.L == Color(zero) || ls.wi.pdf == 0.0f || dot(dg.Ns,Vector3f(ls.wi)) <= 0.0f) continue; 
					if (ls.L == Color(zero) || ls.wi.pdf == 0.0f) continue;

					/*! Evaluate BRDF */
					Color brdf = brdfs.eval(wo, dg, ls.wi, directLightingBRDFTypes);
					if (brdf == Color(zero)) continue;

					//if (strictNormals && dot(lightPath.lastRay.dir, dg.Ng) * Frame::cosTheta(dg.wi) >= 0.f) {
					//	continue;
					//}

					/*! Test for shadows. */
					ls.tMax = tMaxShadowRay;
					Ray shadowRay(dg.P, ls.wi, dg.error*epsilon, ls.tMax - dg.error*epsilon, lightPath.lastRay.time, dg.shadowMask);
					//bool inShadow = scene->intersector->occluded(shadowRay);
					rtcOccluded(scene->scene, (RTCRay&)shadowRay);
					state.numRays++;
					if (shadowRay) continue;

					/*! Evaluate BRDF. */
					L += lightPath.throughput * ls.L * brdf * rcp(ls.wi.pdf);
				}
			}

			/*! Global illumination. Pick one BRDF component and sample it. */
			{
				/*! sample brdf */
				Sample3f sample; BRDFType type; size_t index;
				const Vec2f s = state.sample->getVec2f(firstScatterSampleID + lightPath.depth);
				const float ss = state.sample->getFloat(firstScatterTypeSampleID + lightPath.depth);
				Color c = brdfs.sample(wo, dg, sample, type, index, s, ss, giBRDFTypes);

				/*! Continue only if we hit something valid. */
				if (c == Color(zero) || sample.pdf <= 0.f) break;

				eta *= sample.eta;

				/*! Compute simple volumetric effect. */
				const Color& transmission = lightPath.lastMedium.transmission;
				if (transmission != Color(one)) {
					c *= pow(transmission, lightPath.lastRay.tfar);
				}

				/*! Terminate path if the contribution is too low. */
				if (reduce_max(lightPath.throughput * c) < minContribution)
					break;

				/*! Tracking medium if we hit a medium interface. */
				Medium nextMedium = lightPath.lastMedium;
				if (type & TRANSMISSION)
					nextMedium = dg.material->nextMedium(lightPath.lastMedium);

				/*! Continue the path. */
				lightPath = lightPath.extended(
					Ray(dg.P, sample, dg.error*epsilon, inf, lightPath.lastRay.time),
					nextMedium, c, rcp(sample.pdf), (type & directLightingBRDFTypes) != NONE
					);
			}
		} // while (lightPath.depth < maxDepth)

		return L;
	}

	Color GPTIntegrator::Li(Ray& ray, const Ref<BackendScene>& scene, IntegratorState& state) {
		LightPath path(ray);
		return Li(path, scene, state);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////

	/// Returns the vertex type of a vertex by its roughness value.
	GPTIntegrator::VertexType GPTIntegrator::getVertexTypeByRoughness(float roughness, float glossyShiftThreshold) const {
		if (roughness <= glossyShiftThreshold) {
			return VERTEX_TYPE_GLOSSY;
		}
		else {
			return VERTEX_TYPE_DIFFUSE;
		}
	}

	GPTIntegrator::VertexType GPTIntegrator::getVertexType(const LightPath &path, float glossyShiftThreshold, BRDFType bsdfType) const {
		if (!path.lastDG.material) return VERTEX_TYPE_UNDEFINED;

		CompositedBRDF pathBRDFs;
		path.lastDG.material->shade(path.lastRay, path.lastMedium, path.lastDG, pathBRDFs);

		return getVertexType(pathBRDFs, path.lastDG, glossyShiftThreshold, bsdfType);
	}

	/// Returns the vertex type (diffuse / glossy) of a vertex, for the purposes of determining
	/// the shifting strategy.
	///
	/// A bare classification by roughness alone is not good for multi-component BSDFs since they
	/// may contain a diffuse component and a perfect specular component. If the base path
	/// is currently working with a sample from a BSDF's smooth component, we don't want to care
	/// about the specular component of the BSDF right now - we want to deal with the smooth component.
	///
	/// For this reason, we vary the classification a little bit based on the situation.
	/// This is perfectly valid, and should be done.
	GPTIntegrator::VertexType GPTIntegrator::getVertexType(const CompositedBRDF &brdfs, const DifferentialGeometry &dg, float glossyShiftThreshold, BRDFType bsdfType) const {
		// Return the lowest roughness value of the components of the vertex's BSDF.
		// If 'bsdfType' does not have a delta component, do not take perfect speculars (zero roughness) into account in this.

		float lowestRoughness = std::numeric_limits<float>::infinity();

		bool foundSmooth = false;
		bool foundDirac = false;
		for (int i = 0, componentCount = brdfs.size(); i < componentCount && lowestRoughness != 0.f; ++i) {
			const auto brdf = brdfs[i];
			const float componentRoughness = brdf->roughness(dg);

			if (componentRoughness == 0.f) {
				foundDirac = true;
				if (!(bsdfType & SPECULAR)) {
					// Skip Dirac components if a smooth component is requested.
					continue;
				}
			}
			else {
				foundSmooth = true;
			}

			if (componentRoughness < lowestRoughness) {
				lowestRoughness = componentRoughness;
			}
		}

		// Roughness has to be zero also if there is a delta component but no smooth components.
		if (!foundSmooth && foundDirac && !(bsdfType & SPECULAR)) {
			lowestRoughness = 0.f;
		}

		return getVertexTypeByRoughness(lowestRoughness, glossyShiftThreshold);
	}

	/// Tries to connect the offset path to a specific vertex of the main path.
	GPTIntegrator::ReconnectionShiftResult GPTIntegrator::reconnectShift(const Ref<BackendScene> &scene, IntegratorState &state, Vector3f mainSourceVertex, Vector3f targetVertex, Vector3f shiftSourceVertex, Vector3f targetNormal, float time) const {
		ReconnectionShiftResult result;

		// Check visibility of the connection.
		//if (!testVisibility(scene, shiftSourceVertex, targetVertex, time)) {
		//	// Since this is not a light sample, we cannot allow shifts through occlusion.
		//	result.success = false;
		//	return result;
		//}

#ifdef DOUBLE_PRECISION
#define Epsilon 1e-7
#define ShadowEpsilon 1e-5
#else
#define Epsilon 1e-4f
#define ShadowEpsilon 1e-3f
#endif
		Ray shadowRay(shiftSourceVertex, targetVertex - shiftSourceVertex, Epsilon, 1.f - ShadowEpsilon, time);
		rtcOccluded(scene->scene, (RTCRay&)shadowRay);
		state.numRays++;

		if (shadowRay) {
			result.success = false;
			return result;
		}

		// Calculate the Jacobian.
		const Vector3f mainEdge = mainSourceVertex - targetVertex;
		const Vector3f shiftedEdge = shiftSourceVertex - targetVertex;

		const float mainEdgeLengthSquared = lengthSquared(mainEdge);
		const float shiftedEdgeLengthSquared = lengthSquared(shiftedEdge);

		const Vector3f shiftedWo = -shiftedEdge / sqrt(shiftedEdgeLengthSquared);

		const float mainOpposingCosine = dot(mainEdge, targetNormal) / sqrt(mainEdgeLengthSquared);
		const float shiftedOpposingCosine = dot(shiftedWo, targetNormal);

		const float jacobian = std::abs(shiftedOpposingCosine * mainEdgeLengthSquared) / (D_EPSILON + std::abs(mainOpposingCosine * shiftedEdgeLengthSquared));

		// Return the results.
		result.success = true;
		result.jacobian = jacobian;
		result.wo = shiftedWo;

		return result;
	}

	/// Tries to connect the offset path to a the environment emitter.
	GPTIntegrator::ReconnectionShiftResult GPTIntegrator::environmentShift(const Ref<BackendScene> &scene, IntegratorState &state, const Ray& mainRay, Vector3f shiftSourceVertex) const {
		ReconnectionShiftResult result;

		Ray shadowRay(mainRay.org, mainRay.dir, Epsilon, 1.f - ShadowEpsilon, mainRay.time);

		for (size_t i = 0; i < scene->envLights.size(); i++) {
			auto light = scene->envLights[i];

			// Needs work
			//Color L = light->sample(basePath.lastDG, baseLs, state.sample->getVec2f(lightSampleID));

			//// Check visibility of the environment.
			//if (!testEnvironmentVisibility(scene, mainRay)) {
			//	// Sampled by BSDF so cannot accept occlusion.
			//	result.success = false;
			//	return result;
			//}

			Ray lightRay(mainRay.org, mainRay.dir, 0.f, inf, mainRay.time);
			float tNear, tFar;
			if (!light->rayIntersect(lightRay, tNear, tFar)) {
				result.success = false;
				return result;
			}

			Ray shadowRay(mainRay.org, mainRay.dir, Epsilon, (1.f - ShadowEpsilon) * tFar, mainRay.time);
			rtcOccluded(scene->scene, (RTCRay&)shadowRay);
			state.numRays++;

			if (shadowRay) {
				result.success = false;
				return result;
			}


			// Return the results.
			result.success = true;
			result.jacobian = 1.f;
			result.wo = mainRay.dir;
		}

		return result;
	}

	/// Calculates the outgoing direction of a shift by duplicating the local half-vector.
	GPTIntegrator::HalfVectorShiftResult GPTIntegrator::halfVectorShift(const Vector3f &mainWi, const Vector3f &mainWo, const DifferentialGeometry &mainDG, const Vector3f &shiftedWi, const DifferentialGeometry &shiftedDG, const BRDF *mainBrdf, const BRDF *shiftedBrdf) const {
		HalfVectorShiftResult result;

		//if (Frame::cosTheta(tangentSpaceMainWi) * Frame::cosTheta(tangentSpaceMainWo) < 0.f) {
		if (dot(mainWi, mainDG.Ns) * dot(mainWo, mainDG.Ns) < 0.f) {
			// Refraction.

			// Refuse to shift if one of the Etas is exactly 1. This causes degenerate half-vectors.
			const float mainEta = mainBrdf->eta();
			const float shiftedEta = shiftedBrdf->eta();
			const bool noRefraction = (mainEta == 1.f || shiftedEta == 1.f);
			if (noRefraction) {
				// This could be trivially handled as a special case if ever needed.
				//result.success = false;

				//halfVectorNonNormalizedMain = (mainDG.Ns + shiftedDG.Ns) *.5f;
				//result.wo = -normalize(mainWi + shiftedWi);
				result.wo = -mainWi;
				result.jacobian = 1.f;
				result.success = true;

				return result;
			}

			// Get the non-normalized half vector.
			Vector3f halfVectorNonNormalizedMain;
				if (dot(mainWi, mainDG.Ns) < 0.f) {
					halfVectorNonNormalizedMain = (mainWi * mainEta + mainWo);
				}
				else {
					halfVectorNonNormalizedMain = (mainWi + mainWo * mainEta);
				}

			// Get the normalized half vector.
			const Vector3f halfVectorMain = normalize(halfVectorNonNormalizedMain);

#if 0
			// Lev: use the actual BRDF sampling method instead of a refraction function, since it always produces the correct implementation-dependent result
			Sample3f wo;
			DifferentialGeometry dg;
			dg.Ns = halfVectorMain;
			Vec2f s = zero;
			if (zero == shiftedBrdf->sample(shiftedWi, dg, wo, s)) {
				result.success = false;
				return result;
			}

			const Vector3f shiftedWo = wo;
#else
			// Refract to get the outgoing direction.
			const Vector3f shiftedWo = refract(shiftedWi, halfVectorMain, shiftedEta);
#endif

			// Refuse to shift between transmission and full internal reflection.
			// This shift would not be invertible: reflections always shift to other reflections.
			if (shiftedWo == zero) {
				result.success = false;
				return result;
			}

			if (noRefraction) {
				result.wo = shiftedWo;
				const float hLengthSquared = rcp(lengthSquared(halfVectorNonNormalizedMain));
				const float WoDotH = abs(dot(mainWo, halfVectorMain)) / (D_EPSILON + abs(dot(shiftedWo, halfVectorMain)));
				result.jacobian = WoDotH * hLengthSquared;
				result.success = true;
			}
			else {
				// Calculate the Jacobian.
				Vector3f halfVectorNonNormalizedShifted;
				if (dot(shiftedWi, shiftedDG.Ns) < 0.f) {
					halfVectorNonNormalizedShifted = (shiftedWi * shiftedEta + shiftedWo);
				}
				else {
					halfVectorNonNormalizedShifted = (shiftedWi + shiftedWo * shiftedEta);
				}

				const float hLengthSquared = lengthSquared(halfVectorNonNormalizedShifted) / (D_EPSILON + lengthSquared(halfVectorNonNormalizedMain));
				const float WoDotH = abs(dot(mainWo, halfVectorMain)) / (D_EPSILON + abs(dot(shiftedWo, halfVectorMain)));

				// Output results.
				result.wo = shiftedWo;
				result.jacobian = hLengthSquared * WoDotH;
				result.success = true;
			}
		} else {
			// Reflection.
			const Vector3f halfVector = normalize(mainWi + mainWo);
			const Vector3f shiftedWo = reflect(shiftedWi, halfVector);

			const float WoDotH = dot(shiftedWo, halfVector) / dot(mainWo, halfVector);
			const float jacobian = abs(WoDotH);

			result.wo = shiftedWo;
			result.jacobian = jacobian;
			result.success = true;
		}

		return result;
	}

	void GPTIntegrator::evaluate(LightPath &basePath, LightPath *shiftedPaths, int shiftedCount, Color &outVeryDirect, const Ref<BackendScene> &scene, IntegratorState &state)
	{
		static const BRDFType directLightingBRDFTypes = (BRDFType)(DIFFUSE);
		static const BRDFType giBRDFTypes = (BRDFType)(ALL);

		// Perform the first ray intersection for the base path
		rtcIntersect(scene->scene, (RTCRay&)basePath.lastRay);
		state.numRays++;

		Vector3f baseWo = -basePath.lastRay.dir;

		/*! Environment shading when nothing is hit. */
		if (!basePath.lastRay) {
			if (backplate && basePath.unbent) {
				const int x = clamp(int(state.pixel.x * backplate->width), 0, int(backplate->width) - 1);
				const int y = clamp(int(state.pixel.y * backplate->height), 0, int(backplate->height) - 1);
				outVeryDirect += basePath.throughput * backplate->get(x, y);
			}
			else {
				if (!basePath.ignoreVisibleLights)
					for (size_t i = 0; i < scene->envLights.size(); i++)
						outVeryDirect += basePath.throughput * scene->envLights[i]->Le(baseWo);
			}

			// First hit is not in the scene so can't continue. Also there are no paths to shift.
			return;
		}

		// Get the hit differential geometry.
		scene->postIntersect(basePath.lastRay, basePath.lastDG);

		/*! face forward normals */
		basePath.backFacing = false;
		if (dot(basePath.lastDG.Ng, basePath.lastRay.dir) > 0) {
			basePath.backFacing = true; basePath.lastDG.Ng = -basePath.lastDG.Ng; basePath.lastDG.Ns = -basePath.lastDG.Ns;
		}

		/*! Add light emitted by hit area light source. */
		if (!basePath.ignoreVisibleLights && basePath.lastDG.light && !basePath.backFacing)
			outVeryDirect += basePath.throughput * basePath.lastDG.light->Le(basePath.lastDG, baseWo);

		// Perform the same first ray intersection for the offset paths.
		for (int i = 0; i < shiftedCount; ++i) {
			LightPath &shiftedPath = shiftedPaths[i];
			rtcIntersect(scene->scene, (RTCRay&)shiftedPath.lastRay);
			state.numRays++;

			// If no intersection of an offset ray could be found, its offset paths can not be generated.
			if (!shiftedPath.lastRay) {
				shiftedPath.alive = false;
			}
			else {
				// Get the hit differential geometry.
				scene->postIntersect(shiftedPath.lastRay, shiftedPath.lastDG);

				/*! face forward normals */
				shiftedPath.backFacing = false;
				if (dot(shiftedPath.lastDG.Ng, shiftedPath.lastRay.dir) > 0) {
					shiftedPath.backFacing = true; shiftedPath.lastDG.Ng = -shiftedPath.lastDG.Ng; shiftedPath.lastDG.Ns = -shiftedPath.lastDG.Ns;
				}
			}
		}

		/*! Traverse ray. */
		while (basePath.depth < maxDepth) {
			/*! Terminate path if the contribution is too low. */
			if (reduce_max(basePath.throughput) < minContribution)
				break;

			// Some optimizations can be made if this is the last traced segment.
			const bool lastSegment = (basePath.depth + 1 == maxDepth);

			/*! Shade surface. */
			CompositedBRDF baseBRDFs;
			if (basePath.lastDG.material)
				basePath.lastDG.material->shade(basePath.lastRay, basePath.lastMedium, basePath.lastDG, baseBRDFs);

			/*! Check if any BRDF component uses direct lighting. */
			bool useDirectLighting = false;
			for (size_t i = 0; i < baseBRDFs.size() && !useDirectLighting; ++i)
				useDirectLighting |= (baseBRDFs[i]->type & directLightingBRDFTypes) != NONE;

			/*! Direct lighting. Shoot shadow rays to all light sources. */
			if (useDirectLighting) {
				for (size_t i = 0; i < scene->allLights.size(); i++) {
					Ref<Light> light = scene->allLights[i];
					if ((light->illumMask & basePath.lastDG.illumMask) == 0)
						continue;

					/*! Either use precomputed samples for the light or sample light now. */
					LightSample baseLs;
					Color baseBrdf = zero;
					if (light->precompute())
						baseLs = state.sample->getLightSample(precomputedLightSampleID[i]);
					else
						baseLs.L = light->sample(basePath.lastDG, baseLs, state.sample->getVec2f(lightSampleID));

#if 0
					/*! Ignore zero radiance or illumination from the back. */
					//if (ls.L == Color(zero) || ls.wi.pdf == 0.0f || dot(dg.Ns,Vector3f(ls.wi)) <= 0.0f) continue; 
					if (ls.L == Color(zero) || ls.wi.pdf == 0.0f) continue;
					
					/*! Evaluate BRDF */
					Color brdf = baseBRDFs.eval(wo, basePath.lastDG, ls.wi, directLightingBRDFTypes);
					if (brdf == Color(zero)) continue;
#else
					if (baseLs.L != Color(zero) && baseLs.wi.pdf > 0.f) {
						baseBrdf = baseBRDFs.eval(baseWo, basePath.lastDG, baseLs.wi, directLightingBRDFTypes);
					}
#endif

					/*! Test for shadows. */
					// Lev: limit the max shadow ray length to introduce some fake direct lighting into the scene.
					// This is done to allow the lighting of the indoor scenes without having to use actually use any additional lights.
					baseLs.tMax = tMaxShadowRay;
					Ray baseShadowRay(basePath.lastDG.P, baseLs.wi, basePath.lastDG.error*epsilon, baseLs.tMax - basePath.lastDG.error*epsilon, basePath.lastRay.time, basePath.lastDG.shadowMask);
					rtcOccluded(scene->scene, (RTCRay&)baseShadowRay);
					state.numRays++;

					// Stuff from Mitsuba "translated" into our framework
					{
						const bool mainEmitterVisible = !baseShadowRay;
						const Color mainEmitterRadiance = mainEmitterVisible ? baseLs.L : zero;
						const Color mainBSDFValue = baseBrdf;

						// Calculate the probability density of having generated the sampled path segment by BSDF sampling. Note that if the emitter is not visible, the probability density is zero.
						// Even if the BSDF sampler has zero probability density, the light sampler can still sample it.
						const float mainBsdfPdf = mainEmitterVisible ? baseBRDFs.pdf(baseWo, basePath.lastDG, baseLs.wi, directLightingBRDFTypes) : 0.f;

						// There values are probably needed soon for the Jacobians.
						const float mainDistanceSquared = lengthSquared(basePath.lastDG.P - baseLs.p);
						const float mainOpposingCosine = dot(baseLs.n, (basePath.lastDG.P - baseLs.p)) / sqrt(mainDistanceSquared);

						// Power heuristic weights for the following strategies: light sample from base, BSDF sample from base.
						const float mainWeightNumerator = basePath.pdf * baseLs.wi.pdf;
						const float mainWeightDenominator = (basePath.pdf * basePath.pdf) * ((baseLs.wi.pdf * baseLs.wi.pdf) + (mainBsdfPdf * mainBsdfPdf));

#ifdef CENTRAL_RADIANCE
						basePath.addRadiance(basePath.throughput * (mainBSDFValue * mainEmitterRadiance), mainWeightNumerator / (D_EPSILON + mainWeightDenominator));
#endif
						// The base path is good. Add radiance differences to offset paths.
						for (int j = 0; j < shiftedCount; ++j) {
							// Evaluate and apply the gradient.
							LightPath &shiftedPath = shiftedPaths[j];
							const Vector3f shiftedWo = -shiftedPath.lastRay.dir;

							Color mainContribution = zero;
							Color shiftedContribution = zero;
							float weight = 0.f;

							bool shiftSuccessful = shiftedPath.alive;

							// Construct the offset path.
							if (shiftSuccessful) {
								// Generate the offset path.
								if (shiftedPath.connectionState == PATH_CONNECTED) {
									// Follow the base path. All relevant vertices are shared. 
									const float shiftedBsdfPdf = mainBsdfPdf;
									const float shiftedDRecPdf = baseLs.wi.pdf;
									const Color shiftedBsdfValue = mainBSDFValue;
									const Color shiftedEmitterRadiance = mainEmitterRadiance;
									const float jacobian = 1.f;

									// Power heuristic between light sample from base, BSDF sample from base, light sample from offset, BSDF sample from offset.
									const float shiftedWeightDenominator = (jacobian * shiftedPath.pdf) * (jacobian * shiftedPath.pdf) * ((shiftedDRecPdf * shiftedDRecPdf) + (shiftedBsdfPdf * shiftedBsdfPdf));
									weight = mainWeightNumerator / (D_EPSILON + shiftedWeightDenominator + mainWeightDenominator);

									mainContribution = basePath.throughput * (mainBSDFValue * mainEmitterRadiance);
									shiftedContribution = jacobian * shiftedPath.throughput * (shiftedBsdfValue * shiftedEmitterRadiance);

									// Note: The Jacobians were baked into shifted.pdf and shifted.throughput at connection phase.
								}
								else if (shiftedPath.connectionState == PATH_RECENTLY_CONNECTED) {
									// Follow the base path. The current vertex is shared, but the incoming directions differ.
									//const Vector3f incomingDirection = normalize(shiftedPath.lastDG.P - basePath.lastDG.P);
									const Vector3f incomingDirection = normalize(shiftedPath.lastRay.org - basePath.lastDG.P);
									

									//BSDFSamplingRecord bRec(main.rRec.its, main.rRec.its.toLocal(incomingDirection), main.rRec.its.toLocal(dRec.d), ERadiance);

									// Sample the BSDF.
									//Float shiftedBsdfPdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle && mainEmitterVisible) ? mainBSDF->pdf(bRec) : 0; // The BSDF sampler can not sample occluded path segments.
									//const float shiftedBsdfPdf = mainEmitterVisible ? baseBRDFs.pdf(baseLs.wi, basePath.lastDG, incomingDirection, directLightingBRDFTypes) : 0.f;
									//const Color shiftedBsdfValue = baseBRDFs.eval(baseLs.wi, basePath.lastDG, incomingDirection, directLightingBRDFTypes);
									const float shiftedBsdfPdf = mainEmitterVisible ? baseBRDFs.pdf(incomingDirection, basePath.lastDG, baseLs.wi, directLightingBRDFTypes) : 0.f;
									const Color shiftedBsdfValue = baseBRDFs.eval(incomingDirection, basePath.lastDG, baseLs.wi, directLightingBRDFTypes);
									const float shiftedDRecPdf = baseLs.wi.pdf;
									const Color shiftedEmitterRadiance = mainEmitterRadiance;
									const float jacobian = 1.f;

									// Power heuristic between light sample from base, BSDF sample from base, light sample from offset, BSDF sample from offset.
									const float shiftedWeightDenominator = (jacobian * shiftedPath.pdf) * (jacobian * shiftedPath.pdf) * ((shiftedDRecPdf * shiftedDRecPdf) + (shiftedBsdfPdf * shiftedBsdfPdf));
									weight = mainWeightNumerator / (D_EPSILON + shiftedWeightDenominator + mainWeightDenominator);

									mainContribution = basePath.throughput * (mainBSDFValue * mainEmitterRadiance);
									shiftedContribution = jacobian * shiftedPath.throughput * (shiftedBsdfValue * shiftedEmitterRadiance);

									// Note: The Jacobians were baked into shiftedPath.pdf and shiftedPath.throughput at connection phase.
								}
								else {
									// Reconnect to the sampled light vertex. No shared vertices.
									assert(shiftedPath.connectionState == PATH_NOT_CONNECTED);

									CompositedBRDF shiftedBRDFs;
									if (shiftedPath.lastDG.material)
										shiftedPath.lastDG.material->shade(shiftedPath.lastRay, shiftedPath.lastMedium, shiftedPath.lastDG, shiftedBRDFs);

									// This implementation uses light sampling only for the reconnect-shift.
									// When one of the BSDFs is very glossy, light sampling essentially reduces to a failed shift anyway.
									bool mainAtPointLight = false;// (dRec.measure == EDiscrete);

									const BRDFType brdfTypeToTest = (BRDFType)(SMOOTH);
									const VertexType mainVertexType = getVertexType(baseBRDFs, basePath.lastDG, glossyShiftThreshold, brdfTypeToTest);
									const VertexType shiftedVertexType = getVertexType(shiftedBRDFs, shiftedPath.lastDG, glossyShiftThreshold, brdfTypeToTest);

									if (mainAtPointLight || (mainVertexType == VERTEX_TYPE_DIFFUSE && shiftedVertexType == VERTEX_TYPE_DIFFUSE)) {
										// Get emitter radiance.
										//DirectSamplingRecord shiftedDRec(shifted.rRec.its);
										//std::pair<Spectrum, bool> emitterTuple = m_scene->sampleEmitterDirectVisible(shiftedDRec, lightSample);
										//bool shiftedEmitterVisible = emitterTuple.second;
										LightSample shiftedLs;
										Color shiftedBrdf = zero;
										if (light->precompute())
											shiftedLs = state.sample->getLightSample(precomputedLightSampleID[j]);
										else
											shiftedLs.L = light->sample(shiftedPath.lastDG, shiftedLs, state.sample->getVec2f(lightSampleID));

										/*! Test for shadows. */
										shiftedLs.tMax = baseLs.tMax;
										Ray shiftedShadowRay(shiftedPath.lastDG.P, shiftedLs.wi, shiftedPath.lastDG.error*epsilon, shiftedLs.tMax - shiftedPath.lastDG.error*epsilon, shiftedPath.lastRay.time, shiftedPath.lastDG.shadowMask);
										rtcOccluded(scene->scene, (RTCRay&)shiftedShadowRay);
										state.numRays++;
										const bool shiftedEmitterVisible = !shiftedShadowRay;


										//Spectrum shiftedEmitterRadiance = emitterTuple.first * shiftedDRec.pdf;
										//Float shiftedDRecPdf = shiftedDRec.pdf;
										const Color shiftedEmitterRadiance = shiftedEmitterVisible ? shiftedLs.L : zero;
										const float shiftedDRecPdf = shiftedLs.wi.pdf;

										// Sample the BSDF.
										//float shiftedDistanceSquared = (dRec.p - shifted.rRec.its.p).lengthSquared();
										//Vector emitterDirection = (dRec.p - shifted.rRec.its.p) / sqrt(shiftedDistanceSquared);
										//Float shiftedOpposingCosine = -dot(dRec.n, emitterDirection);

										const float shiftedDistanceSquared = lengthSquared(shiftedLs.p - shiftedPath.lastDG.P);
										const Vector3f emitterDirection = (shiftedLs.p - shiftedPath.lastDG.P) / sqrt(shiftedDistanceSquared);
										const float shiftedOpposingCosine = -dot(shiftedLs.n, emitterDirection);


										//BSDFSamplingRecord bRec(shifted.rRec.its, shifted.rRec.its.toLocal(emitterDirection), ERadiance);

										// Strict normals check, to make the output match with bidirectional methods when normal maps are present.
										//if (m_config->m_strictNormals && dot(shifted.rRec.its.geoFrame.n, emitterDirection) * Frame::cosTheta(bRec.wo) < 0) {
										//	// Invalid, non-sampleable offset path.
										//	shiftSuccessful = false;
										//}
										//else
										{
											//Spectrum shiftedBsdfValue = shiftedBSDF->eval(bRec);
											//Float shiftedBsdfPdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle && shiftedEmitterVisible) ? shiftedBSDF->pdf(bRec) : 0;
											//Float jacobian = std::abs(shiftedOpposingCosine * mainDistanceSquared) / (Epsilon + std::abs(mainOpposingCosine * shiftedDistanceSquared));

											if (shiftedEmitterVisible && shiftedLs.L != Color(zero) && shiftedLs.wi.pdf > 0.f) {
												shiftedBrdf = shiftedBRDFs.eval(shiftedWo, shiftedPath.lastDG, emitterDirection/*shiftedLs.wi*/, directLightingBRDFTypes);
											}
											
											const Color shiftedBsdfValue = shiftedBrdf;
											const float shiftedBsdfPdf = shiftedEmitterVisible ? shiftedBRDFs.pdf(shiftedWo, shiftedPath.lastDG, emitterDirection/*shiftedLs.wi*/, directLightingBRDFTypes) : 0.f;
											const float jacobian = std::abs(shiftedOpposingCosine * mainDistanceSquared) / (D_EPSILON + std::abs(mainOpposingCosine * shiftedDistanceSquared));

											if (shiftedBsdfPdf > 0.f) {
												int n = 0;
											}

											// Power heuristic between light sample from base, BSDF sample from base, light sample from offset, BSDF sample from offset.
											const float shiftedWeightDenominator = (jacobian * shiftedPath.pdf) * (jacobian * shiftedPath.pdf) * ((shiftedDRecPdf * shiftedDRecPdf) + (shiftedBsdfPdf * shiftedBsdfPdf));
											weight = mainWeightNumerator / (D_EPSILON + shiftedWeightDenominator + mainWeightDenominator);

											mainContribution = basePath.throughput * (mainBSDFValue * mainEmitterRadiance);
											shiftedContribution = jacobian * shiftedPath.throughput * (shiftedBsdfValue * shiftedEmitterRadiance);
										}
									}
								}
							} // if (shiftSuccessful) 

							if (!shiftSuccessful) {
								// The offset path cannot be generated; Set offset PDF and offset throughput to zero. This is what remains.

								// Power heuristic between light sample from base, BSDF sample from base, light sample from offset, BSDF sample from offset. (Offset path has zero PDF)
								const float shiftedWeightDenominator = 0.f;
								weight = mainWeightNumerator / (D_EPSILON + mainWeightDenominator);

								mainContribution = basePath.throughput * (mainBSDFValue * mainEmitterRadiance);
								shiftedContribution = Color(0.f);
							}

#ifndef CENTRAL_RADIANCE
							// Note: Using also the offset paths for the throughput estimate, like we do here, provides some advantage when a large reconstruction alpha is used,
							// but using only throughputs of the base paths doesn't usually lose by much.
							basePath.addRadiance(mainContribution, weight);
							shiftedPath.addRadiance(shiftedContribution, weight);
#endif

							shiftedPath.addGradient(shiftedContribution - mainContribution, weight);

						} // for (int j = 0; j < shiftedCount; ++j)
					} // Stuff from Mitsuba "translated" into our framework
				} // for (size_t i = 0; i < scene->allLights.size(); i++)
			} // if (useDirectLighting)

			/*! Global illumination. Pick one BRDF component and sample it. */
			{
				/*! sample brdf */
				Sample3f baseBrdfSample;
				BRDFType baseSampledType;
				size_t baseSampledIndex;
				const Vec2f s = state.sample->getVec2f(firstScatterSampleID + basePath.depth);
				const float ss = state.sample->getFloat(firstScatterTypeSampleID + basePath.depth);
				Color baseBrdfWeight = baseBRDFs.sample(baseWo, basePath.lastDG, baseBrdfSample, baseSampledType, baseSampledIndex, s, ss, giBRDFTypes);
				//const float baseBrdfPdf = baseBRDFs.pdf(baseWo, basePath.lastDG, baseBrdfSample, baseSampledType);

				/*! Continue only if we hit something valid. */
				// If PDF <= 0, impossible base path.
				if (baseBrdfWeight == Color(zero) || baseBrdfSample.pdf <= 0.f) {
					// Impossible base path.
					break;
				}

				// Since we pre-multiply brdf by rcp(pdf) in the sampling function above, we need to multiply it again to get the unweighted value
				baseBrdfWeight *= rcp(baseBrdfSample.pdf);

				/*! Compute simple volumetric effect. */
				const Color& transmission = basePath.lastMedium.transmission;
				if (transmission != Color(one)) {
					baseBrdfWeight *= pow(transmission, basePath.lastRay.tfar);
				}

				/*! Tracking medium if we hit a medium interface. */
				Medium baseNextMedium = basePath.lastMedium;
				if (baseSampledType & TRANSMISSION)
					baseNextMedium = basePath.lastDG.material->nextMedium(basePath.lastMedium);

				// Stuff from Mitsuba "translated" into our framework
				{
					// The old intersection structure is still needed after main.rRec.its gets updated.
					const LightPath previousBasePath = basePath;

					// Trace a ray in the sampled direction.
					bool mainHitEmitter = false;
					Color mainEmitterRadiance = zero;
					float mainLumPdf = 0;

					// Update the vertex types.
					const VertexType mainVertexType = getVertexType(baseBRDFs, basePath.lastDG, glossyShiftThreshold, baseBRDFs.type());
					VertexType mainNextVertexType;

					/*! Continue the path. */
					//basePath = basePath.extended(Ray(basePath.lastDG.P, baseBrdfSample, basePath.lastDG.error*epsilon, inf, basePath.lastRay.time), nextMedium, baseBrdfWeight, 1.f/*rcp(baseBrdfSample.pdf)*/, (type & directLightingBRDFTypes) != NONE);
					const Ray newRay(basePath.lastDG.P, baseBrdfSample, basePath.lastDG.error*epsilon, inf, basePath.lastRay.time);
					//basePath.throughput *= baseBrdfWeight
					basePath.lastMedium = baseNextMedium;
					basePath.ignoreVisibleLights = (baseSampledType & directLightingBRDFTypes) != NONE;
					basePath.unbent = (basePath.unbent && basePath.lastRay.dir == newRay.dir);
					basePath.lastRay = newRay;

					// Already done above (see basePath.extended method)
					//main.ray = Ray(main.rRec.its.p, mainWo, main.ray.time);
					rtcIntersect(scene->scene, (RTCRay&)basePath.lastRay);
					state.numRays++;

					baseWo = -basePath.lastRay.dir;

					// Get the hit differential geometry.
					scene->postIntersect(basePath.lastRay, basePath.lastDG);

					/*! face forward normals */
					basePath.backFacing = false;
					if (dot(basePath.lastDG.Ng, basePath.lastRay.dir) > 0) {
						basePath.backFacing = true; basePath.lastDG.Ng = -basePath.lastDG.Ng; basePath.lastDG.Ns = -basePath.lastDG.Ns;
					}

					/*! Environment shading when nothing is hit. */
					if (!basePath.lastRay) {
						if (backplate && basePath.unbent) {
							const int x = clamp(int(state.pixel.x * backplate->width), 0, int(backplate->width) - 1);
							const int y = clamp(int(state.pixel.y * backplate->height), 0, int(backplate->height) - 1);
							mainEmitterRadiance += backplate->get(x, y);
							mainLumPdf = 1;
							mainHitEmitter = true;
						}
						else {
							if (!basePath.ignoreVisibleLights) {
								mainLumPdf = 1.f;
								for (size_t i = 0; i < scene->envLights.size(); i++) {
									mainEmitterRadiance += scene->envLights[i]->Le(baseWo);
									const float pdf = scene->envLights[i]->pdf(basePath.lastDG, baseWo);
									mainLumPdf *= pdf;
									mainHitEmitter = true;
								}
							}
						}

						if (!mainHitEmitter) {
							// Nothing to do anymore.
							break;
						}

						// Handle environment connection as diffuse (that's ~infinitely far away).
						// Update the vertex type.
						mainNextVertexType = VERTEX_TYPE_DIFFUSE;
					}
					else {
						// Intersected something - check if it was a luminaire.
						/*! Add light emitted by hit area light source. */
						if (!basePath.ignoreVisibleLights && basePath.lastDG.light && !basePath.backFacing) {
							mainEmitterRadiance = basePath.lastDG.light->eval(basePath.lastDG, baseWo);
							mainLumPdf = basePath.lastDG.light->pdf(basePath.lastDG, baseWo);

							mainHitEmitter = true;
						}

						// Update the vertex type.
						mainNextVertexType = getVertexType(basePath, glossyShiftThreshold, baseBRDFs.type());
					}

					// Continue the shift.
					const float mainBsdfPdf = baseBrdfSample.pdf;
					const float mainPreviousPdf = basePath.pdf;

					basePath.throughput *= baseBrdfWeight * mainBsdfPdf;
					basePath.pdf *= mainBsdfPdf;
					basePath.eta *= baseBrdfSample.eta;
					//basePath.ignoreVisibleLights = (type & directLightingBRDFTypes) != NONE;
					//basePath.addRadiance(mainEmitterRadiance * basePath.throughput, 1.f);
					//basePath.depth++;
					//continue;

					// Compute the probability density of generating base path's direction using the implemented direct illumination sampling technique.
					//if (mainHitEmitter && basePath.depth + 1 >= minDepth) {
					//	if (basePath.lastDG.light) {
					//		mainLumPdf = basePath.lastDG.light->pdf(basePath.lastDG, baseWo);
					//	}
					//	else {
					//		mainLumPdf = 1;
					//		for (size_t i = 0; i < scene->envLights.size(); i++) {
					//			mainLumPdf *= scene->envLights[i]->pdf(basePath.lastDG, baseWo);
					//		}
					//	}
					//}


					// Power heuristic weights for the following strategies: light sample from base, BSDF sample from base.
					const float mainWeightNumerator = mainPreviousPdf * mainBsdfPdf;
					const float mainWeightDenominator = (mainPreviousPdf * mainPreviousPdf) * ((mainLumPdf * mainLumPdf) + (mainBsdfPdf * mainBsdfPdf));

#ifdef CENTRAL_RADIANCE
					if (basePath.depth + 1 >= minDepth) {
						basePath.addRadiance(basePath.throughput * mainEmitterRadiance, mainWeightNumerator / (D_EPSILON + mainWeightDenominator));
					}
#endif
					for (int j = 0; j < shiftedCount; ++j) {
						// Evaluate and apply the gradient.
						LightPath &shiftedPath = shiftedPaths[j];

						Color shiftedEmitterRadiance = zero;
						Color mainContribution = zero;
						Color shiftedContribution = zero;
						float weight = 0.f;

						bool postponedShiftEnd = false; // Kills the shift after evaluating the current radiance.

						if (shiftedPath.alive) {
							// The offset path is still good, so it makes sense to continue its construction.
							const float shiftedPreviousPdf = shiftedPath.pdf;

							if (shiftedPath.connectionState == PATH_CONNECTED) {
								// The offset path keeps following the base path.
								// As all relevant vertices are shared, we can just reuse the sampled values.
								const Color shiftedBsdfValue = baseBrdfWeight * mainBsdfPdf;
								const float shiftedBsdfPdf = mainBsdfPdf;
								const float shiftedLumPdf = mainLumPdf;
								const Color shiftedEmitterRadiance = mainEmitterRadiance;

								// Update throughput and pdf.
								shiftedPath.throughput *= shiftedBsdfValue;
								shiftedPath.pdf *= shiftedBsdfPdf;

								// Power heuristic between light sample from base, BSDF sample from base, light sample from offset, BSDF sample from offset.
								const float shiftedWeightDenominator = (shiftedPreviousPdf * shiftedPreviousPdf) * ((shiftedLumPdf * shiftedLumPdf) + (shiftedBsdfPdf * shiftedBsdfPdf));
								weight = mainWeightNumerator / (D_EPSILON + shiftedWeightDenominator + mainWeightDenominator);

								mainContribution = basePath.throughput * mainEmitterRadiance;
								shiftedContribution = shiftedPath.throughput * shiftedEmitterRadiance; // Note: Jacobian baked into .throughput.
							}
							else if (shiftedPath.connectionState == PATH_RECENTLY_CONNECTED) {
								// Recently connected - follow the base path but evaluate BSDF to the new direction.
								//const Vector3f incomingDirection = normalize(shiftedPath.lastDG.P - basePath.lastRay.org);
								const Vector3f incomingDirection = normalize(shiftedPath.lastRay.org - basePath.lastDG.P);

								//BSDFSamplingRecord bRec(previousMainIts, previousMainIts.toLocal(incomingDirection), previousMainIts.toLocal(main.ray.d), ERadiance);

								// Note: mainBSDF is the BSDF at previousMainIts, which is the current position of the offset path.

								//EMeasure measure = (mainBsdfResult.bRec.sampledType & BSDF::EDelta) ? EDiscrete : ESolidAngle;
								//Color shiftedBsdfValue = baseBRDFs.eval(basePath.lastRay.dir, previousBaseDG, incomingDirection, directLightingBRDFTypes);
								//const float shiftedBsdfPdf = baseBRDFs.pdf(basePath.lastRay.dir, previousBaseDG, incomingDirection, directLightingBRDFTypes);
								const Color shiftedBsdfValue = baseBRDFs.eval(incomingDirection, previousBasePath.lastDG, basePath.lastRay.dir, directLightingBRDFTypes);
								const float shiftedBsdfPdf = baseBRDFs.pdf(incomingDirection, previousBasePath.lastDG, basePath.lastRay.dir, directLightingBRDFTypes);

								const float shiftedLumPdf = mainLumPdf;
								const Color shiftedEmitterRadiance = mainEmitterRadiance;

								// Update throughput and pdf.
								shiftedPath.throughput *= shiftedBsdfValue;
								shiftedPath.pdf *= shiftedBsdfPdf;

								shiftedPath.connectionState = PATH_CONNECTED;

								// Power heuristic between light sample from base, BSDF sample from base, light sample from offset, BSDF sample from offset.
								const float shiftedWeightDenominator = (shiftedPreviousPdf * shiftedPreviousPdf) * ((shiftedLumPdf * shiftedLumPdf) + (shiftedBsdfPdf * shiftedBsdfPdf));
								weight = mainWeightNumerator / (D_EPSILON + shiftedWeightDenominator + mainWeightDenominator);

								mainContribution = basePath.throughput * mainEmitterRadiance;
								shiftedContribution = shiftedPath.throughput * shiftedEmitterRadiance; // Note: Jacobian baked into .throughput.
							}
							else {
								// Not connected - apply either reconnection or half-vector duplication shift.

								//const BSDF* shiftedBSDF = shiftedPath.rRec.its.getBSDF(shifted.ray);
								CompositedBRDF shiftedBRDFs;
								if (shiftedPath.lastDG.material) {
									shiftedPath.lastDG.material->shade(shiftedPath.lastRay, shiftedPath.lastMedium, shiftedPath.lastDG, shiftedBRDFs);
								}

								// Update the vertex type of the offset path.
								const VertexType shiftedVertexType = getVertexType(shiftedPath, glossyShiftThreshold, shiftedBRDFs.type());

								if (mainVertexType == VERTEX_TYPE_DIFFUSE && mainNextVertexType == VERTEX_TYPE_DIFFUSE && shiftedVertexType == VERTEX_TYPE_DIFFUSE) {
									// Use reconnection shift.

									// Optimization: Skip the last raycast and BSDF evaluation for the offset path when it won't contribute and isn't needed anymore.
									if (!lastSegment || mainHitEmitter) {
										ReconnectionShiftResult shiftResult;
										bool environmentConnection = false;

										if (basePath.isValid()) {
											// This is an actual reconnection shift.
											shiftResult = reconnectShift(scene, state, basePath.lastRay.org, basePath.lastDG.P, shiftedPath.lastDG.P, basePath.lastDG.Ng, basePath.lastRay.time);
										}
										else {
											// This is a reconnection at infinity in environment direction.
											environmentConnection = true;
											shiftResult = environmentShift(scene, state, basePath.lastRay, shiftedPath.lastDG.P);
										}

										if (!shiftResult.success) {
											// Failed to construct the offset path.
											shiftedPath.alive = false;
											goto shift_failed;
										}

										const Vector3f incomingDirection = -shiftedPath.lastRay.dir;
										const Vector3f outgoingDirection = shiftResult.wo;

										//BSDFSamplingRecord bRec(shifted.rRec.its, shifted.rRec.its.toLocal(incomingDirection), shifted.rRec.its.toLocal(outgoingDirection), ERadiance);

										// Strict normals check.
										//if (m_config->m_strictNormals && dot(outgoingDirection, shifted.rRec.its.geoFrame.n) * Frame::cosTheta(bRec.wo) <= 0) {
										//	shifted.alive = false;
										//	goto shift_failed;
										//}

										// Evaluate the BRDF to the new direction.
										//Color shiftedBsdfValue = shiftedBRDFs.eval(outgoingDirection, shiftedPath.lastDG, incomingDirection, directLightingBRDFTypes);
										//const float shiftedBsdfPdf = shiftedBRDFs.pdf(outgoingDirection, shiftedPath.lastDG, incomingDirection, directLightingBRDFTypes);
										Color shiftedBsdfValue = shiftedBRDFs.eval(incomingDirection, shiftedPath.lastDG, outgoingDirection, directLightingBRDFTypes);
										const float shiftedBsdfPdf = shiftedBRDFs.pdf(incomingDirection, shiftedPath.lastDG, outgoingDirection, directLightingBRDFTypes);

										// Update throughput and pdf.
										shiftedPath.throughput *= shiftedBsdfValue * shiftResult.jacobian;
										shiftedPath.pdf *= shiftedBsdfPdf * shiftResult.jacobian;

										shiftedPath.connectionState = PATH_RECENTLY_CONNECTED;

										if (mainHitEmitter) {
											// Also the offset path hit the emitter, as visibility was checked at reconnectShift or environmentShift.

											// Evaluate radiance to this direction.
											Color shiftedEmitterRadiance = zero;
											float shiftedLumPdf = 0.f;

											if (basePath.isValid() && !basePath.ignoreVisibleLights && basePath.lastDG.light && !basePath.backFacing) {
												// Hit an object.
												shiftedEmitterRadiance = basePath.lastDG.light->Le(basePath.lastDG, -outgoingDirection);

												// Evaluate the light sampling PDF of the new segment.
												//DirectSamplingRecord shiftedDRec;
												//shiftedDRec.p = mainDRec.p;
												//shiftedDRec.n = mainDRec.n;
												//shiftedDRec.dist = (mainDRec.p - shifted.rRec.its.p).length();
												//shiftedDRec.d = (mainDRec.p - shifted.rRec.its.p) / shiftedDRec.dist;
												//shiftedDRec.ref = mainDRec.ref;
												//shiftedDRec.refN = shifted.rRec.its.shFrame.n;
												//shiftedDRec.object = mainDRec.object;

												//shiftedLumPdf = scene->pdfEmitterDirect(shiftedDRec);

												const float dist = length(basePath.lastDG.P - shiftedPath.lastDG.P);
												const Vector3f dir = (basePath.lastDG.P - shiftedPath.lastDG.P) / dist;
												shiftedLumPdf = basePath.lastDG.light->pdf(basePath.lastDG, dir);
											}
											else {
												// Hit the environment.
												shiftedEmitterRadiance = mainEmitterRadiance;
												shiftedLumPdf = mainLumPdf;
											}

											// Power heuristic between light sample from base, BSDF sample from base, light sample from offset, BSDF sample from offset.
											const float shiftedWeightDenominator = (shiftedPreviousPdf * shiftedPreviousPdf) * ((shiftedLumPdf * shiftedLumPdf) + (shiftedBsdfPdf * shiftedBsdfPdf));
											weight = mainWeightNumerator / (D_EPSILON + shiftedWeightDenominator + mainWeightDenominator);

											mainContribution = basePath.throughput * mainEmitterRadiance;
											shiftedContribution = shiftedPath.throughput * shiftedEmitterRadiance; // Note: Jacobian baked into .throughput.
										}
									}
								}
								else {
									// Use half-vector duplication shift. These paths could not have been sampled by light sampling (by our decision).
									//const Vector3f tangentSpaceIncomingDirection = shiftedPath.lastDG.shadingFrame.toLocal(-shiftedPath.lastRay.dir);
									//Vector3f tangentSpaceOutgoingDirection;
									const Vector3f incomingDirection = -shiftedPath.lastRay.dir;
									Vector3f outgoingDirection;
									Color shiftedEmitterRadiance = zero;
									float shiftedLumPdf = 0.;

									// Deny shifts between Dirac and non-Dirac BSDFs.
									const bool bothDelta = (baseBRDFs.type() & SPECULAR) && (shiftedBRDFs.type() & SPECULAR);
									const bool bothSmooth = (baseBRDFs.type() & SMOOTH) && (shiftedBRDFs.type() & SMOOTH);
									if (!(bothDelta || bothSmooth)) {
										shiftedPath.alive = false;
										goto half_vector_shift_failed;
									}

									//assert(fabs(lengthSquared(shiftedPath.lastRay.dir) - 1.f) < .000001f);

									// Apply the local shift.
									const int shiftedSampledIndex = shiftedBRDFs.has(baseSampledType);
									if (shiftedSampledIndex < 0) {
										// The shift is non-invertible so kill it.
										shiftedPath.alive = false;
										goto half_vector_shift_failed;
									}

									HalfVectorShiftResult shiftResult = halfVectorShift(-previousBasePath.lastRay.dir, baseBrdfSample, previousBasePath.lastDG, -shiftedPath.lastRay.dir, shiftedPath.lastDG, baseBRDFs[baseSampledIndex], shiftedBRDFs[shiftedSampledIndex]);

									if (baseBRDFs.type() & SPECULAR) {
										// Dirac delta integral is a point evaluation - no Jacobian determinant!
										shiftResult.jacobian = 1.f;
									}

									if (shiftResult.success) {
										// Invertible shift, success.
										shiftedPath.throughput *= shiftResult.jacobian;
										shiftedPath.pdf *= shiftResult.jacobian;
										outgoingDirection = shiftResult.wo;
									}
									else {
										// The shift is non-invertible so kill it.
										shiftedPath.alive = false;
										goto half_vector_shift_failed;
									}

									// Update throughput and pdf.
									//BSDFSamplingRecord bRec(shifted.rRec.its, tangentSpaceIncomingDirection, tangentSpaceOutgoingDirection, ERadiance);
									//EMeasure measure = (mainBsdfResult.bRec.sampledType & BSDF::EDelta) ? EDiscrete : ESolidAngle;

									//shifted.throughput *= shiftedBSDF->eval(bRec, measure);
									//shifted.pdf *= shiftedBSDF->pdf(bRec, measure);

									//Color shiftedBsdfValue = shiftedBRDFs.eval(outgoingDirection, shiftedPath.lastDG, incomingDirection, directLightingBRDFTypes);
									//const float shiftedBsdfPdf = shiftedBRDFs.pdf(outgoingDirection, shiftedPath.lastDG, incomingDirection, directLightingBRDFTypes);
									Color shiftedBsdfValue = shiftedBRDFs.eval(incomingDirection, shiftedPath.lastDG, outgoingDirection, shiftedBRDFs.type());
									const float shiftedBsdfPdf = shiftedBRDFs.pdf(incomingDirection, shiftedPath.lastDG, outgoingDirection, shiftedBRDFs.type());

									shiftedPath.throughput *= shiftedBsdfValue * rcp(shiftedBsdfPdf);
									shiftedPath.pdf *= shiftedBsdfPdf;

									///////////////////////////////////

									//Sample3f shiftedBrdfSample;
									//BRDFType shiftedSampledType;
									//const Vec2f s = state.sample->getVec2f(firstScatterSampleID + shiftedPath.depth);
									//const float ss = state.sample->getFloat(firstScatterTypeSampleID + shiftedPath.depth);
									//Color shiftedBsdfValue = shiftedBRDFs.sample(incomingDirection, basePath.lastDG, baseBrdfSample, type, s, ss, giBRDFTypes);

									// Since we pre-multiply brdf by rcp(pdf) in the sampling function above, we need to multiply it again to get the unweighted value
									//shiftedBsdfValue *= rcp(shiftedBsdfPdf);

									/*! Compute simple volumetric effect. */
									const Color& transmission = shiftedPath.lastMedium.transmission;
									if (transmission != Color(one)) {
										shiftedBsdfValue *= pow(transmission, shiftedPath.lastRay.tfar);
									}

									///*! Tracking medium if we hit a medium interface. */
									Medium shiftedNextMedium = shiftedPath.lastMedium;
									if (shiftedBRDFs.type() & TRANSMISSION)
										shiftedNextMedium = basePath.lastDG.material->nextMedium(basePath.lastMedium);
									//////////////////////////////////////
									if (shiftedPath.pdf == 0.f) {
										// Offset path is invalid!
										shiftedPath.alive = false;
										goto half_vector_shift_failed;
									}

									// Update the vertex type.
									//VertexType shiftedVertexType = getVertexType(shifted, *m_config, mainBsdfResult.bRec.sampledType);
									const VertexType shiftedVertexType = getVertexType(shiftedPath, glossyShiftThreshold, shiftedBRDFs.type());

									if (shiftedVertexType == VERTEX_TYPE_UNDEFINED) {
										// Offset path is invalid!
										shiftedPath.alive = false;
										goto half_vector_shift_failed;
									}

									// Trace the next hit point.
									Ray newRay(shiftedPath.lastDG.P, outgoingDirection, basePath.lastRay.time);
									shiftedPath.lastMedium = shiftedNextMedium;
									shiftedPath.ignoreVisibleLights = (shiftedBRDFs.type() & directLightingBRDFTypes) != NONE;
									shiftedPath.unbent = (shiftedPath.unbent && shiftedPath.lastRay.dir == newRay.dir);
									shiftedPath.lastRay = newRay;

									rtcIntersect(scene->scene, (RTCRay&)shiftedPath.lastRay);
									scene->postIntersect(shiftedPath.lastRay, shiftedPath.lastDG);
									state.numRays++;

									shiftedPath.depth++;

									/*! face forward normals */
									shiftedPath.backFacing = false;
									if (dot(shiftedPath.lastDG.Ng, shiftedPath.lastRay.dir) > 0) {
										shiftedPath.backFacing = true; shiftedPath.lastDG.Ng = -shiftedPath.lastDG.Ng; shiftedPath.lastDG.Ns = -shiftedPath.lastDG.Ns;
									}

									if (!shiftedPath.lastRay) {
										bool shiftedHitEmitter = false;
										// Hit nothing - Evaluate environment radiance.
										if (backplate && shiftedPath.unbent) {
											const int x = clamp(int(state.pixel.x * backplate->width), 0, int(backplate->width) - 1);
											const int y = clamp(int(state.pixel.y * backplate->height), 0, int(backplate->height) - 1);
											shiftedEmitterRadiance += backplate->get(x, y);
											shiftedLumPdf = 1.f;
											shiftedHitEmitter = true;
										}
										else {
											if (!shiftedPath.ignoreVisibleLights) {
												shiftedLumPdf = 1.f;
												for (size_t i = 0; i < scene->envLights.size(); i++) {
													shiftedEmitterRadiance += scene->envLights[i]->Le(-shiftedPath.lastRay.dir);
													shiftedLumPdf *= scene->envLights[i]->pdf(basePath.lastDG, -shiftedPath.lastRay.dir);
													shiftedHitEmitter = true;
												}
											}
										}

										if (!shiftedHitEmitter) {
											// Since base paths that hit nothing are not shifted, we must be symmetric and kill shifts that hit nothing.
											shiftedPath.alive = false;
											goto half_vector_shift_failed;
										}

										if (basePath.isValid()) {
											// Deny shifts between env and non-env.
											shiftedPath.alive = false;
											goto half_vector_shift_failed;
										}

										if (mainVertexType == VERTEX_TYPE_DIFFUSE && shiftedVertexType == VERTEX_TYPE_DIFFUSE) {
											// Environment reconnection shift would have been used for the reverse direction!
											shiftedPath.alive = false;
											goto half_vector_shift_failed;
										}

										// The offset path is no longer valid after this path segment.
										//shiftedEmitterRadiance = env->evalEnvironment(shifted.ray);
										postponedShiftEnd = true;
									}
									else {
										// Hit something.

										if (!basePath.isValid()) {
											// Deny shifts between env and non-env.
											shiftedPath.alive = false;
											goto half_vector_shift_failed;
										}

										//const VertexType shiftedNextVertexType = getVertexType(shifted, *m_config, mainBsdfResult.bRec.sampledType);
										const VertexType shiftedNextVertexType = getVertexType(shiftedPath, glossyShiftThreshold, shiftedBRDFs.type());

										// Make sure that the reverse shift would use this same strategy!
										// ==============================================================

										if (mainVertexType == VERTEX_TYPE_DIFFUSE && shiftedVertexType == VERTEX_TYPE_DIFFUSE && shiftedNextVertexType == VERTEX_TYPE_DIFFUSE) {
											// Non-invertible shift: the reverse-shift would use another strategy!
											shiftedPath.alive = false;
											goto half_vector_shift_failed;
										}

										if (!shiftedPath.ignoreVisibleLights && shiftedPath.lastDG.light && !shiftedPath.backFacing) {
											// Hit emitter.
											shiftedEmitterRadiance = shiftedPath.lastDG.light->Le(shiftedPath.lastDG, -shiftedPath.lastRay.dir);
											shiftedLumPdf = shiftedPath.lastDG.light->pdf(shiftedPath.lastDG, -shiftedPath.lastRay.dir);
										}
									}

								half_vector_shift_failed:
									if (shiftedPath.alive) {
										// Evaluate radiance difference using power heuristic between BSDF samples from base and offset paths.
										// Note: No MIS with light sampling since we don't use it for this connection type.
										weight = basePath.pdf / (shiftedPath.pdf * shiftedPath.pdf + basePath.pdf * basePath.pdf);
										//weight *= shiftedLumPdf / (shiftedLumPdf * shiftedLumPdf + shiftedBsdfPdf * shiftedBsdfPdf);
										mainContribution = basePath.throughput * mainEmitterRadiance;
										shiftedContribution = shiftedPath.throughput * shiftedEmitterRadiance; // Note: Jacobian baked into .throughput.
									}
									else {
#if 0
										// Handle the failure without taking MIS with light sampling, as we decided not to use it in the half-vector-duplication case.
										// Could have used it, but so far there has been no need. It doesn't seem to be very useful.
										weight = rcp(basePath.pdf);
#else
										// Lev: MIS with light sampling actually can make a lot of difference (depending on the scene), since it prevents specular rays from having too much weight
										// (i.e. making specular reflections too strong) and reduces overall hight frequency artifacts (i.e. fireflies)!
										weight = mainLumPdf / (mainLumPdf * mainLumPdf + baseBrdfSample.pdf * baseBrdfSample.pdf);
#endif
										mainContribution = basePath.throughput * mainEmitterRadiance;
										shiftedContribution = Color(0);

										// Disable the failure detection below since the failure was already handled.
										shiftedPath.alive = true;
										postponedShiftEnd = true;

										// (TODO: Restructure into smaller functions and get rid of the gotos... Although this may mean having lots of small functions with a large number of parameters.)
									}
								}
							}

						} //if (shiftedPath.alive)

					shift_failed:
						if (!shiftedPath.alive) {
							// The offset path cannot be generated; Set offset PDF and offset throughput to zero.
							weight = mainWeightNumerator / (D_EPSILON + mainWeightDenominator);
							mainContribution = basePath.throughput * mainEmitterRadiance;
							shiftedContribution = Color(zero);
						}

						// Note: Using also the offset paths for the throughput estimate, like we do here, provides some advantage when a large reconstruction alpha is used,
						// but using only throughputs of the base paths doesn't usually lose by much.
						if (basePath.depth + 1 >= minDepth) {
#ifndef CENTRAL_RADIANCE
							basePath.addRadiance(mainContribution, weight);
							//shiftedPath.addRadiance(shiftedContribution, weight);
#endif

							shiftedPath.addGradient(shiftedContribution - mainContribution, weight);
						}

						if (postponedShiftEnd) {
							shiftedPath.alive = false;
						}
					} //for (int j = 0; j < shiftedCount; ++j)

					// Stop if the base path hit the environment.
					//main.rRec.type = RadianceQueryRecord::ERadianceNoEmission;
					if (!basePath.isValid()/* || !(main.rRec.type & RadianceQueryRecord::EIndirectSurfaceRadiance)*/) {
						break;
					}
				} // Mistuba stuff
			} // Global illumination. Pick one BRDF component and sample it.

			// Increment the depth of the path
			++basePath.depth;
		} // while (basePath.depth < maxDepth)
	}

	void GPTIntegrator::EvaluatePoint(Ray &centralRay, Ray *offsetRays,
		Color &outVeryDirect, Color &outThroughput,
		Color *outGradients, Color *outShiftedThroughputs,
		const Ref<BackendScene> &scene, IntegratorState &state) {
		// Init the base path
		LightPath basePath(centralRay);

		// Init the offset paths
		LightPath offsetPaths[4] = {
			LightPath(offsetRays[0]),
			LightPath(offsetRays[1]),
			LightPath(offsetRays[2]),
			LightPath(offsetRays[3])
		};

		Color veryDirect = zero;
		evaluate(basePath, offsetPaths, 4, veryDirect, scene, state);

		// Accumulate the results.
		outVeryDirect += veryDirect;
		outThroughput += basePath.radiance;
		for (int i = 0; i < 4; ++i) {
			outGradients[i] += offsetPaths[i].gradient;
			outShiftedThroughputs[i] += offsetPaths[i].radiance;
		}
	}

}
