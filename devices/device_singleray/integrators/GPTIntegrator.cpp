#include "GPTIntegrator.h"



#include "integrators/GPTIntegrator.h"

namespace embree
{
	GPTIntegrator::GPTIntegrator(const Parms& parms)
		: lightSampleID(-1), firstScatterSampleID(-1), firstScatterTypeSampleID(-1)
	{
		maxDepth = parms.getInt("maxDepth", 10);
		minContribution = parms.getFloat("minContribution", .01f);
		epsilon = parms.getFloat("epsilon", 32.f) * float(ulp);
		tMaxShadowRay = parms.getFloat("tMaxShadowRay", std::numeric_limits<float>::infinity());
		backplate = parms.getImage("backplate");
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

	// Non-recursive standard version
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

		while (true) {

			/*! Terminate path if too long or contribution too low. */
			if (reduce_max(lightPath.throughput) < minContribution)
				break;

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
			if (dot(dg.Ng, lightPath.lastRay.dir) > 0) {
				backfacing = true; dg.Ng = -dg.Ng; dg.Ns = -dg.Ns;
			}

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
						ls.L = scene->allLights[i]->sample(dg, ls.wi, ls.tMax, state.sample->getVec2f(lightSampleID));

					/*! Ignore zero radiance or illumination from the back. */
					//if (ls.L == Color(zero) || ls.wi.pdf == 0.0f || dot(dg.Ns,Vector3f(ls.wi)) <= 0.0f) continue; 
					if (ls.L == Color(zero) || ls.wi.pdf == 0.0f) continue;

					/*! Evaluate BRDF */
					Color brdf = brdfs.eval(wo, dg, ls.wi, directLightingBRDFTypes);
					if (brdf == Color(zero)) continue;

					/*! Test for shadows. */
					{
						ls.tMax = tMaxShadowRay;
						Ray shadowRay(dg.P, ls.wi, dg.error*epsilon, ls.tMax - dg.error*epsilon, lightPath.lastRay.time, dg.shadowMask);
						//bool inShadow = scene->intersector->occluded(shadowRay);
						rtcOccluded(scene->scene, (RTCRay&)shadowRay);
						state.numRays++;
						if (shadowRay) continue;
					}

					/*! Evaluate BRDF. */
					L += lightPath.throughput * ls.L * brdf * rcp(ls.wi.pdf);
				}
			}

			// Stop if the next ray is going to exceed the max allowed depth
			if (lightPath.depth >= maxDepth - 1) break;

			/*! Global illumination. Pick one BRDF component and sample it. */
			{
				/*! sample brdf */
				Sample3f wi; BRDFType type;
				const Vec2f s = state.sample->getVec2f(firstScatterSampleID + lightPath.depth);
				const float ss = state.sample->getFloat(firstScatterTypeSampleID + lightPath.depth);
				Color c = brdfs.sample(wo, dg, wi, type, s, ss, giBRDFTypes);

				/*! Continue only if we hit something valid. */
				if (c == Color(zero) || wi.pdf <= 0.0f) break;

				/*! Compute simple volumetric effect. */
				const Color& transmission = lightPath.lastMedium.transmission;
				if (transmission != Color(one)) {
					c *= pow(transmission, lightPath.lastRay.tfar);
				}

				/*! Tracking medium if we hit a medium interface. */
				Medium nextMedium = lightPath.lastMedium;
				if (type & TRANSMISSION)
					nextMedium = dg.material->nextMedium(lightPath.lastMedium);

				/*! Continue the path. */
				const auto weight = c * rcp(wi.pdf);
				lightPath = lightPath.extended(Ray(dg.P, wi, dg.error*epsilon, inf, lightPath.lastRay.time), nextMedium, weight, (type & directLightingBRDFTypes) != NONE);
			}
		} // while (lightPath.depth < maxDepth)

		return L;
	}

	Color GPTIntegrator::Li(Ray& ray, const Ref<BackendScene>& scene, IntegratorState& state) {
		LightPath path(ray);
		return Li(path, scene, state);
	}


	////////////////////////////////////////////////////////////////////////////////////////////////////////

	void GPTIntegrator::evaluate(LightPath &basePath, LightPath *shiftedPaths, int shiftedCount, Color &outVeryDirect, const Ref<BackendScene>& scene, IntegratorState& state)
	{
#if 0
		BRDFType directLightingBRDFTypes = (BRDFType)(DIFFUSE | GLOSSY);
		BRDFType giBRDFTypes = (BRDFType)(SPECULAR);
#else
		BRDFType directLightingBRDFTypes = (BRDFType)(DIFFUSE);
		BRDFType giBRDFTypes = (BRDFType)(ALL);
#endif

		while (true) {

			/*! Terminate path if too long or contribution too low. */
			if (reduce_max(basePath.throughput) < minContribution)
				break;

			/*! Traverse ray. */
			//scene->intersector->intersect(lightPath.lastRay);
			rtcIntersect(scene->scene, (RTCRay&)basePath.lastRay);
			state.numRays++;

			const Vector3f wo = -basePath.lastRay.dir;

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
							outVeryDirect += basePath.throughput * scene->envLights[i]->Le(wo);
				}

				break;
			}

			// Perform the same first ray intersection for the offset paths.
			for (int i = 0; i < shiftedCount; ++i) {
				LightPath &shiftedPath = shiftedPaths[i];
				rtcIntersect(scene->scene, (RTCRay&)shiftedPath.lastRay);
				state.numRays++;

				// If no intersection of an offset ray could be found, its offset paths can not be generated.
				if (!shiftedPath.lastRay) shiftedPath.alive = false;
			}


			// Get the hit differential geometry.
			scene->postIntersect(basePath.lastRay, basePath.lastDG);
			for (int i = 0; i < shiftedCount; ++i) {
				LightPath &shiftedPath = shiftedPaths[i];
				scene->postIntersect(shiftedPath.lastRay, shiftedPath.lastDG);
			}

			// Some optimizations can be made if this is the last traced segment.
			const bool lastSegment = (basePath.depth + 1 == maxDepth);

			/*! face forward normals */
			bool backfacing = false;
			if (dot(basePath.lastDG.Ng, basePath.lastRay.dir) > 0) {
				backfacing = true; basePath.lastDG.Ng = -basePath.lastDG.Ng; basePath.lastDG.Ns = -basePath.lastDG.Ns;
			}

			/*! Shade surface. */
			CompositedBRDF baseBRDFs;
			if (basePath.lastDG.material)
				basePath.lastDG.material->shade(basePath.lastRay, basePath.lastMedium, basePath.lastDG, baseBRDFs);


			/*! Add light emitted by hit area light source. */
			if (!basePath.ignoreVisibleLights && basePath.lastDG.light && !backfacing)
				outVeryDirect += basePath.throughput * basePath.lastDG.light->Le(basePath.lastDG, wo);

			/*! Check if any BRDF component uses direct lighting. */
			bool useDirectLighting = false;
			for (size_t i = 0; i < baseBRDFs.size(); i++)
				useDirectLighting |= (baseBRDFs[i]->type & directLightingBRDFTypes) != NONE;

			/*! Direct lighting. Shoot shadow rays to all light sources. */
			if (useDirectLighting) {
				for (size_t i = 0; i < scene->allLights.size(); i++) {
					if ((scene->allLights[i]->illumMask & basePath.lastDG.illumMask) == 0)
						continue;

					/*! Either use precomputed samples for the light or sample light now. */
					LightSample ls;
					/*
					if (scene->allLights[i]->precompute())
						ls = state.sample->getLightSample(precomputedLightSampleID[i]);
					else
					*/
						ls.L = scene->allLights[i]->sample(basePath.lastDG, ls.wi, ls.tMax, state.sample->getVec2f(lightSampleID));

					// There values are probably needed soon for the Jacobians.
					//float mainDistanceSquared = (basePath.lastDG.P - dRec.p).lengthSquared();
					//Float mainOpposingCosine = dot(dRec.n, (main.rRec.its.p - dRec.p)) / sqrt(mainDistanceSquared);

					/*! Ignore zero radiance or illumination from the back. */
					//if (ls.L == Color(zero) || ls.wi.pdf == 0.0f || dot(dg.Ns,Vector3f(ls.wi)) <= 0.0f) continue; 
					if (ls.L == Color(zero) || ls.wi.pdf == 0.0f) continue;

					/*! Evaluate BRDF */
					Color brdf = baseBRDFs.eval(wo, basePath.lastDG, ls.wi, directLightingBRDFTypes);
					if (brdf == Color(zero)) continue;

					/*! Test for shadows. */
					{
						ls.tMax = tMaxShadowRay;
						Ray shadowRay(basePath.lastDG.P, ls.wi, basePath.lastDG.error*epsilon, ls.tMax - basePath.lastDG.error*epsilon, basePath.lastRay.time, basePath.lastDG.shadowMask);
						//bool inShadow = scene->intersector->occluded(shadowRay);
						rtcOccluded(scene->scene, (RTCRay&)shadowRay);
						state.numRays++;
						if (shadowRay) continue;
					}

					/*! Evaluate BRDF. */
					outVeryDirect += basePath.throughput * ls.L * brdf * rcp(ls.wi.pdf);
				}
			}

			// Stop if the next ray is going to exceed the max allowed depth
			if (basePath.depth >= maxDepth - 1) break;

			/*! Global illumination. Pick one BRDF component and sample it. */
			{
				/*! sample brdf */
				Sample3f wi; BRDFType type;
				const Vec2f s = state.sample->getVec2f(firstScatterSampleID + basePath.depth);
				const float ss = state.sample->getFloat(firstScatterTypeSampleID + basePath.depth);
				Color c = baseBRDFs.sample(wo, basePath.lastDG, wi, type, s, ss, giBRDFTypes);

				/*! Continue only if we hit something valid. */
				if (c == Color(zero) || wi.pdf <= 0.0f) break;

				/*! Compute simple volumetric effect. */
				const Color& transmission = basePath.lastMedium.transmission;
				if (transmission != Color(one)) {
					c *= pow(transmission, basePath.lastRay.tfar);
				}

				/*! Tracking medium if we hit a medium interface. */
				Medium nextMedium = basePath.lastMedium;
				if (type & TRANSMISSION)
					nextMedium = basePath.lastDG.material->nextMedium(basePath.lastMedium);

				/*! Continue the path. */
				const auto weight = c * rcp(wi.pdf);
				basePath = basePath.extended(Ray(basePath.lastDG.P, wi, basePath.lastDG.error*epsilon, inf, basePath.lastRay.time), nextMedium, weight, (type & directLightingBRDFTypes) != NONE);
			}
		} // while (lightPath.depth < maxDepth)
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

		evaluate(basePath, offsetPaths, 4, outVeryDirect, scene, state);
	}

}
