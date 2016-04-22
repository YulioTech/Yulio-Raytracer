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

	// Non-recursive version
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
}
