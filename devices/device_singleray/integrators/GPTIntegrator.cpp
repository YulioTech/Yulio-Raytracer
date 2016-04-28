#include "GPTIntegrator.h"

#include "integrators/GPTIntegrator.h"

namespace embree
{
	/// A threshold to use in positive denominators to avoid division by zero.
	const float D_EPSILON = (float)(1e-14);

	GPTIntegrator::GPTIntegrator(const Parms& parms)
		: lightSampleID(-1), firstScatterSampleID(-1), firstScatterTypeSampleID(-1)
	{
		maxDepth = parms.getInt("maxDepth", 10);
		minContribution = parms.getFloat("minContribution", .02f);
		epsilon = parms.getFloat("epsilon", 32.f) * float(ulp);
		tMaxShadowRay = parms.getFloat("tMaxShadowRay", std::numeric_limits<float>::infinity());
		backplate = parms.getImage("backplate");
		shiftThreshold = parms.getFloat("shiftThreshold", .001f);
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

		while (true) {

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
					L += lightPath.misWeight * backplate->get(x, y);
				}
				else {
					if (!lightPath.ignoreVisibleLights)
						for (size_t i = 0; i < scene->envLights.size(); i++)
							L += lightPath.misWeight * scene->envLights[i]->Le(wo);
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
				L += lightPath.misWeight * dg.light->Le(dg, wo);

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

					/*! Test for shadows. */
					ls.tMax = tMaxShadowRay;
					Ray shadowRay(dg.P, ls.wi, dg.error*epsilon, ls.tMax - dg.error*epsilon, lightPath.lastRay.time, dg.shadowMask);
					//bool inShadow = scene->intersector->occluded(shadowRay);
					rtcOccluded(scene->scene, (RTCRay&)shadowRay);
					state.numRays++;
					if (shadowRay) continue;

					/*! Evaluate BRDF. */
					L += lightPath.misWeight * ls.L * brdf * rcp(ls.wi.pdf);
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

				/*! Terminate path if the contribution is too low. */
				if (reduce_max(lightPath.throughput * c) < minContribution)
					break;

				/*! Tracking medium if we hit a medium interface. */
				Medium nextMedium = lightPath.lastMedium;
				if (type & TRANSMISSION)
					nextMedium = dg.material->nextMedium(lightPath.lastMedium);

				/*! Continue the path. */
				lightPath = lightPath.extended(Ray(dg.P, wi, dg.error*epsilon, inf, lightPath.lastRay.time), nextMedium, c, rcp(wi.pdf), (type & directLightingBRDFTypes) != NONE);
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
	GPTIntegrator::VertexType GPTIntegrator::getVertexTypeByRoughness(float roughness, float shiftThreshold) const {
		if (roughness <= shiftThreshold) {
			return VERTEX_TYPE_GLOSSY;
		}
		else {
			return VERTEX_TYPE_DIFFUSE;
		}
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
	GPTIntegrator::VertexType GPTIntegrator::getVertexType(const CompositedBRDF &brdfs, const DifferentialGeometry &dg, float shiftThreshold, BRDFType bsdfType) const {
		// Return the lowest roughness value of the components of the vertex's BSDF.
		// If 'bsdfType' does not have a delta component, do not take perfect speculars (zero roughness) into account in this.

		float lowestRoughness = std::numeric_limits<float>::infinity();

		bool foundSmooth = false;
		bool foundDirac = false;
		for (int i = 0, componentCount = brdfs.size(); i < componentCount; ++i) {
			auto brdf = brdfs[i];
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

		return getVertexTypeByRoughness(lowestRoughness, shiftThreshold);
	}

	void GPTIntegrator::evaluate(LightPath &basePath, LightPath *shiftedPaths, int shiftedCount, Color &outVeryDirect, const Ref<BackendScene> &scene, IntegratorState &state)
	{
#if 0
		BRDFType directLightingBRDFTypes = (BRDFType)(DIFFUSE | GLOSSY);
		BRDFType giBRDFTypes = (BRDFType)(SPECULAR);
#else
		BRDFType directLightingBRDFTypes = (BRDFType)(DIFFUSE);
		BRDFType giBRDFTypes = (BRDFType)(ALL);
#endif

		/*! Traverse ray. */
		while (true) {

			// Perform the first ray intersection for the base path
			//scene->intersector->intersect(lightPath.lastRay);
			rtcIntersect(scene->scene, (RTCRay&)basePath.lastRay);
			state.numRays++;

			const Vector3f wo = -basePath.lastRay.dir;

			/*! Environment shading when nothing is hit. */
			if (!basePath.lastRay) {
				if (backplate && basePath.unbent) {
					const int x = clamp(int(state.pixel.x * backplate->width), 0, int(backplate->width) - 1);
					const int y = clamp(int(state.pixel.y * backplate->height), 0, int(backplate->height) - 1);
					outVeryDirect += basePath.misWeight * backplate->get(x, y);
				}
				else {
					if (!basePath.ignoreVisibleLights)
						for (size_t i = 0; i < scene->envLights.size(); i++)
							outVeryDirect += basePath.misWeight * scene->envLights[i]->Le(wo);
				}

				// First hit is not in the scene so can't continue. Also there are no paths to shift.
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
			basePath.backFacing = false;
			if (dot(basePath.lastDG.Ng, basePath.lastRay.dir) > 0) {
				basePath.backFacing = true; basePath.lastDG.Ng = -basePath.lastDG.Ng; basePath.lastDG.Ns = -basePath.lastDG.Ns;
			}

			for (int i = 0; i < shiftedCount; ++i) {
				LightPath &shiftedPath = shiftedPaths[i];
				shiftedPath.backFacing = false;
				if (dot(shiftedPath.lastDG.Ng, shiftedPath.lastRay.dir) > 0) {
					shiftedPath.backFacing = true; shiftedPath.lastDG.Ng = -shiftedPath.lastDG.Ng; shiftedPath.lastDG.Ns = -shiftedPath.lastDG.Ns;
				}
			}

			/*! Shade surface. */
			CompositedBRDF baseBRDFs;
			if (basePath.lastDG.material)
				basePath.lastDG.material->shade(basePath.lastRay, basePath.lastMedium, basePath.lastDG, baseBRDFs);

#if 0
			/*! Add light emitted by hit area light source. */
			if (!basePath.ignoreVisibleLights && basePath.lastDG.light && !basePath.backFacing)
				outVeryDirect += basePath.misWeight * basePath.lastDG.light->Le(basePath.lastDG, wo);
#endif

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
						ls.L = scene->allLights[i]->sample(basePath.lastDG, ls, state.sample->getVec2f(lightSampleID));

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
					ls.tMax = tMaxShadowRay;
					Ray shadowRay(basePath.lastDG.P, ls.wi, basePath.lastDG.error*epsilon, ls.tMax - basePath.lastDG.error*epsilon, basePath.lastRay.time, basePath.lastDG.shadowMask);
					//bool inShadow = scene->intersector->occluded(shadowRay);
					rtcOccluded(scene->scene, (RTCRay&)shadowRay);
					state.numRays++;
					if (shadowRay) continue;

					/*! Evaluate BRDF. */
					outVeryDirect += basePath.misWeight * ls.L * brdf * rcp(ls.wi.pdf);

					// Stuff from Mitsuba "translated" into our framework
					{
						Color mainEmitterRadiance = ls.L;
						bool mainEmitterVisible = shadowRay;
						Color mainBSDFValue = brdf;
						// Calculate the probability density of having generated the sampled path segment by BSDF sampling. Note that if the emitter is not visible, the probability density is zero.
						// Even if the BSDF sampler has zero probability density, the light sampler can still sample it.
						float mainBsdfPdf = mainEmitterVisible ? baseBRDFs.pdf(wo, basePath.lastDG, ls.wi, directLightingBRDFTypes) : 0.f;

						// There values are probably needed soon for the Jacobians.
						float mainDistanceSquared = lengthSquared(basePath.lastDG.P - ls.p);
						float mainOpposingCosine = dot(ls.n, (basePath.lastDG.P - ls.p)) / sqrt(mainDistanceSquared);

						// Power heuristic weights for the following strategies: light sample from base, BSDF sample from base.
						float mainWeightNumerator = basePath.pdf * ls.wi.pdf;
						float mainWeightDenominator = (basePath.pdf * basePath.pdf) * ((ls.wi.pdf * ls.wi.pdf) + (mainBsdfPdf * mainBsdfPdf));

						// The base path is good. Add radiance differences to offset paths.
						for (int i = 0; i < shiftedCount; ++i) {
							// Evaluate and apply the gradient.
							LightPath &shiftedPath = shiftedPaths[i];

							Color  mainContribution = zero;
							Color shiftedContribution = zero;
							float weight = 0.f;

							bool shiftSuccessful = shiftedPath.alive;

							// Construct the offset path.
							if (shiftSuccessful) {
								// Generate the offset path.
								if (shiftedPath.connectionState == PATH_CONNECTED) {
									// Follow the base path. All relevant vertices are shared. 
									float shiftedBsdfPdf = mainBsdfPdf;
									float shiftedDRecPdf = ls.wi.pdf;
									Color shiftedBsdfValue = mainBSDFValue;
									Color shiftedEmitterRadiance = mainEmitterRadiance;
									float jacobian = 1.f;

									// Power heuristic between light sample from base, BSDF sample from base, light sample from offset, BSDF sample from offset.
									float shiftedWeightDenominator = (jacobian * shiftedPath.pdf) * (jacobian * shiftedPath.pdf) * ((shiftedDRecPdf * shiftedDRecPdf) + (shiftedBsdfPdf * shiftedBsdfPdf));
									weight = mainWeightNumerator / (D_EPSILON + shiftedWeightDenominator + mainWeightDenominator);

									mainContribution = basePath.throughput * (mainBSDFValue * mainEmitterRadiance);
									shiftedContribution = jacobian * shiftedPath.throughput * (shiftedBsdfValue * shiftedEmitterRadiance);

									// Note: The Jacobians were baked into shifted.pdf and shifted.throughput at connection phase.
								}
								else if (shiftedPath.connectionState == PATH_RECENTLY_CONNECTED) {
									// Follow the base path. The current vertex is shared, but the incoming directions differ.
									Vector3f incomingDirection = normalize(shiftedPath.lastDG.P - basePath.lastDG.P);

									//BSDFSamplingRecord bRec(main.rRec.its, main.rRec.its.toLocal(incomingDirection), main.rRec.its.toLocal(dRec.d), ERadiance);

									// Sample the BSDF.
									//Float shiftedBsdfPdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle && mainEmitterVisible) ? mainBSDF->pdf(bRec) : 0; // The BSDF sampler can not sample occluded path segments.
									float shiftedBsdfPdf = mainEmitterVisible ? baseBRDFs.pdf(wo, basePath.lastDG, incomingDirection, directLightingBRDFTypes) : 0.f;
									float shiftedDRecPdf = ls.wi.pdf;
									Color shiftedBsdfValue = baseBRDFs.eval(wo, basePath.lastDG, incomingDirection, directLightingBRDFTypes);
									Color shiftedEmitterRadiance = mainEmitterRadiance;
									float jacobian = 1.f;

									// Power heuristic between light sample from base, BSDF sample from base, light sample from offset, BSDF sample from offset.
									float shiftedWeightDenominator = (jacobian * shiftedPath.pdf) * (jacobian * shiftedPath.pdf) * ((shiftedDRecPdf * shiftedDRecPdf) + (shiftedBsdfPdf * shiftedBsdfPdf));
									weight = mainWeightNumerator / (D_EPSILON + shiftedWeightDenominator + mainWeightDenominator);

									mainContribution = basePath.throughput * (mainBSDFValue * mainEmitterRadiance);
									shiftedContribution = jacobian * basePath.throughput * (shiftedBsdfValue * shiftedEmitterRadiance);

									// Note: The Jacobians were baked into shifted.pdf and shifted.throughput at connection phase.
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

									const auto brdfTypeToTest = (BRDFType)(DIFFUSE | GLOSSY);
									const VertexType mainVertexType = getVertexType(baseBRDFs, basePath.lastDG, shiftThreshold, brdfTypeToTest);
									const VertexType shiftedVertexType = getVertexType(shiftedBRDFs, shiftedPath.lastDG, shiftThreshold, brdfTypeToTest);

									if (mainAtPointLight || (mainVertexType == VERTEX_TYPE_DIFFUSE && shiftedVertexType == VERTEX_TYPE_DIFFUSE)) {
										/*
										// Get emitter radiance.
										DirectSamplingRecord shiftedDRec(shifted.rRec.its);
										std::pair<Spectrum, bool> emitterTuple = m_scene->sampleEmitterDirectVisible(shiftedDRec, lightSample);
										bool shiftedEmitterVisible = emitterTuple.second;

										Spectrum shiftedEmitterRadiance = emitterTuple.first * shiftedDRec.pdf;
										Float shiftedDRecPdf = shiftedDRec.pdf;

										// Sample the BSDF.
										Float shiftedDistanceSquared = (dRec.p - shifted.rRec.its.p).lengthSquared();
										Vector emitterDirection = (dRec.p - shifted.rRec.its.p) / sqrt(shiftedDistanceSquared);
										Float shiftedOpposingCosine = -dot(dRec.n, emitterDirection);

										BSDFSamplingRecord bRec(shifted.rRec.its, shifted.rRec.its.toLocal(emitterDirection), ERadiance);

										// Strict normals check, to make the output match with bidirectional methods when normal maps are present.
										if (m_config->m_strictNormals && dot(shifted.rRec.its.geoFrame.n, emitterDirection) * Frame::cosTheta(bRec.wo) < 0) {
											// Invalid, non-sampleable offset path.
											shiftSuccessful = false;
										}
										else {
											Spectrum shiftedBsdfValue = shiftedBSDF->eval(bRec);
											Float shiftedBsdfPdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle && shiftedEmitterVisible) ? shiftedBSDF->pdf(bRec) : 0;
											Float jacobian = std::abs(shiftedOpposingCosine * mainDistanceSquared) / (Epsilon + std::abs(mainOpposingCosine * shiftedDistanceSquared));

											// Power heuristic between light sample from base, BSDF sample from base, light sample from offset, BSDF sample from offset.
											Float shiftedWeightDenominator = (jacobian * shifted.pdf) * (jacobian * shifted.pdf) * ((shiftedDRecPdf * shiftedDRecPdf) + (shiftedBsdfPdf * shiftedBsdfPdf));
											weight = mainWeightNumerator / (D_EPSILON + shiftedWeightDenominator + mainWeightDenominator);

											mainContribution = main.throughput * (mainBSDFValue * mainEmitterRadiance);
											shiftedContribution = jacobian * shifted.throughput * (shiftedBsdfValue * shiftedEmitterRadiance);
										}
										*/
									}
								}
							} // if (shiftSuccessful) 

							if (!shiftSuccessful) {
								// The offset path cannot be generated; Set offset PDF and offset throughput to zero. This is what remains.

								// Power heuristic between light sample from base, BSDF sample from base, light sample from offset, BSDF sample from offset. (Offset path has zero PDF)
								float shiftedWeightDenominator = 0.f;
								weight = mainWeightNumerator / (D_EPSILON + mainWeightDenominator);

								mainContribution = basePath.throughput * (mainBSDFValue * mainEmitterRadiance);
								shiftedContribution = Color(0.f);
							}

							// Note: Using also the offset paths for the throughput estimate, like we do here, provides some advantage when a large reconstruction alpha is used,
							// but using only throughputs of the base paths doesn't usually lose by much.
							basePath.addRadiance(mainContribution, weight);
							shiftedPath.addRadiance(shiftedContribution, weight);

							shiftedPath.addGradient(shiftedContribution - mainContribution, weight);

						} // for (int i = 0; i < shiftedCount; ++i)
					} // Stuff from Mitsuba "translated" into our framework
				} // for (size_t i = 0; i < scene->allLights.size(); i++)
			} // if (useDirectLighting)

			// Stop if the next ray is going to exceed the max allowed depth
			if (basePath.depth >= maxDepth - 1) break;

			/*! Global illumination. Pick one BRDF component and sample it. */
			{
				/*! sample brdf */
				Sample3f sample; BRDFType type;
				const Vec2f s = state.sample->getVec2f(firstScatterSampleID + basePath.depth);
				const float ss = state.sample->getFloat(firstScatterTypeSampleID + basePath.depth);
				Color brdfWeight = baseBRDFs.sample(wo, basePath.lastDG, sample, type, s, ss, giBRDFTypes);

				/*! Continue only if we hit something valid. */
				// If PDF <= 0, impossible base path.
				if (brdfWeight == Color(zero) || sample.pdf <= 0.0f) break;

				/*! Compute simple volumetric effect. */
				const Color& transmission = basePath.lastMedium.transmission;
				if (transmission != Color(one)) {
					brdfWeight *= pow(transmission, basePath.lastRay.tfar);
				}

				/*! Terminate path if the contribution is too low. */
				if (reduce_max(basePath.throughput * brdfWeight) < minContribution)
					break;

				/*! Tracking medium if we hit a medium interface. */
				Medium nextMedium = basePath.lastMedium;
				if (type & TRANSMISSION)
					nextMedium = basePath.lastDG.material->nextMedium(basePath.lastMedium);

				/*! Continue the path. */
				basePath = basePath.extended(Ray(basePath.lastDG.P, sample, basePath.lastDG.error*epsilon, inf, basePath.lastRay.time), nextMedium, brdfWeight, rcp(sample.pdf), (type & directLightingBRDFTypes) != NONE);

				// Stuff from Mitsuba "translated" into our framework
				{
					 basePath.lastDG;
					// The old intersection structure is still needed after main.rRec.its gets updated.
					 const DifferentialGeometry previousBaseDG = basePath.lastDG;

					// Trace a ray in the sampled direction.
					bool mainHitEmitter = false;
					Color mainEmitterRadiance = zero;

					// Update the vertex types.
					//const VertexType mainVertexType = getVertexType(baseBRDFs, basePath.lastDG, shiftThreshold, brdfTypeToTest);
					//VertexType mainNextVertexType;

					//main.ray = Ray(main.rRec.its.p, mainWo, main.ray.time);

				}
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
