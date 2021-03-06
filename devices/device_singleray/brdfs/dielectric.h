// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#ifndef __EMBREE_DIELECTRIC_BRDF_H__
#define __EMBREE_DIELECTRIC_BRDF_H__

#include "../brdfs/brdf.h"
#include "../brdfs/optics.h"

namespace embree
{
	/*! BRDF of the reflection of a dielectricum. */
	class DielectricReflection : public BRDF
	{
	public:

		/*! Dielectric reflection BRDF constructor. \param etai is the
		 *  refraction index of the medium the incident ray travels in
		 *  \param etat is the refraction index of the opposite medium */
		__forceinline DielectricReflection(float etai, float etat, const float alpha = 1.f)
			: BRDF(SPECULAR_REFLECTION), eta_(etai*rcp(etat)), alpha(alpha) {}

		__forceinline Color eval(const Vector3f& wo, const DifferentialGeometry& dg, const Vector3f& wi) const {
			return zero;
		}

		Color sample(const Vector3f& wo, const DifferentialGeometry& dg, Sample3f& wi, const Vec2f& s) const {
			const float cosThetaO = clamp(dot(wo, dg.Ns));
			wi = reflect(wo, dg.Ns, cosThetaO);
			const auto c = alpha * Color(fresnelDielectric(cosThetaO, eta_));
			wi.eta = 1.f; 
			return c;
		}

		float pdf(const Vector3f& wo, const DifferentialGeometry& dg, const Vector3f& wi) const {
			return 0.f;
		}

		float roughness(const DifferentialGeometry &dg) const override {
			return 0.f;
		}

		float eta() const override {
			return eta_;
		}

	private:

		/*! relative refraction index etai/etat of both media */
		float eta_;
		float alpha;
	};

	/*! BRDF of the transmission of a dielectricum. */
	class DielectricTransmission : public BRDF
	{
	public:

		/*! Dielectric transmission BRDF constructor. \param etai is the
		 *  refraction index of the medium the incident ray travels in
		 *  \param etat is the refraction index of the opposite medium */
		__forceinline DielectricTransmission(float etai, float etat)
			: BRDF(SPECULAR_TRANSMISSION), eta_(etai*rcp(etat)) {}

		__forceinline Color eval(const Vector3f& wo, const DifferentialGeometry& dg, const Vector3f& wi) const {
			return zero;
		}

		Color sample(const Vector3f& wo, const DifferentialGeometry& dg, Sample3f& wi, const Vec2f& s) const {
			const float cosThetaO = clamp(dot(wo, dg.Ns));
			float cosThetaI;
			wi = refract(wo, dg.Ns, eta_, cosThetaO, cosThetaI);
			const Color c = Color(1.0f - fresnelDielectric(cosThetaO, cosThetaI, eta_));
			wi.eta = cosThetaI < 0.f ? eta_ : rcp(eta_);
			return c;
		}

		float pdf(const Vector3f& wo, const DifferentialGeometry& dg, const Vector3f& wi) const {
			return zero;
		}

		float roughness(const DifferentialGeometry &dg) const override {
			return 0.f;
		}

		float eta() const override {
			return eta_;
		}

	private:

		/*! relative refraction index etai/etat of both media */
		float eta_;
	};

	/*! BRDF of the transmission part of a thin dielectricum. Supports a
	 *  color of the dielectric medium. */
	class ThinDielectricTransmission : public BRDF
	{
	public:

		/*! Thin dielectric transmission BRDF constructor. \param etai is
		 *  the refraction index of the medium the incident ray travels in
		 *  \param etat is the refraction index of the opposite medium
		 *  \param T is volumetric transmission coefficient \param
		 *  thickness is the thickness of the medium */

		__forceinline ThinDielectricTransmission(float etai, float etat, const Color& T, float thickness)
			: BRDF(SPECULAR_TRANSMISSION), eta_(etai*rcp(etat)), logT(log(T.r), log(T.g), log(T.b)), thickness(thickness) {}

		__forceinline Color eval(const Vector3f& wo, const DifferentialGeometry& dg, const Vector3f& wi) const {
			return zero;
		}

		Color sample(const Vector3f& wo, const DifferentialGeometry& dg, Sample3f& wi, const Vec2f& s) const
		{
			wi = Sample3f(-wo, 1.0f);
			const float cosTheta = clamp(dot(wo, dg.Ns));
			if (cosTheta <= 0.0f) return zero;
			const float alpha = thickness * rcp(cosTheta);
			float cosThetaT;
			const Color c = exp(logT * alpha) * (1.f - fresnelDielectric(cosTheta, eta_, &cosThetaT));
			wi.eta = cosThetaT < 0.f ? eta_ : rcp(eta_);
			return c;
		}

		float pdf(const Vector3f& wo, const DifferentialGeometry& dg, const Vector3f& wi) const {
			return 0.f;
		}

		float roughness(const DifferentialGeometry &dg) const override {
			return 0.f;
		}

		float eta() const override {
			/* The relative IOR across this interface is 1, since the internal
			material is thin: it begins and ends here. */
			return 1;// eta_;
		}

	private:

		/*! Relative refraction index etai/etat of both media. */
		float eta_;

		/*! Logarithm of volumetric transmission coefficient. */
		Color logT;

		/*! Thickness of the medium. */
		float thickness;
	};

	/*! BRDF to support constant transmission expressed the alpha channel of a texture. */
	class ConstDielectricTransmission : public BRDF
	{
	public:

		/*! Thin dielectric transmission BRDF constructor. \param etai is
		*  the refraction index of the medium the incident ray travels in
		*  \param etat is the refraction index of the opposite medium
		*  \param T is volumetric transmission coefficient \param
		*  thickness is the thickness of the medium */

		__forceinline ConstDielectricTransmission(const float opacity)
			: BRDF(SPECULAR_TRANSMISSION), color(opacity) {}

		__forceinline Color eval(const Vector3f& wo, const DifferentialGeometry& dg, const Vector3f& wi) const {
			return zero;
		}

		Color sample(const Vector3f& wo, const DifferentialGeometry& dg, Sample3f& wi, const Vec2f& s) const {
			wi = Sample3f(-wo, 1.0f);
			const float cosTheta = clamp(dot(wo, dg.Ns));
			return cosTheta <= 0.0f ? zero : color;
		}

		float pdf(const Vector3f& wo, const DifferentialGeometry& dg, const Vector3f& wi) const {
			return 0.f;
		}

		float roughness(const DifferentialGeometry &dg) const override {
			return 0.f;
		}

		float eta() const override {
			return 1.f;
		}

	private:
		Color color;
	};
}

#endif
