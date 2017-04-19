#pragma once

#include "../materials/material.h"
#include "../brdfs/lambertian.h"
#include "../brdfs/dielectric.h"
#include "../brdfs/dielectriclayer.h"
#include "../brdfs/microfacet.h"
#include "../textures/texture.h"

namespace embree
{
	/*! Implements a universal diffuse and textured material with the alpha support.*/
	class Uber : public Material
	{
		using MicrofacetUber = Microfacet<FresnelDielectric, PowerCosineDistribution>;

	public:

		/*! Construction from parameters. */
		Uber(const Parms& parms)
		{
			Kd = parms.getTexture("Kd");
			diffuse = parms.getColor("diffuse", zero);
			s0 = parms.getVec2f("s0", Vec2f(0.0f, 0.0f));
			ds = parms.getVec2f("ds", Vec2f(1.0f, 1.0f));
			eta = parms.getFloat("eta", 1.4f);
			roughness = parms.getFloat("roughness", .9f);
			reflectivity = parms.getFloat("reflectivity", .0f);
			rcpRoughness = rcp(roughness);
		}

		~Uber() {}

		void shade(const Ray& ray, const Medium& currentMedium, const DifferentialGeometry& dg, CompositedBRDF& brdfs) const {

			Color4 diffuseColor(diffuse.r, diffuse.g, diffuse.b, 1.f);
			float alpha = 1.f, opacity = 0.f;
			if (Kd) {
				diffuseColor = Kd->get(ds*dg.st + s0);
				alpha = diffuseColor.a;
				opacity = 1.f - alpha;
			}

			// Lev: use the vanilla Lambertian BRDF to make sure the weights (that are based on BRDF sampling PDFs) are properly calculated.
			brdfs.add(NEW_BRDF(Lambertian)(diffuseColor * alpha));
			/*! the dielectric layer that models the covered diffuse part */
			//brdfs.add(NEW_BRDF(DielectricLayer<Lambertian>)(one, 1.0f, eta, Lambertian(diffuseColor * alpha)));

			if (alpha < 1.f) {
				//brdfs.add(NEW_BRDF(ThinDielectricTransmission)(1.f, 1.f, Color4(1.f - diffuseColor.a), 1.f));
				brdfs.add(NEW_BRDF(ConstDielectricTransmission)(opacity));
			}

			/*! use dielectric modulated reflection in case of an explicitly specified reflectivity */
			if (reflectivity > 0.f) {
				brdfs.add(NEW_BRDF(DielectricReflection)(1.f, eta, alpha * reflectivity));
			}
			/*! use dielectric reflection in case of a specular surface */
			else if (roughness == 0.f) {
				brdfs.add(NEW_BRDF(DielectricReflection)(1.f, eta, alpha));
			}
			// Lev: microfacets don't play nice with the gradient-based PT under the current implementation, so, ideally, it should be disabled whenm GPT is used.
#if 1
			/*! use the microfacet BRDF to model the rough surface */
			else {
				brdfs.add(NEW_BRDF(MicrofacetUber)(Color(alpha), FresnelDielectric(1.f, eta), PowerCosineDistribution(rcpRoughness, dg.Ns)));
			}
#endif
		}

	protected:
		Vec2f s0;         //!< Offset for texture coordinates.
		Vec2f ds;         //!< Scaling for texture coordinates.
		Ref<Texture> Kd;  //!< Diffuse texture with an optional alpha channel to the surface.
		Color diffuse;  //! Diffuse reflectance of the surface. The range is from 0 (black) to 1 (white).
		float eta;          //!< Refraction index of the dielectric layer.
		float roughness;    //!< Roughness parameter. The range goes from 0 (specular) to 1 (diffuse).
		float reflectivity;    //!< Reflectivity (i.e. how much of a "mirror" effect is added).
		float rcpRoughness; //!< Reciprocal roughness parameter.
	};
}