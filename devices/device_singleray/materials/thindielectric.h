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

#ifndef __EMBREE_THIN_DIELECTRIC_H__
#define __EMBREE_THIN_DIELECTRIC_H__

#include "../materials/material.h"
#include "../brdfs/dielectric.h"

namespace embree
{
	/*! Implements a thin dielectricum material. The model uses a
	 *  dielectric reflection BRDF and thin dielectric transmission
	 *  BRDF. */
	class ThinDielectric : public Material
	{
	public:

		/*! Construction from parameters. */
		ThinDielectric(const Parms& parms) {
			Kd = parms.getTexture("Kd");
			s0 = parms.getVec2f("s0", Vec2f(0.0f, 0.0f));
			ds = parms.getVec2f("ds", Vec2f(1.0f, 1.0f));
			transmission = parms.getColor("transmission", one);
			eta = parms.getFloat("eta", 1.4f);
			thickness = parms.getFloat("thickness", 0.1f);
		}

		void shade(const Ray& ray, const Medium& currentMedium, const DifferentialGeometry& dg, CompositedBRDF& brdfs) const {
			brdfs.add(NEW_BRDF(DielectricReflection)(1.0f, eta));

			Color4 diffuseColor(transmission.r, transmission.g, transmission.b, 1.f);
			if (Kd) {
				diffuseColor = Kd->get(ds*dg.st + s0);
				//diffuseColor = Color4(1.f - diffuseColor.r, 1.f - diffuseColor.g, 1.f - diffuseColor.b, 1.f - diffuseColor.a);
			}

			brdfs.add(NEW_BRDF(ThinDielectricTransmission)(1.0f, eta, diffuseColor, thickness));
		}

	protected:
		Vec2f s0;         //!< Offset for texture coordinates.
		Vec2f ds;         //!< Scaling for texture coordinates.
		Ref<Texture> Kd;  //!< Diffuse texture with an optional alpha channel to the surface.
		Color transmission;   //!< Transmission coefficient of material.
		float eta;            //!< Refraction index of material.
		float thickness;      //!< Thickness of material layer.
	};
}

#endif
