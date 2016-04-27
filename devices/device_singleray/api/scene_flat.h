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

#ifndef __EMBREE_BACKEND_SCENE_FLAT_H__
#define __EMBREE_BACKEND_SCENE_FLAT_H__

#include "scene.h"
#include <embree2/rtcore.h>

namespace embree
{
	/*! Flat scene, no support for instancing, best render performance. */
	class BackendSceneFlat : public BackendScene
	{
	public:

		struct Primitive : public RefCount
		{
		public:
			Primitive(const Ref<Shape>& shape,
				const Ref<Light>& light,
				const Ref<Material>& material,
				const light_mask_t illumMask,
				const light_mask_t shadowMask)
				: shape(shape), light(light), material(material),
				illumMask(illumMask), shadowMask(shadowMask) {}

			__forceinline void postIntersect(const Ray& ray, DifferentialGeometry& dg) const
			{
				dg.material = material.ptr;
				dg.light = (AreaLight*)light.ptr;
				dg.illumMask = illumMask;
				dg.shadowMask = shadowMask;
				shape->postIntersect(ray, dg);
			}

		public:
			Ref<Shape> shape;
			Ref<Light> light;
			Ref<Material> material;
			light_mask_t illumMask;  /*! which light masks we receive illum from */
			light_mask_t shadowMask; /*! which light masks we cast shadows to */
		};

		/*! API handle that manages user actions. */
		class Handle : public BackendScene::Handle {
			ALIGNED_CLASS;
		public:

			void setPrimitive(size_t slot, Ref<PrimitiveHandle> prim)
			{
				if (prim->faceCamera) dynamicScene = true;
				if (slot >= prims.size()) prims.resize(slot + 1);
				Ref<Shape> shape = prim->getShapeInstance();
				Ref<Light> light = prim->getLightInstance();
				if (light) shape = light->shape();
				if (shape) shape = shape->transform(prim->transform);
				if (light) light = light->transform(prim->transform, prim->illumMask, prim->shadowMask);
				prims[slot] = new Primitive(shape, light, prim->getMaterialInstance(), prim->illumMask, prim->shadowMask);
			}

			void updatePrimitive(size_t slot, Ref<PrimitiveHandle> prim, const AffineSpace3f &transform)
			{
				if (slot > prims.size()) return;

				Ref<Shape> shape = prim->getShapeInstance();
				Ref<Light> light = prim->getLightInstance();
				if (light) shape = light->shape();
				if (shape) shape = shape->transform(transform);
				if (light) light = light->transform(transform, prim->illumMask, prim->shadowMask);
				prims[slot] = new Primitive(shape, light, prim->getMaterialInstance(), prim->illumMask, prim->shadowMask);
			}

			void create()
			{
				// Create the scene
				RTCScene scene = rtcNewScene(
					RTC_SCENE_STATIC//(dynamicScene ? RTC_SCENE_DYNAMIC : RTC_SCENE_STATIC)
					| RTC_SCENE_INCOHERENT
					| RTC_SCENE_HIGH_QUALITY
					| RTC_SCENE_ROBUST
					, RTC_INTERSECT1
					);

				// Extract the geometry
				size_t primIdx = 0;
				for (size_t i = 0; i < prims.size(); ++i) {
					if (prims[i] && prims[i]->shape) {
						prims[i]->shape->extract(scene, primIdx);
						++primIdx;
					}
				}

				// Commit the changes
				rtcCommit(scene);

				/* create new scene */
				instance = new BackendSceneFlat(prims, scene);
			}

		public:
			bool dynamicScene = false;
			std::vector<Ref<Primitive>> prims;
		};

		/*! Construction of scene. */
		BackendSceneFlat(const std::vector<Ref<Primitive>>& geometry, RTCScene scene)
			: BackendScene(scene), geometry(geometry)
		{
			for (const auto &prim : geometry) {
				if (!prim) continue;
				if (prim->light) {
					add(prim->light);
				}
				if (prim->shape) {
					add(prim->shape);
				}
			}

			for (auto &light : allLights) {
				light->createShape(*this);
			}
		}

		/*! Helper to call the post intersector of the shape instance,
		 *  which will call the post intersector of the shape. */
		void postIntersect(const Ray& ray, DifferentialGeometry& dg) const {
			if (ray) geometry[ray.id0]->postIntersect(ray, dg);
		}

	private:
		std::vector<Ref<Primitive>> geometry;  //!< Geometry of the scene
	};
}

#endif
