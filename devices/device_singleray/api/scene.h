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

#ifndef __EMBREE_BACKEND_SCENE_H__
#define __EMBREE_BACKEND_SCENE_H__

#include "handle.h"
#include "instance.h"

#include "../lights/light.h"
#include "../shapes/differentialgeometry.h"

/*! include interface to ray tracing core */
#include <embree2/rtcore.h>

namespace embree
{
	/*! Scene holding all geometry and lights. */
	class BackendScene : public RefCount
	{
	public:

		class Handle : public InstanceHandle<BackendScene> {
			ALIGNED_CLASS;
		public:

			Handle() : accelTy("default"), builderTy("default"), traverserTy("default") {}

			void set(const std::string& property, const Variant& data)
			{
				if (property == "accel") accelTy = data.getString();
				else if (property == "builder") builderTy = data.getString();
				else if (property == "traverser") traverserTy = data.getString();
			}

			void get(const std::string& property, Variant& data) {
				if (property == "accel") data = accelTy;
				else if (property == "builder") data = builderTy;
				else if (property == "traverser") data = traverserTy;
			}

			virtual void setPrimitive(size_t slot, Ref<PrimitiveHandle> prim) = 0;
			virtual void updatePrimitive(size_t slot, Ref<PrimitiveHandle> prim, const AffineSpace3f &transform) = 0;

		public:
			std::string accelTy;
			std::string builderTy;
			std::string traverserTy;
		};

	public:

		BackendScene(RTCScene scene)
			: scene(scene) {}

		~BackendScene() {
			if (scene) rtcDeleteScene(scene);
		}

		/*! Adds a light to the scene. */
		void add(const Ref<Light>& light) {
			allLights.push_back(light);
			if (Ref<EnvironmentLight> envlight = dynamic_cast<EnvironmentLight*>(light.ptr)) envLights.push_back(envlight);
		}

		void add(const Ref<Shape>& shape) {
			allShapes.push_back(shape);
			sceneBBox = merge(sceneBBox, shape->bbox());
		}

		/*! Helper to call the post intersector of the shape instance,
		 *  which will call the post intersector of the shape. */
		virtual void postIntersect(const Ray& ray, DifferentialGeometry& dg) const = 0;

	public:
		std::vector<Ref<Light>> allLights;              //!< All lights of the scene
		std::vector<Ref<Shape>> allShapes;              //!< All shapes of the scene
		std::vector<Ref<EnvironmentLight>> envLights;   //!< Environment lights of the scene
		RTCScene scene;
		BBox3f sceneBBox = empty;
	};
}

#endif
