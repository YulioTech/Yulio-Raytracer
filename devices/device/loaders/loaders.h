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

#ifndef __EMBREE_LOADERS_H__
#define __EMBREE_LOADERS_H__

#include "obj_loader.h"
#include "xml_loader.h"
#include "ColladaLoader.h"

namespace embree
{
	extern std::string g_mesh_accel;
	extern std::string g_mesh_builder;
	extern std::string g_mesh_traverser;

	Handle<Device::RTImage> rtLoadImage(const FileName& fileName);
	void rtClearImageCache();

	Handle<Device::RTTexture> rtLoadTexture(const FileName& fileName, const std::string &filtering = "bilinear", bool invert = false);
	void rtClearTextureCache();

	std::vector<Handle<Device::RTPrimitive>> rtLoadScene(const FileName& fileName, std::vector<Handle<Device::RTCamera>> *cameras = nullptr, const std::string& faceCullingMode = "default");
}

#endif
