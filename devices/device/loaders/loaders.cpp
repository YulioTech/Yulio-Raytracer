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

#include "loaders.h"
#include "sys/stl/string.h"
#include <map>

namespace embree
{
	std::string g_mesh_accel = "default";
	std::string g_mesh_builder = "default";
	std::string g_mesh_traverser = "default";

	static std::map<std::string, Handle<Device::RTImage>>* image_map = nullptr;

	Handle<Device::RTImage> rtLoadImage(const FileName &fileName)
	{
		if (image_map == nullptr)
			image_map = new std::map<std::string, Handle<Device::RTImage> >;

		if (image_map->find(fileName.str()) != image_map->end())
			return((*image_map)[fileName.str()]);

		return((*image_map)[fileName.str()] = g_device->rtNewImageFromFile(fileName.c_str()));
	}

	void rtClearImageCache() {
		if (image_map) delete image_map;
		image_map = nullptr;
	}

	static std::map<std::string, Handle<Device::RTTexture>>* texture_map = nullptr;

	Handle<Device::RTTexture> rtLoadTexture(const FileName &fileName, const std::string &filtering)
	{
		if (texture_map == nullptr)
			texture_map = new std::map<std::string, Handle<Device::RTTexture> >;

		if (texture_map->find(fileName.str()) != texture_map->end())
			return((*texture_map)[fileName.str()]);

		Handle<Device::RTTexture> texture = g_device->rtNewTexture(filtering.c_str());
		g_device->rtSetImage(texture, "image", rtLoadImage(fileName));
		g_device->rtCommit(texture);

		return((*texture_map)[fileName.str()] = texture);
	}

	void rtClearTextureCache() {
		if (texture_map) delete texture_map;
		texture_map = nullptr;
	}

	std::vector<Handle<Device::RTPrimitive>> rtLoadScene(const FileName &fileName, std::vector<Handle<Device::RTCamera>> *cameras)
	{
		std::string ext = strlwr(fileName.ext());
		if (ext == "obj") return loadOBJ(fileName);
		if (ext == "xml") return loadXML(fileName);
		if (ext == "dae") return loadDAE(fileName, *cameras);
		throw std::runtime_error("file format " + ext + " not supported");
	}
}
