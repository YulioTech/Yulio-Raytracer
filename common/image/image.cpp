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

#include "image.h"
#include "sys/stl/string.h"

#include <map>
#include <iostream>

namespace embree
{
	/*! loads an image from a file with auto-detection of format */
	Ref<Image> loadImageFromDisk(const FileName& fileName) try
	{
		std::string ext = std::strlwr(fileName.ext());
#ifdef USE_OPENEXR
		if (ext == "exr") return loadExr(fileName);
#endif
#ifdef USE_IMAGEMAGICK
		if (ext == "bmp") return loadMagick(fileName);
		if (ext == "gif") return loadMagick(fileName);
		if (ext == "png") return loadMagick(fileName);
		if (ext == "tga") return loadMagick(fileName);
		if (ext == "tif") return loadMagick(fileName);
		if (ext == "tiff") return loadMagick(fileName);
#endif
#ifdef USE_LIBJPEG_TURBO
		// Lev: Note, that FreeImage v3.17.0 (see the call below) appears to have a bug that causes JPEG images of certain sizes to be loaded improperly (they appear skewed and/or discolored).
		// So, for the time being, for loading JPEGs in particular, we'll be using libjpeg-turbo instead.
		if (ext == "jpg" || ext == "jpeg") return loadJPEG(fileName);
#endif
		if (ext == "pfm") return loadPFM(fileName);
		if (ext == "ppm") return loadPPM(fileName);

		// As a last resort, try FreeImage - a universal image loader
		auto image = loadFreeImage(fileName);
		if (image != null) return image;

		throw std::runtime_error("image format " + ext + " not supported");
	}
	catch (const std::exception& e) {
		std::cout << "cannot read file " << fileName << ": " << e.what() << std::endl;
		return null;
	}

	/*! loads an image from a file with auto-detection of format */
	static std::map<std::string, Ref<Image>> image_cache;
	Ref<Image> loadImage(const FileName& fileName, bool cache)
	{
		if (!cache)
			return loadImageFromDisk(fileName);

		if (image_cache.find(fileName) == image_cache.end())
			image_cache[fileName] = loadImageFromDisk(fileName);

		return image_cache[fileName];
	}

	void clearImageCache() {
		image_cache.clear();
	}

	/*! stores an image to file with auto-detection of format */
	void storeImage(const Ref<Image>& img, const FileName& fileName, int quality) try
	{
		std::string ext = strlwr(fileName.ext());
#ifdef USE_OPENEXR
		if (ext == "exr") { storeExr(img, fileName);  return; }
#endif
#ifdef USE_IMAGEMAGICK
		if (ext == "bmp") { storeMagick(img, fileName);  return; }
		if (ext == "gif") { storeMagick(img, fileName);  return; }
		if (ext == "png") { storeMagick(img, fileName);  return; }
		if (ext == "tif") { storeMagick(img, fileName);  return; }
		if (ext == "tiff") { storeMagick(img, fileName);  return; }
#endif
#ifdef USE_DEVIL
		if (ext == "png") { storeDevIL(img, fileName);  return; }
#endif
//#ifdef USE_LIBJPEG
//		if (ext == "jpg") { storeJPEG(img, fileName, quality);  return; }
//#endif
		if (ext == "png") { storePNG(img, fileName);  return; }
		if (ext == "pfm") { storePFM(img, fileName);  return; }
		if (ext == "ppm") { storePPM(img, fileName);  return; }
		if (ext == "tga") { storeTga(img, fileName);  return; }

		// As a last resort, try FreeImage - a universal image saver
		if (!storeFreeImage(img, fileName, quality))
			throw std::runtime_error("image format " + ext + " not supported");
	}
	catch (const std::exception& e) {
		std::cout << "cannot write file " << fileName << ": " << e.what() << std::endl;
	}

	/*! template instantiations */
	template class ImageT<Col3c>;
	template class ImageT<Col3f>;

}
