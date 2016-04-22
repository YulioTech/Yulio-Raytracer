#ifdef USE_DEVIL

#include "image/image.h"

#include <IL/il.h>
#include <IL/ilut.h>

// Lev: ToDo - needs to be relative or configured in the project settings 
#pragma comment(lib,"e:/Documents/My Code/Ray Tracing/CPU/embree-renderer-2.3.2/3rd party/DevIL/lib/x64/DevIL.lib" )

namespace embree
{
	void storeDevIL(const Ref<Image> &image, const FileName &filename)
	{
		/*! Allocate storage for the uncompressed packed image. */
		std::vector<unsigned char> rgb(image->height * image->width * 3, 0U);

		/*! Convert the image to unsigned char RGB. */
		for (size_t y = 0, i = 0; y < image->height; y++) {
			for (size_t x = 0; x < image->width; x++) {
				const Color4 pixel = image->get(x, y);
				rgb[i++] = (unsigned char)(clamp(pixel.r) * 255.0f);
				rgb[i++] = (unsigned char)(clamp(pixel.g) * 255.0f);
				rgb[i++] = (unsigned char)(clamp(pixel.b) * 255.0f);
			}
		}

		// Save the image via DevIL
		{
			if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
			{
				/// wrong DevIL version ///
				std::string err_msg = "Wrong DevIL version. Old devil.dll in system32/SysWow64?";
				char* cErr_msg = (char *)err_msg.c_str();
				return;
			}

			// Initialization of DevIL
			ilInit();

			ILuint imageId;
			ilGenImages(1, &imageId);
			ilBindImage(imageId);

			ilTexImage(image->width, image->height, 1, 3U, IL_RGB, IL_UNSIGNED_BYTE, &rgb[0]);

			ilEnable(IL_FILE_OVERWRITE);
			ilResetWrite();
			auto res = ilSaveImage(filename.c_str());

			ilDeleteImages(1, &imageId);
		}
	}
}
#endif //USE_DEVIL