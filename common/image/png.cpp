#include "image/image.h"

#include "../3rd party/LodePNG/lodepng.h"

namespace embree
{
	void storePNG(const Ref<Image> &image, const FileName &filename)
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

		std::vector<unsigned char> png;

		unsigned error = lodepng::encode(png, rgb, image->width, image->height, LCT_RGB, 8U);
		if (!error) {
			lodepng::save_file(png, filename);
		}
		else {
			std::cout << "encoder error " << error << ": " << lodepng_error_text(error) << std::endl;
			throw std::runtime_error("Unable to save \"" + filename.str() + "\".");
		}
	}
}