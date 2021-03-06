#include "image/image.h"

#include <FreeImage.h>

namespace embree
{

	Ref<Image> loadFreeImage(const uint8_t* data, const size_t dataLength, float scale, bool flipVertical, bool flipHorizontal) {
		FreeImage_Initialise();

		FIBITMAP *dib = nullptr;
		Ref<Image> image = null;
		FIMEMORY *mem = FreeImage_OpenMemory((BYTE*)data, dataLength);
		if (!mem) goto cleanup;

		FREE_IMAGE_FORMAT fif = FreeImage_GetFileTypeFromMemory(mem, 0);

		// check that the plug-in has reading capabilities ...
		if ((fif != FIF_UNKNOWN) && FreeImage_FIFSupportsReading(fif)) {
			dib = FreeImage_LoadFromMemory(fif, mem, 0);
		}

		if (!dib) goto cleanup;

		// Re-scale the image
		if (scale > 0.f && scale != 1.f) {
			const auto newWidth = max<unsigned int>(FreeImage_GetWidth(dib) * scale, 1);
			const auto newHeight = max<unsigned int>(FreeImage_GetHeight(dib) * scale, 1);
			FIBITMAP *newDib = FreeImage_Rescale(dib, newWidth, newHeight, FILTER_CATMULLROM);
			FreeImage_Unload(dib);
			dib = newDib;
		}

		if (flipVertical) {
			FreeImage_FlipVertical(dib);
		}

		if (flipHorizontal) {
			FreeImage_FlipHorizontal(dib);
		}

		const auto width = FreeImage_GetWidth(dib);
		const auto height = FreeImage_GetHeight(dib);
		const auto bpp = FreeImage_GetBPP(dib);

		/*! Allocate the Embree image. */
		image = new Image4c(width, height);

		/*! Convert the image from unsigned char RGB to unsigned char RGBA.
		Not, that the pixel layout used by FreeImage is OS dependant. Using a byte by byte memory
		order to label the pixel layout, then FreeImage uses a BGR[A] pixel layout under a Little
		Endian processor (Windows, Linux) and uses a RGB[A] pixel layout under a Big Endian
		processor (Mac OS X or any Big Endian Linux / Unix). This choice was made to ease the use
		of FreeImage with graphics API.
		This subtle difference is however transparent to the user. In order to make pixel access OS
		independent, FreeImage defines a set of macros used to set or get individual color
		components in a 24- or 32-bit DIB. */
		switch (bpp)
		{
		case 24: case 32: {
			unsigned char *bits = FreeImage_GetBits(dib);
			for (size_t y = 0, i = 0; y < height; ++y) {
				for (size_t x = 0; x < width; ++x) {
					const float r = (float)bits[i + FI_RGBA_RED] / 255.0f;
					const float g = (float)bits[i + FI_RGBA_GREEN] / 255.0f;
					const float b = (float)bits[i + FI_RGBA_BLUE] / 255.0f;

					// Handle the alpha
					float a = 1.f;
					if (bpp == 32) {
						a = (float)bits[i + FI_RGBA_ALPHA] / 255.0f;
						i += 4;
					}
					else {
						i += 3;
					}

					image->set(x, y, Color4(r, g, b, a));
				}
			}
		}
				 break;

		default:
			// ToDo: need to work on supporting other BPP values. For now just skip the image.
			break;
		}

		cleanup:
		// Clean up
		if (dib) FreeImage_Unload(dib);
		if (mem) FreeImage_CloseMemory(mem);
		FreeImage_DeInitialise();

		return image;
	}

	Ref<Image> loadFreeImage(const FileName& fileName, float scale, bool flipVertical) {
		
		FreeImage_Initialise();

		// check the file signature and deduce its format
		// (the second argument is currently not used by FreeImage)
		FREE_IMAGE_FORMAT fif = FreeImage_GetFileType(fileName.c_str(), 0);
		if (fif == FIF_UNKNOWN) {
			// no signature ?
			// try to guess the file format from the file extension
			fif = FreeImage_GetFIFFromFilename(fileName.c_str());
		}

		FIBITMAP *dib = nullptr;
		Ref<Image> image = null;

		// check that the plug-in has reading capabilities ...
		if ((fif != FIF_UNKNOWN) && FreeImage_FIFSupportsReading(fif)) {
			// ok, let's load the file
			dib = FreeImage_Load(fif, fileName.c_str(), 0);
			// unless a bad file format, we are done !
		}

		if (!dib) goto cleanup;

		// Re-scale the image
		if (scale > 0.f && scale != 1.f) {
			const auto newWidth = max<unsigned int>(FreeImage_GetWidth(dib) * scale, 1);
			const auto newHeight = max<unsigned int>(FreeImage_GetHeight(dib) * scale, 1);
			FIBITMAP *newDib = FreeImage_Rescale(dib, newWidth, newHeight, FILTER_CATMULLROM);
			FreeImage_Unload(dib);
			dib = newDib;
		}

		if (flipVertical) {
			FreeImage_FlipVertical(dib);
		}

		const auto width = FreeImage_GetWidth(dib);
		const auto height = FreeImage_GetHeight(dib);
		const auto bpp = FreeImage_GetBPP(dib);

		/*! Allocate the Embree image. */
		image = new Image4c(width, height, fileName);

		/*! Convert the image from unsigned char RGB to unsigned char RGBA.
		 Not, that the pixel layout used by FreeImage is OS dependant. Using a byte by byte memory
		order to label the pixel layout, then FreeImage uses a BGR[A] pixel layout under a Little
		Endian processor (Windows, Linux) and uses a RGB[A] pixel layout under a Big Endian
		processor (Mac OS X or any Big Endian Linux / Unix). This choice was made to ease the use
		of FreeImage with graphics API.
		This subtle difference is however transparent to the user. In order to make pixel access OS
		independent, FreeImage defines a set of macros used to set or get individual color
		components in a 24- or 32-bit DIB. */
		switch (bpp)
		{
		case 24: case 32: {
			unsigned char *bits = FreeImage_GetBits(dib);
			for (size_t y = 0, i = 0; y < height; ++y) {
				for (size_t x = 0; x < width; ++x) {
					const float r = (float)bits[i + FI_RGBA_RED] / 255.0f;
					const float g = (float)bits[i + FI_RGBA_GREEN] / 255.0f;
					const float b = (float)bits[i + FI_RGBA_BLUE] / 255.0f;
					
					// Handle the alpha
					float a = 1.f;
					if (bpp == 32) {
						a = (float)bits[i + FI_RGBA_ALPHA] / 255.0f;
						i += 4;
					}
					else{
						i += 3;
					}

					image->set(x, y, Color4(r, g, b, a));
				}
			}
		}
			break;

		default:
			// ToDo: need to work on supporting other BPP values. For now just skip the image.
			break;
		}

		// Clean up
		cleanup:
		if (dib) FreeImage_Unload(dib);
		FreeImage_DeInitialise();

		return image;
	}

	bool storeFreeImage(const Ref<Image>& image, const FileName& fileName, int quality) {

		bool bSuccess = false;

		FreeImage_Initialise();

		const int bpp = 24;
		FIBITMAP *dib = FreeImage_Allocate(image->width, image->height, bpp);

		if (!dib)
			return bSuccess;

		RGBQUAD color;
		for (size_t y = 0, yFlip = image->height - 1, i = 0; y < image->height; y++, yFlip--) {
			for (size_t x = 0; x < image->width; x++) {
				const Color4 pixel = image->get(x, yFlip); // Flip the image vertically
				color.rgbRed = (unsigned char)(clamp(pixel.r) * 255.f);
				color.rgbGreen = (unsigned char)(clamp(pixel.g) * 255.f);
				color.rgbBlue = (unsigned char)(clamp(pixel.b) * 255.f);
				FreeImage_SetPixelColor(dib, x, y, &color);
			}
		}

		// try to guess the file format from the file extension
		FREE_IMAGE_FORMAT fif = FreeImage_GetFIFFromFilename(fileName.c_str());
		if (fif != FIF_UNKNOWN) {
			// check that the plug-in has sufficient writing and export capabilities ...
			if (FreeImage_FIFSupportsWriting(fif) && FreeImage_FIFSupportsExportBPP(fif, bpp)) {
				// ok, we can save the file
				bSuccess = FreeImage_Save(fif, dib, fileName.c_str(), fif == FIF_JPEG ? quality : 0);
				// unless an abnormal bug, we are done !
			}
		}

		// Clean up
		FreeImage_Unload(dib);
		FreeImage_DeInitialise();

		return bSuccess;
	}

} // namespace embree
