#pragma once

#include "image/image.h"
#include "../textures/texture.h"

namespace embree
{
	/*! Implements an image mapped texture with nearest neighbor lookup. */
	class Bilinear : public Texture
	{
	public:

		/*! Construction from image. */
		Bilinear(const Ref<Image>& image)
			: image(image) {}

		/*! Construction from parameters. */
		Bilinear(const Parms& parms) {
			image = parms.getImage("image");
			invert = parms.getBool("invert", false);
		}

		__forceinline Color4 get(const Vec2f &p) const {
			const float s1 = p.x - floor(p.x), t1 = p.y - floor(p.y);
			const float u = s1 * image->width - .5f;
			const float v = t1 * image->height - .5f;
			const int x = clamp(int(floorf(u)), int(0), int(image->width - 2));
			const int y = clamp(int(floorf(v)), int(0), int(image->height - 2));
			const float u_ratio = u - x;
			const float v_ratio = v - y;
			const float u_opposite = 1.f - u_ratio;
			const float v_opposite = 1.f - v_ratio;
			const Color4 c = (image->get(x, y) * u_opposite + image->get(x + 1, y) * u_ratio) * v_opposite +
				(image->get(x, y + 1) * u_opposite + image->get(x + 1, y + 1) * u_ratio) * v_ratio;

			//const Color4 c = (image->get(x, y) * u_opposite + image->get(x + 1 > image->width - 1 ? x : x + 1, y) * u_ratio) * v_opposite +
			//	(image->get(x, y + 1 > image->height - 1 ? y : y + 1) * u_opposite + image->get(x + 1 > image->width - 1 ? x : x + 1, y + 1 > image->height - 1 ? y : y + 1) * u_ratio) * v_ratio;

			return invert ? Color4(1.f) - c : c;
		}

	protected:
		Ref<Image> image; //!< Image mapped to surface.
		bool invert = false;
	};
}